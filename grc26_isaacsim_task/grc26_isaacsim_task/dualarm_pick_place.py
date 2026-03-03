#!/usr/bin/python3

import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
import rclpy.time
import rclpy.duration

from moveit_client.action import MoveToPose, MoveToCartesianPose, GripperCommand

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from pydantic import BaseModel
from typing import Optional

from coord_dsl.fsm import fsm_step, FSMData
from coord_dsl.event_loop import (
    produce_event,
    consume_event,
    reconfig_event_buffers,
)
from models.fsm import create_fsm, EventID, StateID


OBJECT_LINK = 'cutting_board_A'
GRASP_LINK_1 = 'grasp_point_1'
GRASP_LINK_2 = 'grasp_point_2'

GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.8

class Manipulator(BaseModel):
    arm: str
    gripper: str
    ee_link: str

class Robot(BaseModel):
    arm1: Manipulator
    arm2: Manipulator

DEFAULT_ARM1 = Manipulator(
    arm='kinova1',
    gripper='g1',
    ee_link='kinova1_robotiq_85_grasp_link',
)

DEFAULT_ARM2 = Manipulator(
    arm='kinova2',
    gripper='g2',
    ee_link='kinova2_robotiq_85_grasp_link',
)

class ActionFuture(BaseModel):
    send_goal_future: Optional[rclpy.task.Future]  = None
    goal_handle: Optional[ClientGoalHandle]        = None
    get_result_future: Optional[rclpy.task.Future] = None

    class Config:
        arbitrary_types_allowed = True


class UserData(BaseModel):
    arm1_af: ActionFuture                 = ActionFuture()
    arm2_af: ActionFuture                 = ActionFuture()
    g1_af: ActionFuture                   = ActionFuture()
    g2_af: ActionFuture                   = ActionFuture()
    move_arm1: bool                       = False
    move_arm2: bool                       = False
    move_gripper1: bool                   = False
    move_gripper2: bool                   = False
    arm1_done: bool                       = False
    arm2_done: bool                       = False
    g1_done: bool                         = False
    g2_done: bool                         = False
    arm1_target_pose: Pose                = None
    arm2_target_pose: Pose                = None
    arm1_gripper_open: bool               = True
    arm2_gripper_open: bool               = True

    m1: bool                              = False
    m2: bool                              = False
    m3: bool                              = False
    m4: bool                              = False
    
    class Config:
        arbitrary_types_allowed = True


class DualArmPickPlace(Node):
    def __init__(self):
        super().__init__('dual_arm_pick_place')
        self.logger = self.get_logger()
        self.logger.info('DualArmPickPlace node started.')

    def configure(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('arm1',     DEFAULT_ARM1.arm)
        self.declare_parameter('gripper1', DEFAULT_ARM1.gripper)
        self.declare_parameter('ee_link1', DEFAULT_ARM1.ee_link)
        self.declare_parameter('arm2',     DEFAULT_ARM2.arm)
        self.declare_parameter('gripper2', DEFAULT_ARM2.gripper)
        self.declare_parameter('ee_link2', DEFAULT_ARM2.ee_link)

        self.robot = Robot(
            arm1=Manipulator(
                arm=self.get_parameter('arm1').value,
                gripper=self.get_parameter('gripper1').value,
                ee_link=self.get_parameter('ee_link1').value,
            ),
            arm2=Manipulator(
                arm=self.get_parameter('arm2').value,
                gripper=self.get_parameter('gripper2').value,
                ee_link=self.get_parameter('ee_link2').value,
            )
        )
    
        arm1_action_name = f'/{self.robot.arm1.arm}/move_to_pose'
        arm2_action_name = f'/{self.robot.arm2.arm}/move_to_pose'
        g1_action_name   = f'/{self.robot.arm1.gripper}/gripper_command'
        g2_action_name   = f'/{self.robot.arm2.gripper}/gripper_command'
        
        self.arm1_ac = ActionClient(self, MoveToPose, arm1_action_name)
        self.arm2_ac = ActionClient(self, MoveToPose, arm2_action_name)
        self.g1_ac   = ActionClient(self, GripperCommand, g1_action_name)
        self.g2_ac   = ActionClient(self, GripperCommand, g2_action_name)

        self.arm1_target_pub = self.create_publisher(PoseStamped, 'arm1_target_pose', 10)
        self.arm2_target_pub = self.create_publisher(PoseStamped, 'arm2_target_pose', 10)

        self.get_logger().info('DualArmPickPlace node configured.')

    def execute_action(self, ac: ActionClient, af: ActionFuture, goal_msg):
        '''
        Generic action execution function.
        Call this function repeatedly until it returns True.

        Parameters:
        - ac: ActionClient
        - af: ActionFuture
        - goal_msg: Goal message to send
        '''

        assert ac is not None, "ActionClient is None"
        assert af is not None, "ActionFuture is None"
        assert goal_msg is not None, "Goal message is None"
    
        # check server availability
        if not ac.server_is_ready():
            self.logger.warning('action server not available')
            return False

        # send goal
        if af.send_goal_future is None:
            self.logger.info(f'{ac._action_name}: Sending goal to action server...')
            af.send_goal_future = ac.send_goal_async(goal_msg)
            assert af.send_goal_future is not None, "send_goal_future is None"
            return False

        # wait for goal to be accepted
        if not af.send_goal_future.done():
            self.logger.info(f'{ac._action_name}: Waiting for goal to be accepted...')
            return False

        if af.goal_handle is None:
            af.goal_handle = af.send_goal_future.result()
            if not af.goal_handle or not af.goal_handle.accepted:
                self.logger.error(f'{ac._action_name}: Goal rejected')
                af.send_goal_future = None
                return False

            self.logger.info(f'{ac._action_name}: Goal accepted, waiting for result...')
            af.get_result_future = af.goal_handle.get_result_async()
            return False

        # wait for result
        if af.get_result_future is None:
            self.logger.warning(f'{ac._action_name}: get_result_future is None')
            return False

        if not af.get_result_future.done():
            self.logger.info(f'{ac._action_name}: Waiting for result...', throttle_duration_sec=5.0)
            return False

        result = af.get_result_future.result()
        self.logger.info(f'{ac._action_name}: Action result: {result.result}')
        
        # reset action future for next use
        self.reset_action_future(af)
        return True

    def reset_action_future(self, af: ActionFuture):
        assert af is not None, "ActionFuture is None"

        # cleanup
        af.send_goal_future = None
        af.get_result_future = None
        af.goal_handle = None

    def _lookup_pose(self, link: str) -> PoseStamped | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                'world', link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as exc:
            self.get_logger().error(f'TF lookup failed for {link}: {exc}')
            return None

        pose = PoseStamped()
        pose.header.frame_id  = 'world'
        pose.header.stamp     = self.get_clock().now().to_msg()
        pose.pose.position.x  = tf.transform.translation.x
        pose.pose.position.y  = tf.transform.translation.y
        pose.pose.position.z  = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def get_move_arm_msg(self, target_pose:Pose) -> MoveToPose.Goal:
        msg = MoveToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = target_pose
        msg.target_pose = pose

        return msg

    def get_gripper_cmd_msg(self, position: float):
        msg = GripperCommand.Goal()
        msg.gripper_width = position
        return msg

    def move_arm(self, ac: ActionClient,
                 af: ActionFuture, target_pose: Pose):
        move_arm_msg = self.get_move_arm_msg(target_pose)
        # send goal to arm
        if not self.execute_action(ac, af, move_arm_msg):
            return False

        return True

    def gripper_control(self, ac, af, open: bool):
        val = 0.0 if open else 0.8
        print(f"Gripper command: {'OPEN' if open else 'CLOSE'} (width={val})")

        gripper_msg = self.get_gripper_cmd_msg(val)
        # send goal to gripper
        if not self.execute_action(ac, af, gripper_msg):
            return False

        return True
    
def configure_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    node.configure()
    return True

def idle_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    ud.m1 = True
    produce_event(fsm.event_data, EventID.E_M_TOUCH_TABLE_CONFIG)
    return True

def m_touch_table_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    # obj_pose = node._lookup_pose(OBJECT_LINK)
    k1_target_pose = node._lookup_pose(GRASP_LINK_1)
    k2_target_pose = node._lookup_pose(GRASP_LINK_2)
    
    if k1_target_pose is None or k2_target_pose is None:
        node.get_logger().error('Failed to lookup target poses')
        return False

    # adjust to be behind the object for grasping
    approach_offset = 0.075  # 10 cm
    k1_target_pose.pose.position.x -= approach_offset
    k2_target_pose.pose.position.x += approach_offset

    k1_target_pose.pose.position.z += 0.0
    k2_target_pose.pose.position.z += 0.0
    
    ud.arm1_target_pose = k1_target_pose.pose
    ud.arm2_target_pose = k2_target_pose.pose

    # publish target poses for visualization/debugging
    node.arm1_target_pub.publish(k1_target_pose)
    node.arm2_target_pub.publish(k2_target_pose)

    ud.move_arm1 = True
    ud.move_arm2 = True

    return True

def m_slide_along_table_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    k1_target_pose = node._lookup_pose(GRASP_LINK_1)
    k2_target_pose = node._lookup_pose(GRASP_LINK_2)

    if k1_target_pose is None or k2_target_pose is None:
        node.get_logger().error('Failed to lookup target poses')
        return False

    ud.arm1_target_pose = k1_target_pose.pose
    ud.arm2_target_pose = k2_target_pose.pose

    node.arm1_target_pub.publish(k1_target_pose)
    node.arm2_target_pub.publish(k2_target_pose)

    ud.move_arm1 = True
    ud.move_arm2 = True

    return True

def m_grasp_object_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    
    ud.arm1_gripper_open = False
    ud.arm2_gripper_open = False

    ud.move_arm1 = False
    ud.move_arm2 = False
    ud.move_gripper1 = True
    ud.move_gripper2 = True

    return True

def m_collaborate_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    
    k1_target_pose = node._lookup_pose(GRASP_LINK_1)
    k2_target_pose = node._lookup_pose(GRASP_LINK_2)

    if k1_target_pose is None or k2_target_pose is None:
        node.get_logger().error('Failed to lookup target poses')
        return False

    k1_target_pose.pose.position.z += 0.2
    k2_target_pose.pose.position.z += 0.2

    ud.arm1_target_pose = k1_target_pose.pose
    ud.arm2_target_pose = k2_target_pose.pose

    ud.move_arm1 = True
    ud.move_arm2 = True
    ud.move_gripper1 = False
    ud.move_gripper2 = False

    return True

def execute_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    if ud.move_arm1 and not ud.arm1_done:
        ud.arm1_done = node.move_arm(node.arm1_ac, ud.arm1_af, ud.arm1_target_pose)
    if ud.move_arm2 and not ud.arm2_done:
        ud.arm2_done = node.move_arm(node.arm2_ac, ud.arm2_af, ud.arm2_target_pose)
    if ud.move_gripper1 and not ud.g1_done:
        ud.g1_done = node.gripper_control(node.g1_ac, ud.g1_af, ud.arm1_gripper_open)
    if ud.move_gripper2 and not ud.g2_done:
        ud.g2_done = node.gripper_control(node.g2_ac, ud.g2_af, ud.arm2_gripper_open)

    all_done = (
        (not ud.move_arm1    or ud.arm1_done) and
        (not ud.move_arm2    or ud.arm2_done) and
        (not ud.move_gripper1 or ud.g1_done)  and
        (not ud.move_gripper2 or ud.g2_done)
    )
    if all_done:
        # reset for next step
        ud.move_arm1 = False
        ud.move_arm2 = False
        ud.move_gripper1 = False
        ud.move_gripper2 = False
        ud.arm1_done = False
        ud.arm2_done = False
        ud.g1_done = False
        ud.g2_done = False

        if ud.m1:
            ud.m1 = False
            ud.m2 = True
            produce_event(fsm.event_data, EventID.E_M_SLIDE_ALONG_TABLE_CONFIG)
        elif ud.m2:
            ud.m2 = False
            ud.m3 = True
            produce_event(fsm.event_data, EventID.E_M_GRASP_OBJECT_CONFIG)
        elif ud.m3:
            ud.m3 = False
            ud.m4 = True
            produce_event(fsm.event_data, EventID.E_M_COLLABORATE_CONFIG)
        elif ud.m4:
            ud.m4 = False
    return all_done

def generic_on_end(fsm: FSMData, ud: UserData, end_events: list[EventID]):
    print(f"State '{StateID(fsm.current_state_index).name}' finished")
    for evt in end_events:
        produce_event(fsm.event_data, evt)

FSM_BHV = {
    StateID.S_CONFIGURE: {
        "step": configure_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_CONFIGURED]
        ),
    },
    StateID.S_IDLE: {
        "step": idle_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_STEP]
        ),
    },
    StateID.S_M_TOUCH_TABLE: {
        "step": m_touch_table_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_M_TOUCH_TABLE_CONFIGURED]
        ),
    },
    StateID.S_M_SLIDE_ALONG_TABLE: {
        "step": m_slide_along_table_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_M_SLIDE_ALONG_TABLE_CONFIGURED]
        ),
    },
    StateID.S_M_GRASP_OBJECT: {
        "step": m_grasp_object_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_M_GRASP_OBJECT_CONFIGURED]
        ),
    },
    StateID.S_M_COLLABORATE: {
        "step": m_collaborate_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_M_COLLABORATE_CONFIGURED]
        ),
    },
    StateID.S_EXECUTE: {
        "step": execute_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_STEP]
        ),
    },
}

def fsm_behavior(fsm: FSMData, ud: UserData, bhv_data: dict[StateID, dict], node: DualArmPickPlace):
    cs = fsm.current_state_index
    if cs not in bhv_data:
        return

    bhv_data_cs = bhv_data[StateID(cs)]
    
    assert "step" in bhv_data_cs, f"Missing 'step' in behavior data for state {StateID(cs)}"
    if not bhv_data_cs["step"](fsm, ud, node):
        return

    if "on_end" in bhv_data_cs:
        bhv_data_cs["on_end"](fsm, ud, node)


def signal_handler(sig, frame):
    print("You pressed Ctrl+C! Exiting gracefully...")
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal_handler)

    node = DualArmPickPlace()

    ud = UserData()
    fsm = create_fsm()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    while rclpy.ok():
        executor.spin_once(timeout_sec=0.01)

        if fsm.current_state_index == StateID.S_EXIT:
            node.get_logger().info("Exiting FSM loop.")
            break

        reconfig_event_buffers(fsm.event_data)
        produce_event(fsm.event_data, EventID.E_STEP)
        fsm_behavior(fsm, ud, FSM_BHV, node)

        reconfig_event_buffers(fsm.event_data)
        fsm_step(fsm)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

