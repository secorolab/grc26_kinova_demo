#!/usr/bin/python3

import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
import rclpy.time
import rclpy.duration

from moveit_client.action import (
    MoveToPose,
    MoveToCartesianPose,
    MoveToNamedTarget,
    GripperCommand
)

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from pydantic import BaseModel
from typing import Optional
from enum import StrEnum

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
    gripper_joint: str

class Robot(BaseModel):
    arm1: Manipulator
    arm2: Manipulator

DEFAULT_ARM1 = Manipulator(
    arm='kinova1',
    gripper='g1',
    ee_link='kinova1_robotiq_85_grasp_link',
    gripper_joint='kinova1_robotiq_85_left_knuckle_joint',
)

DEFAULT_ARM2 = Manipulator(
    arm='kinova2',
    gripper='g2',
    ee_link='kinova2_robotiq_85_grasp_link',
    gripper_joint='kinova2_robotiq_85_left_knuckle_joint',
)

class ActionFuture(BaseModel):
    send_goal_future: Optional[rclpy.task.Future]  = None
    goal_handle: Optional[ClientGoalHandle]        = None
    get_result_future: Optional[rclpy.task.Future] = None

    class Config:
        arbitrary_types_allowed = True

class ArmActionClientType(StrEnum):
    POSE      = 'pose'
    CARTESIAN = 'cartesian'
    NAMED     = 'named'

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

    arm1_end_pose: Pose                   = None
    arm2_end_pose: Pose                   = None

    max_vel_sf: float                     = 0.0
    max_acc_sf: float                     = 0.0

    arm_action_type: ArmActionClientType  = ArmActionClientType.CARTESIAN

    m1: bool                              = False
    m2: bool                              = False
    m3: bool                              = False
    m4: bool                              = False
    m5: bool                              = False
    m6: bool                              = False
    
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
        self.declare_parameter('gripper_joint1', DEFAULT_ARM1.gripper_joint)
        self.declare_parameter('arm2',     DEFAULT_ARM2.arm)
        self.declare_parameter('gripper2', DEFAULT_ARM2.gripper)
        self.declare_parameter('ee_link2', DEFAULT_ARM2.ee_link)
        self.declare_parameter('gripper_joint2', DEFAULT_ARM2.gripper_joint)

        self.robot = Robot(
            arm1=Manipulator(
                arm=self.get_parameter('arm1').value,
                gripper=self.get_parameter('gripper1').value,
                ee_link=self.get_parameter('ee_link1').value,
                gripper_joint=self.get_parameter('gripper_joint1').value,
            ),
            arm2=Manipulator(
                arm=self.get_parameter('arm2').value,
                gripper=self.get_parameter('gripper2').value,
                ee_link=self.get_parameter('ee_link2').value,
                gripper_joint=self.get_parameter('gripper_joint2').value,
            )
        )
    
        arm1_action_name = f'/{self.robot.arm1.arm}/move_to_pose'
        arm2_action_name = f'/{self.robot.arm2.arm}/move_to_pose'
        g1_action_name   = f'/{self.robot.arm1.gripper}/gripper_command'
        g2_action_name   = f'/{self.robot.arm2.gripper}/gripper_command'
        
        self.arm1_pose_ac = ActionClient(self, MoveToPose, arm1_action_name)
        self.arm2_pose_ac = ActionClient(self, MoveToPose, arm2_action_name)
        
        self.arm1_cart_ac = ActionClient(self, MoveToCartesianPose, f'/{self.robot.arm1.arm}/move_to_cartesian_pose')
        self.arm2_cart_ac = ActionClient(self, MoveToCartesianPose, f'/{self.robot.arm2.arm}/move_to_cartesian_pose')

        self.arm1_named_ac = ActionClient(self, MoveToNamedTarget, f'/{self.robot.arm1.arm}/move_to_named_target')
        self.arm2_named_ac = ActionClient(self, MoveToNamedTarget, f'/{self.robot.arm2.arm}/move_to_named_target')

        self.arm1_ac = self.arm1_cart_ac
        self.arm2_ac = self.arm2_cart_ac

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
            self.logger.warning(f'{ac._action_name}: Action server not available, waiting...', throttle_duration_sec=5.0)
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

    def get_move_cartesian_msg(self, target_pose:Pose,
                               max_vel_sf: float, max_acc_sf: float) -> MoveToCartesianPose.Goal:
        msg = MoveToCartesianPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = target_pose
        
        msg.waypoints.append(pose)

        msg.max_velocity_scaling_factor = max_vel_sf
        msg.max_acceleration_scaling_factor = max_acc_sf

        return msg

    def get_move_named_msg(self, target_name: str = 'home') -> MoveToNamedTarget.Goal:
        msg = MoveToNamedTarget.Goal()
        msg.pose_name = target_name
        return msg

    def get_gripper_cmd_msg(self, position: float, name: str) -> GripperCommand.Goal:
        msg = GripperCommand.Goal()
        msg.gripper_width = position
        msg.joint_name = name
        return msg

    def move_arm(self, ac: ActionClient, af: ActionFuture, 
                 target_pose: Pose, action_type: ArmActionClientType,
                 max_vel_sf: float, max_acc_sf: float) -> bool:
        if action_type == ArmActionClientType.POSE:
            move_arm_msg = self.get_move_arm_msg(target_pose)
        elif action_type == ArmActionClientType.CARTESIAN:
            move_arm_msg = self.get_move_cartesian_msg(target_pose, max_vel_sf, max_acc_sf)
        elif action_type == ArmActionClientType.NAMED:
            move_arm_msg = self.get_move_named_msg()
        else:
            self.logger.error(f'Unknown action type: {action_type}')
            return False

        # send goal to arm
        if not self.execute_action(ac, af, move_arm_msg):
            return False

        return True

    def gripper_control(self, ac, af, open: bool, joint_name: str):
        val = 0.0 if open else 0.8
        gripper_msg = self.get_gripper_cmd_msg(val, joint_name)
        # send goal to gripper
        if not self.execute_action(ac, af, gripper_msg):
            return False

        return True
    
def configure_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    node.configure()
    return True

def idle_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    ud.m1 = True
    produce_event(fsm.event_data, EventID.E_M_HOME_CONFIG)
    return True

def m_home_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    ud.arm_action_type = ArmActionClientType.NAMED
    
    ud.move_arm1 = True
    ud.move_arm2 = True

    node.arm1_ac = node.arm1_named_ac
    node.arm2_ac = node.arm2_named_ac

    return True

def m_open_gripper_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    ud.move_gripper1 = True
    ud.move_gripper2 = True

    ud.arm1_gripper_open = True
    ud.arm2_gripper_open = True

    return True

def m_touch_table_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    # obj_pose = node._lookup_pose(OBJECT_LINK)
    k1_target_pose = node._lookup_pose(GRASP_LINK_1)
    k2_target_pose = node._lookup_pose(GRASP_LINK_2)
    
    if k1_target_pose is None or k2_target_pose is None:
        node.get_logger().error('Failed to lookup target poses')
        return False

    ud.arm1_end_pose = k1_target_pose.pose
    ud.arm2_end_pose = k2_target_pose.pose

    # adjust to be behind the object for grasping
    approach_offset = 0.1  # 10 cm
    k1_target_pose.pose.position.x -= approach_offset
    k2_target_pose.pose.position.x += approach_offset

    k1_target_pose.pose.position.z += 0.01
    k2_target_pose.pose.position.z += 0.01
    
    ud.arm1_target_pose = k1_target_pose.pose
    ud.arm2_target_pose = k2_target_pose.pose

    # publish target poses for visualization/debugging
    node.arm1_target_pub.publish(k1_target_pose)
    node.arm2_target_pub.publish(k2_target_pose)

    ud.move_arm1 = True
    ud.move_arm2 = True

    ud.arm_action_type = ArmActionClientType.CARTESIAN
    
    node.arm1_ac = node.arm1_cart_ac
    node.arm2_ac = node.arm2_cart_ac

    return True

def m_slide_along_table_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    k1_target_pose = node._lookup_pose(GRASP_LINK_1)
    k2_target_pose = node._lookup_pose(GRASP_LINK_2)

    if k1_target_pose is None or k2_target_pose is None:
        node.get_logger().error('Failed to lookup target poses')
        return False

    k1_target_pose.pose.position.x += 0.02
    k2_target_pose.pose.position.x -= 0.02

    ud.arm1_end_pose.position.x = k1_target_pose.pose.position.x
    ud.arm2_end_pose.position.x = k2_target_pose.pose.position.x

    k1_target_pose.pose.position.z += 0.01
    k2_target_pose.pose.position.z += 0.01

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
    
    # k1_target_pose = node._lookup_pose(GRASP_LINK_1)
    # k2_target_pose = node._lookup_pose(GRASP_LINK_2)
    #
    # if k1_target_pose is None or k2_target_pose is None:
    #     node.get_logger().error('Failed to lookup target poses')
    #     return False

    # k1_target_pose.pose.position.z += 0.4
    # k2_target_pose.pose.position.z += 0.4

    k1_target_pose = ud.arm1_end_pose
    k2_target_pose = ud.arm2_end_pose

    k1_target_pose.position.x 

    k1_target_pose.position.z += 0.4
    k2_target_pose.position.z += 0.4

    ud.arm1_target_pose = k1_target_pose
    ud.arm2_target_pose = k2_target_pose

    ud.max_vel_sf = 0.1
    ud.max_acc_sf = 0.1

    ud.move_arm1 = True
    ud.move_arm2 = True
    ud.move_gripper1 = False
    ud.move_gripper2 = False

    return True

def execute_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    if ud.move_arm1 and not ud.arm1_done:
        ud.arm1_done = node.move_arm(node.arm1_ac, ud.arm1_af, 
                                     ud.arm1_target_pose, ud.arm_action_type,
                                     ud.max_vel_sf, ud.max_acc_sf)
    if ud.move_arm2 and not ud.arm2_done:
        ud.arm2_done = node.move_arm(node.arm2_ac, ud.arm2_af, 
                                     ud.arm2_target_pose, ud.arm_action_type,
                                     ud.max_vel_sf, ud.max_acc_sf)
    if ud.move_gripper1 and not ud.g1_done:
        ud.g1_done = node.gripper_control(node.g1_ac, ud.g1_af, ud.arm1_gripper_open, node.robot.arm1.gripper_joint)
    if ud.move_gripper2 and not ud.g2_done:
        ud.g2_done = node.gripper_control(node.g2_ac, ud.g2_af, ud.arm2_gripper_open, node.robot.arm2.gripper_joint)

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
            produce_event(fsm.event_data, EventID.E_M_OPEN_GRIPPER_CONFIG)
        elif ud.m2:
            ud.m2 = False
            ud.m3 = True
            produce_event(fsm.event_data, EventID.E_M_TOUCH_TABLE_CONFIG)
        elif ud.m3:
            ud.m3 = False
            ud.m4 = True
            produce_event(fsm.event_data, EventID.E_M_SLIDE_ALONG_TABLE_CONFIG)
        elif ud.m4:
            ud.m4 = False
            ud.m5 = True
            produce_event(fsm.event_data, EventID.E_M_GRASP_OBJECT_CONFIG)
        elif ud.m5:
            ud.m5 = False
            ud.m6 = True
            produce_event(fsm.event_data, EventID.E_M_COLLABORATE_CONFIG)
        elif ud.m6:
            ud.m6 = False
            produce_event(fsm.event_data, EventID.E_EXECUTE_EXIT)
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
    StateID.S_M_HOME: {
        "step": m_home_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_M_HOME_CONFIGURED]
        ),
    },
    StateID.S_M_OPEN_GRIPPER: {
        "step": m_open_gripper_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_M_OPEN_GRIPPER_CONFIGURED]
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

