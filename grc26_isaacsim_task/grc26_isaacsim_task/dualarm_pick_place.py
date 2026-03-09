#!/usr/bin/python3

import signal
import sys
import copy
import time

import rclpy
from rclpy.utilities import ok as rclpy_ok
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse, GoalResponse, ServerGoalHandle
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
from tf2_ros import Buffer, TransformListener, TransformStamped
from tf2_geometry_msgs import do_transform_pose_stamped

from pydantic import BaseModel
from typing import Optional
from enum import StrEnum

from bdd_ros2_interfaces.msg import Event, Trinary, TrinaryStamped
from bdd_ros2_interfaces.action import Behaviour

from rdflib import Namespace
from rdf_utils.uri import URL_SECORO_M

from coord_dsl.fsm import fsm_step, FSMData
from coord_dsl.event_loop import (
    produce_event,
    consume_event,
    reconfig_event_buffers,
)
from models.fsm import create_fsm, EventID, StateID

from simulation_interfaces.srv import ResetSimulation
from simulation_interfaces.msg import Result as SimResult


TOPIC_LOCATED_PICK = "/obs_policy/located_at_pick_ws"
TOPIC_IS_HELD = "/obs_policy/is_held"
TOPIC_LOCATED_PLACE = "/obs_policy/located_at_place_ws"

NS_M_TMPL = Namespace(f"{URL_SECORO_M}/acceptance-criteria/bdd/templates/")

EXPORTED_EVENTS = {
    'PICK_START' : NS_M_TMPL["evt-pick-start"],
    'PICK_END'   : NS_M_TMPL["evt-pick-end"],
    'PLACE_START': NS_M_TMPL["evt-place-start"],
    'PLACE_END'  : NS_M_TMPL["evt-place-end"],
}

HOLDER_A     = 'holderA'
HOLDER_B     = 'holderB'
OBJECT_LINK  = 'cutting_board_A'
GRASP_LINK_1 = 'grasp_point_1'
GRASP_LINK_2 = 'grasp_point_2'

IS_HELD_DIST_THRESH = 0.05

GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.8

class Motions(StrEnum):
    HOME         = 'home'
    OPEN_GRIPPER = 'open_gripper'
    TOUCH_TABLE  = 'touch_table'
    APPROACH     = 'approach'
    GRASP        = 'grasp'
    PLACE        = 'place'
    RELEASE      = 'release'
    RETRACT      = 'retract'
    RETURN_HOME  = 'return_home'

    NONE         = 'none'

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
    
    arm1_target_pose: Pose | list[Pose]   = None
    arm2_target_pose: Pose | list[Pose]   = None
    arm1_gripper_open: bool               = True
    arm2_gripper_open: bool               = True
    arm1_end_pose: Pose                   = None
    arm2_end_pose: Pose                   = None

    arm1_max_vel_sf: float                = 0.0
    arm1_max_acc_sf: float                = 0.0
    arm2_max_vel_sf: float                = 0.0
    arm2_max_acc_sf: float                = 0.0

    arm_action_type: ArmActionClientType  = ArmActionClientType.CARTESIAN

    current_motion: Motions               = Motions.HOME
    
    class Config:
        arbitrary_types_allowed = True


class DualArmPickPlace(Node):
    def __init__(self):
        super().__init__('dual_arm_pick_place')
        self.logger = self.get_logger()
        
        self.reset_simulation_service = self.create_client(ResetSimulation, 'reset_simulation')
        while not self.reset_simulation_service.wait_for_service(timeout_sec=1.0):
            self.logger.warning('Waiting for reset_simulation service...')
        self.reset_simulation_request = ResetSimulation.Request()
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

        self.declare_parameter("event_topic", "/bdd/events")
        self.declare_parameter("bhv_server_name", "bhv_server")

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

        self.bhv_server_name = self.get_parameter("bhv_server_name").value

        self.bhv_action_server = ActionServer(
            self,
            Behaviour,
            self.bhv_server_name,
            goal_callback=self.bhv_goal_cb,
            execute_callback=self.bhv_execute_cb,
            cancel_callback=self.bhv_cancel_cb
        )

        self.event_topic = self.get_parameter("event_topic").value

        self.evt_pub = self.create_publisher(Event, self.event_topic, 10)

        self.located_pick_pub = self.create_publisher(TrinaryStamped, TOPIC_LOCATED_PICK, 10)
        self.is_held_pub = self.create_publisher(TrinaryStamped, TOPIC_IS_HELD, 10)
        self.located_place_pub = self.create_publisher(TrinaryStamped, TOPIC_LOCATED_PLACE, 10)

        self.is_held_timer = self.create_timer(0.1, self.is_held_monitor)
        self.is_held_timer.cancel()

        self.bhv_ctx_id    = None
        self.bhv_goal_in   = False
        self.bhv_goal_done = False
        self.bhv_wait_for_sim_reset = True

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
            # self.logger.info(f'{ac._action_name}: Sending goal to action server...')
            af.send_goal_future = ac.send_goal_async(goal_msg)
            assert af.send_goal_future is not None, "send_goal_future is None"
            return False

        # wait for goal to be accepted
        if not af.send_goal_future.done():
            # self.logger.info(f'{ac._action_name}: Waiting for goal to be accepted...')
            return False

        if af.goal_handle is None:
            af.goal_handle = af.send_goal_future.result()
            if not af.goal_handle or not af.goal_handle.accepted:
                self.logger.error(f'{ac._action_name}: Goal rejected')
                af.send_goal_future = None
                return False

            # self.logger.info(f'{ac._action_name}: Goal accepted, waiting for result...')
            af.get_result_future = af.goal_handle.get_result_async()
            return False

        # wait for result
        if af.get_result_future is None:
            self.logger.warning(f'{ac._action_name}: get_result_future is None')
            return False

        if not af.get_result_future.done():
            self.logger.info(f'{ac._action_name}: Waiting for result...', throttle_duration_sec=10.0)
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

    def bhv_goal_cb(self, goal_request):
        if self.bhv_goal_in:
            self.logger.warning('A behavior goal is already in progress, rejecting new goal')
            return GoalResponse.REJECT

        self.logger.info('Behavior execution goal accepted')
        self.bhv_goal_in = True
        return GoalResponse.ACCEPT

    def bhv_cancel_cb(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.logger.info('Behavior execution canceled')
        return CancelResponse.ACCEPT

    def bhv_execute_cb(self, goal_handle: ServerGoalHandle):
        self.logger.info('Behavior execution started')

        self.bhv_ctx_id = goal_handle.request.scenario_context_id
        self.bhv_goal_done = False

        self.k1_speed = 0.8
        self.k2_speed = 0.8

        for cfg_msg in goal_handle.request.configs:
            target = cfg_msg.target
            name   = cfg_msg.name
            val    = cfg_msg.num_value
            
            # grep kinova1 and kinova2 in target uri
            kinova1 = self.robot.arm1.arm in target
            kinova2 = self.robot.arm2.arm in target

            if kinova1:
                self.k1_speed = val
            elif kinova2:
                self.k2_speed = val

            self.get_logger().info(
                f"Bhv Config {target}: {name} = {val}"
            )

        self.get_logger().info(f"Set arm speeds: k1={self.k1_speed}, k2={self.k2_speed}")

        response = Behaviour.Result()
        response.result.scenario_context_id = self.bhv_ctx_id
        
        feedback = Behaviour.Feedback()
        feedback.scenario_context_id = self.bhv_ctx_id

        self.bhv_wait_for_sim_reset = True

        rate = self.create_rate(100)
        while rclpy_ok() and not self.bhv_goal_done:
            
            if goal_handle.is_cancel_requested:
                self.logger.info('Behavior execution cancel requested')
                goal_handle.canceled()
                return Behaviour.Result()

            rate.sleep()

        response.result.stamp   = self.get_clock().now().to_msg()
        response.result.trinary = Trinary(value=Trinary.TRUE)

        goal_handle.succeed()
        return response

    def _lookup_pose(self, link: str, target: str = 'world') -> PoseStamped | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                target, link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as exc:
            self.get_logger().error(f'TF lookup failed for {link}: {exc}')
            return None

        pose = PoseStamped()
        pose.header.frame_id  = target
        pose.header.stamp     = self.get_clock().now().to_msg()
        pose.pose.position.x  = tf.transform.translation.x
        pose.pose.position.y  = tf.transform.translation.y
        pose.pose.position.z  = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _lookup_transform(self, target: str, source: str) -> Optional[TransformStamped]:
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return tf
        except Exception as exc:
            self.get_logger().error(f'TF lookup failed from {source} to {target}: {exc}')
            return None

    def _get_pose_from_transform(self, tf: TransformStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id  = tf.header.frame_id
        pose.header.stamp     = self.get_clock().now().to_msg()
        pose.pose.position.x  = tf.transform.translation.x
        pose.pose.position.y  = tf.transform.translation.y
        pose.pose.position.z  = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    def _transform_pose_with_tf(self, pose: PoseStamped, tf: TransformStamped) -> Optional[PoseStamped]:
        try:
            transformed_pose = do_transform_pose_stamped(pose, tf)
            return transformed_pose
        except Exception as exc:
            self.get_logger().error(f'Pose transformation failed with given TF: {exc}')
            return None

    def transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        # use lookup_transform to get transform from pose's frame to target_frame
        tf = self._lookup_transform(target_frame, pose.header.frame_id)
        if tf is None:
            return None
        
        try:
            transformed_pose = do_transform_pose_stamped(pose, tf)
            return transformed_pose
        except Exception as exc:
            self.get_logger().error(f'Pose transformation failed to {target_frame}: {exc}')
            return None

    def get_move_arm_msg(self, target_pose:Pose) -> MoveToPose.Goal:
        msg = MoveToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = target_pose
        msg.target_pose = pose

        return msg

    def get_move_cartesian_msg(self, target_pose: Pose | list[Pose],
                               max_vel_sf: float, max_acc_sf: float) -> MoveToCartesianPose.Goal:
        msg = MoveToCartesianPose.Goal()

        poses = []
        if isinstance(target_pose, Pose):
            poses.append(target_pose)
        else:
            poses.extend(target_pose)

        for tp in poses:
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose = tp
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
                 target_pose: Pose | list[Pose],
                 action_type: ArmActionClientType,
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
        val = 0.0 if open else 0.6
        gripper_msg = self.get_gripper_cmd_msg(val, joint_name)
        # send goal to gripper
        if not self.execute_action(ac, af, gripper_msg):
            return False

        return True

    def pose_norm(self, pose: Pose) -> float:
        return (pose.position.x ** 2 + pose.position.y ** 2 + pose.position.z ** 2) ** 0.5

    def is_held_monitor(self):
        arm1_ee_gl_pose = self._lookup_pose(self.robot.arm1.ee_link, GRASP_LINK_1)
        arm2_ee_gl_pose = self._lookup_pose(self.robot.arm2.ee_link, GRASP_LINK_2)

        arm1_gl_norm = self.pose_norm(arm1_ee_gl_pose.pose)
        arm2_gl_norm = self.pose_norm(arm2_ee_gl_pose.pose)

        is_held = arm1_gl_norm < IS_HELD_DIST_THRESH and \
                    arm2_gl_norm < IS_HELD_DIST_THRESH
        
        trinary_msg = TrinaryStamped()
        trinary_msg.stamp = self.get_clock().now().to_msg()
        trinary_msg.scenario_context_id = self.bhv_ctx_id
        trinary_msg.trinary.value = Trinary.TRUE if is_held else Trinary.FALSE
        self.is_held_pub.publish(trinary_msg)

    def is_placed(self):
        obj_pose = self._lookup_pose(OBJECT_LINK, HOLDER_B)

        if obj_pose is None:
            self.logger.error('Failed to lookup object or place pose for is_placed check')
            return False
        
        obj_norm = self.pose_norm(obj_pose.pose)

        return obj_norm < IS_HELD_DIST_THRESH

        
    
def configure_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    node.configure()
    return True

def idle_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    if node.bhv_goal_in and not node.bhv_wait_for_sim_reset:
        ud.current_motion = Motions.HOME
        print(f"Starting behavior execution, transitioning to '{Motions.HOME.name}' motion")
        produce_event(fsm.event_data, EventID.E_M_HOME_CONFIG)
    elif node.bhv_goal_in and node.bhv_wait_for_sim_reset:

        if not hasattr(node, "_reset_future"):
            node._reset_future = None
            node._reset_done_time = None

        if node._reset_future is None:
            node._reset_future = node.reset_simulation_service.call_async(
                node.reset_simulation_request
            )

        if node._reset_future.done() and node._reset_done_time is None:
            service_result = node._reset_future.result().result.result
            node._reset_future = None

            if service_result == SimResult.RESULT_OK:
                node._reset_done_time = node.get_clock().now()

        if node._reset_done_time is not None:
            if (node.get_clock().now() - node._reset_done_time) > rclpy.duration.Duration(seconds=3):
                node.bhv_wait_for_sim_reset = False
                node._reset_done_time = None

        node.get_logger().info(
            "Waiting for simulation reset...", throttle_duration_sec=5.0
        )
    else:
        return False

    return True

def m_home_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    ud.arm_action_type = ArmActionClientType.NAMED
    
    ud.arm1_max_vel_sf = 1.0
    ud.arm1_max_acc_sf = 1.0

    ud.arm2_max_vel_sf = 1.0
    ud.arm2_max_acc_sf = 1.0

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

    # pub bhv
    trinary_msg = TrinaryStamped()
    trinary_msg.stamp = node.get_clock().now().to_msg()
    trinary_msg.scenario_context_id = node.bhv_ctx_id
    trinary_msg.trinary.value = Trinary.TRUE
    node.located_pick_pub.publish(trinary_msg)

    node.logger.info('published located pick message')

    ud.arm1_max_vel_sf = 1.0
    ud.arm1_max_acc_sf = 1.0

    ud.arm2_max_vel_sf = 1.0
    ud.arm2_max_acc_sf = 1.0

    g11_world_pose = node._lookup_pose(GRASP_LINK_1)
    g12_world_pose = node._lookup_pose(GRASP_LINK_2)

    # lookup grasp poses in holder A frame (object is in holder A at this point)
    gl1_a_transform = node._lookup_transform(HOLDER_A, GRASP_LINK_1)
    gl2_a_transform = node._lookup_transform(HOLDER_A, GRASP_LINK_2)

    # need to place obj in holder B for next steps, so also lookup transform from holder A to holder B
    h_a_to_b_transform = node._lookup_transform(HOLDER_A, HOLDER_B)

    # convert grasp transforms to poses in holder A frame
    gl1_a_pose = node._get_pose_from_transform(gl1_a_transform)
    gl2_a_pose = node._get_pose_from_transform(gl2_a_transform)

    # transform grasp poses from holder A frame to holder B frame
    gl1_b_pose = node._transform_pose_with_tf(gl1_a_pose, h_a_to_b_transform)
    gl2_b_pose = node._transform_pose_with_tf(gl2_a_pose, h_a_to_b_transform)

    if None in [gl1_a_pose, gl2_a_pose, gl1_b_pose, gl2_b_pose]:
        node.get_logger().error('Failed to lookup/transform grasp poses')
        return False

    # now transform grasp poses to world frame
    k1_target_pose = node.transform_pose(gl1_a_pose, 'world')
    k2_target_pose = node.transform_pose(gl2_a_pose, 'world')

    k1_b_target_pose = node.transform_pose(gl1_b_pose, 'world')
    k2_b_target_pose = node.transform_pose(gl2_b_pose, 'world')

    k1_target_pose.pose.orientation = g11_world_pose.pose.orientation
    k2_target_pose.pose.orientation = g12_world_pose.pose.orientation

    k1_b_target_pose.pose.orientation = g11_world_pose.pose.orientation
    k2_b_target_pose.pose.orientation = g12_world_pose.pose.orientation
    
    if k1_target_pose is None or k2_target_pose is None:
        node.get_logger().error('Failed to lookup target poses')
        return False

    ud.arm1_end_pose = k1_b_target_pose.pose
    ud.arm2_end_pose = k2_b_target_pose.pose

    approach_offset = 0.1  # 10 cm
    k1_target_pose.pose.position.x -= approach_offset
    k2_target_pose.pose.position.x += approach_offset

    k1_target_pose.pose.position.z += 0.01
    k2_target_pose.pose.position.z += 0.01
    
    ud.arm1_target_pose = k1_target_pose.pose
    ud.arm2_target_pose = k2_target_pose.pose
    
    node.arm1_target_pub.publish(k1_target_pose)
    node.arm2_target_pub.publish(k2_target_pose)

    ud.move_arm1 = True
    ud.move_arm2 = True

    ud.arm_action_type = ArmActionClientType.CARTESIAN
    
    node.arm1_ac = node.arm1_cart_ac
    node.arm2_ac = node.arm2_cart_ac
    
    evt_msg = Event()
    evt_msg.stamp = node.get_clock().now().to_msg()
    evt_msg.scenario_context_id = node.bhv_ctx_id
    evt_msg.uri = EXPORTED_EVENTS['PICK_START']
    node.evt_pub.publish(evt_msg)
    node.logger.info('published pick start event')

    return True

def m_slide_along_table_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    ud.arm1_max_vel_sf = 1.0
    ud.arm1_max_acc_sf = 1.0

    ud.arm2_max_vel_sf = 1.0
    ud.arm2_max_acc_sf = 1.0

    k1_target_pose = node._lookup_pose(GRASP_LINK_1)
    k2_target_pose = node._lookup_pose(GRASP_LINK_2)

    if k1_target_pose is None or k2_target_pose is None:
        node.get_logger().error('Failed to lookup target poses')
        return False

    k1_target_pose.pose.position.x += 0.02
    k2_target_pose.pose.position.x -= 0.02

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

    evt_msg = Event()
    evt_msg.stamp = node.get_clock().now().to_msg()
    evt_msg.scenario_context_id = node.bhv_ctx_id
    evt_msg.uri = EXPORTED_EVENTS['PICK_END']
    node.evt_pub.publish(evt_msg)
    node.logger.info('published pick end event')

    return True

def m_collaborate_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    ud.arm1_max_vel_sf = node.k1_speed
    ud.arm1_max_acc_sf = node.k1_speed
  
    ud.arm2_max_vel_sf = node.k2_speed
    ud.arm2_max_acc_sf = node.k2_speed

    k1_target_pose = ud.arm1_end_pose
    k2_target_pose = ud.arm2_end_pose

    arm1_wp1 = copy.deepcopy(k1_target_pose)
    arm2_wp1 = copy.deepcopy(k2_target_pose)

    arm1_wp1.position.z += 0.3
    arm2_wp1.position.z += 0.3

    arm1_waypoints = [arm1_wp1, k1_target_pose]
    arm2_waypoints = [arm2_wp1, k2_target_pose]

    ud.arm1_target_pose = arm1_waypoints
    ud.arm2_target_pose = arm2_waypoints

    ud.move_arm1 = True
    ud.move_arm2 = True
    ud.move_gripper1 = False
    ud.move_gripper2 = False

    # spwan is-held monitor
    node.is_held_timer.reset()

    evt_msg = Event()
    evt_msg.stamp = node.get_clock().now().to_msg()
    evt_msg.scenario_context_id = node.bhv_ctx_id
    evt_msg.uri = EXPORTED_EVENTS['PLACE_START']
    node.evt_pub.publish(evt_msg)
    node.logger.info('published place start event')

    return True

def m_release_object_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    ud.arm1_gripper_open = True
    ud.arm2_gripper_open = True

    ud.move_arm1 = False
    ud.move_arm2 = False
    ud.move_gripper1 = True
    ud.move_gripper2 = True

    return True

def m_retract_arm_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    node.is_held_timer.cancel()

    ud.arm1_max_vel_sf = 1.0
    ud.arm1_max_acc_sf = 1.0

    ud.arm2_max_vel_sf = 1.0
    ud.arm2_max_acc_sf = 1.0

    a1_current_pose = node._lookup_pose(node.robot.arm1.ee_link)
    a2_current_pose = node._lookup_pose(node.robot.arm2.ee_link)

    a1_current_pose.pose.position.x -= 0.1
    a2_current_pose.pose.position.x += 0.1

    a1_fp = copy.deepcopy(a1_current_pose.pose)
    a2_fp = copy.deepcopy(a2_current_pose.pose)

    a1_fp.position.z += 0.2
    a2_fp.position.z += 0.2

    ud.arm1_target_pose = [a1_current_pose.pose, a1_fp]
    ud.arm2_target_pose = [a2_current_pose.pose, a2_fp]

    ud.move_arm1 = True
    ud.move_arm2 = True
    ud.move_gripper1 = False
    ud.move_gripper2 = False

    return True

def execute_step(fsm: FSMData, ud: UserData, node: DualArmPickPlace):
    if ud.move_arm1 and not ud.arm1_done:
        ud.arm1_done = node.move_arm(node.arm1_ac, ud.arm1_af, 
                                     ud.arm1_target_pose, ud.arm_action_type,
                                     ud.arm1_max_vel_sf, ud.arm1_max_acc_sf)
    if ud.move_arm2 and not ud.arm2_done:
        ud.arm2_done = node.move_arm(node.arm2_ac, ud.arm2_af, 
                                     ud.arm2_target_pose, ud.arm_action_type,
                                     ud.arm2_max_vel_sf, ud.arm2_max_acc_sf)
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
    if not all_done:
        return False

    # reset for next step
    ud.move_arm1 = False
    ud.move_arm2 = False
    ud.move_gripper1 = False
    ud.move_gripper2 = False
    ud.arm1_done = False
    ud.arm2_done = False
    ud.g1_done = False
    ud.g2_done = False
    
    match ud.current_motion:
        case Motions.HOME:
            ud.current_motion = Motions.OPEN_GRIPPER
            print(f"Motion '{Motions.HOME.name}' completed, transitioning to '{Motions.OPEN_GRIPPER.name}'")
            produce_event(fsm.event_data, EventID.E_M_OPEN_GRIPPER_CONFIG)
        case Motions.OPEN_GRIPPER:
            ud.current_motion = Motions.TOUCH_TABLE
            print(f"Motion '{Motions.OPEN_GRIPPER.name}' completed, transitioning to '{Motions.TOUCH_TABLE.name}'")
            produce_event(fsm.event_data, EventID.E_M_TOUCH_TABLE_CONFIG)
        case Motions.TOUCH_TABLE:
            ud.current_motion = Motions.APPROACH
            print(f"Motion '{Motions.TOUCH_TABLE.name}' completed, transitioning to '{Motions.APPROACH.name}'")
            produce_event(fsm.event_data, EventID.E_M_SLIDE_ALONG_TABLE_CONFIG)
        case Motions.APPROACH:
            ud.current_motion = Motions.GRASP
            print(f"Motion '{Motions.APPROACH.name}' completed, transitioning to '{Motions.GRASP.name}'")
            produce_event(fsm.event_data, EventID.E_M_GRASP_OBJECT_CONFIG)
        case Motions.GRASP:
            ud.current_motion = Motions.PLACE
            print(f"Motion '{Motions.GRASP.name}' completed, transitioning to '{Motions.PLACE.name}'")
            produce_event(fsm.event_data, EventID.E_M_COLLABORATE_CONFIG)
        case Motions.PLACE:
            ud.current_motion = Motions.RELEASE
            print(f"Motion '{Motions.PLACE.name}' completed, transitioning to '{Motions.RELEASE.name}'")
            produce_event(fsm.event_data, EventID.E_M_RELEASE_OBJECT_CONFIG)
        case Motions.RELEASE:
            ud.current_motion = Motions.RETRACT
            
           

            print(f"Motion '{Motions.RELEASE.name}' completed, transitioning to '{Motions.RETRACT.name}'")
            produce_event(fsm.event_data, EventID.E_M_RETRACT_ARM_CONFIG)
        case Motions.RETRACT:
            ud.current_motion = Motions.RETURN_HOME
            print(f"Motion '{Motions.RETRACT.name}' completed, behavior execution done")
            produce_event(fsm.event_data, EventID.E_M_RETURN_HOME_CONFIG)
        case Motions.RETURN_HOME:
            ud.current_motion = Motions.NONE
            print(f"Motion '{Motions.RETURN_HOME.name}' completed, all motions done")
            
            evt_msg = Event()
            evt_msg.stamp = node.get_clock().now().to_msg()
            evt_msg.scenario_context_id = node.bhv_ctx_id
            evt_msg.uri = EXPORTED_EVENTS['PLACE_END']
            node.evt_pub.publish(evt_msg)
            node.logger.info('published place end event')
            node.bhv_goal_in = False
            
            # pub bhv
            is_placed = node.is_placed()

            trinary_msg = TrinaryStamped()
            trinary_msg.stamp = node.get_clock().now().to_msg()
            trinary_msg.scenario_context_id = node.bhv_ctx_id
            trinary_msg.trinary.value = Trinary.TRUE if is_placed else Trinary.FALSE
            node.located_place_pub.publish(trinary_msg)
            node.bhv_goal_done = True
            
            node.logger.info('published located place message')

            produce_event(fsm.event_data, EventID.E_EXECUTE_IDLE)
        case _:
            print(f"Motion '{ud.current_motion.name}' completed, but no transition defined")
    
    return all_done

def generic_on_end(fsm: FSMData, ud: UserData, end_events: list[EventID]):
    if StateID(fsm.current_state_index) not in [StateID.S_IDLE, StateID.S_EXECUTE]:
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
    StateID.S_M_RELEASE_OBJECT: {
        "step": m_release_object_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_M_RELEASE_OBJECT_CONFIGURED]
        ),
    },
    StateID.S_M_RETRACT_ARM: {
        "step": m_retract_arm_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_M_RETRACT_ARM_CONFIGURED]
        ),
    },
    StateID.S_M_RETURN_HOME: {
        "step": m_home_step,
        "on_end": lambda fsm, ud, node: generic_on_end(
            fsm, ud, [EventID.E_M_RETURN_HOME_CONFIGURED]
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

    while rclpy_ok():
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

