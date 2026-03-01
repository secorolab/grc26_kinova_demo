#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <atomic>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#include <kdl_parser/kdl_parser.hpp>

#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainjnttojacdotsolver.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

#include "robif2b/functions/kinova_gen3.h"
#include "grc26/fsm_interface.hpp"
#include "grc26/pid_controller.hpp"
#include "grc26/cartesian_motion_setpoint.hpp"
#include "grc26/kinova_single_arm_demo.fsm.hpp"

#define NUM_JOINTS 7

#define KINOVA_TAU_CMD_LIMIT 30.0

FSMInterface::FSMInterface(robif2b_kinova_gen3_nbx& rob)
  : rob(rob), in_comm_with_hw(false)
{

}

FSMInterface::~FSMInterface() {
}

int FSMInterface::get_current_state() const
{
    return fsm.currentStateIndex;
}

bool FSMInterface::is_in_comm_with_hw() const
{
    return in_comm_with_hw;
}

void FSMInterface::configure(events *eventData, ArmState& arm_state){

  // load kinematic tree and extract chain
  std::string urdf_file = "GEN3_URDF_V12.urdf";
  std::string urdf_path = ament_index_cpp::get_package_share_directory("grc26") + "/urdf/" + urdf_file;
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromFile(urdf_path, kdl_tree)) {
      printf("Failed to parse URDF file: %s\n", urdf_path.c_str());
      produce_event(eventData, E_CONFIGURE_EXIT);
      return;
  }

  KDL::Chain kdl_chain;
  constexpr const char* BASE_LINK = "base_link";
  constexpr const char* TCP_LINK = "EndEffector_Link";

  if (!kdl_tree.getChain(BASE_LINK, TCP_LINK, kdl_chain)) {
    printf("Failed to extract KDL chains from URDF\n");
    produce_event(eventData, E_CONFIGURE_EXIT);
    return;
  }
  printf("Successfully parsed URDF and extracted KDL chains\n");
  KDL::Vector gravity_vec(0, 0, -9.81);

  int num_joints = kdl_chain.getNrOfJoints();
  int num_segments = kdl_chain.getNrOfSegments();

  // print link names and joint names
  printf("Kinova arm has %d joints:\n", num_joints);
  for (size_t i = 0; i < kdl_chain.getNrOfSegments(); ++i) {
      const auto& segment = kdl_chain.getSegment(i);
      printf("  Segment %zu: %s\n", i, segment.getName().c_str());
      if (segment.getJoint().getType() != KDL::Joint::None) {
          printf("    Joint: %s\n", segment.getJoint().getName().c_str());
      }
  }
  assert(num_joints == NUM_JOINTS && "Kinova arm has unexpected number of joints");
  printf("Kinova arm chain has %d segments and %d joints\n", num_segments, num_joints);

  // establish communication with arm
  robif2b_kinova_gen3_configure(&rob);
  if (!rob.success) {
      printf("Error during gen3_configure\n");
      robif2b_kinova_gen3_shutdown(&rob);
      if (!rob.success) {
          printf("Error during gen3_shutdown\n");
          produce_event(eventData, E_CONFIGURE_EXIT);
          return;
      }
  }

  robif2b_kinova_gen3_recover(&rob);
  if (!rob.success) {
      printf("Error during gen3_recover\n");
      robif2b_kinova_gen3_shutdown(&rob);
      if (!rob.success) {
          printf("Error during gen3_shutdown\n");
          produce_event(eventData, E_CONFIGURE_EXIT);
          return;
      }
  }

  printf("Starting\n");
  robif2b_kinova_gen3_start(&rob);
  if (!rob.success) {
      printf("Error during gen3_start\n");
      robif2b_kinova_gen3_stop(&rob);
      printf("Stopped\n");
  }

  in_comm_with_hw = true;
  // emit event to transition to idle state after configuration
  produce_event(eventData, E_CONFIGURED);
}

void FSMInterface::idle(events *eventData, const ArmState& arm_state){

}

void FSMInterface::execute(events *eventData, ArmState& arm_state){

  // solve for control commands

  // update control commands
}

void FSMInterface::touch_table_behavior_config(events *eventData, ArmState& arm_state){
  sp.translation_mode = ControlMode::Velocity;
  sp.rotation_mode    = ControlMode::Position;
  sp.gripper_mode    = ControlMode::None;
  sp.linear_vel = Eigen::Vector3d(0.0, 0.0, -0.01); // m/s
  sp.orientation = Eigen::Vector3d(0.0, 0.0, 0.0); // roll, pitch, yaw

  // TODO: add post condition

  produce_event(eventData, E_M_SLIDE_ALONG_TABLE_CONFIGURED);
}

void FSMInterface::slide_on_table_behavior_config(events *eventData, ArmState& arm_state){
  sp.translation_mode = ControlMode::Velocity;
  sp.rotation_mode    = ControlMode::Position;
  sp.gripper_mode    = ControlMode::None;
  sp.linear_vel = Eigen::Vector3d(0.05, 0.0, 0.0); // m/s
  sp.orientation = Eigen::Vector3d(0.0, 0.0, 0.0); // roll, pitch, yaw

  // TODO: add post condition

  produce_event(eventData, E_M_TOUCH_TABLE_CONFIGURED);
}

void FSMInterface::grasp_object_behavior_config(events *eventData, ArmState& arm_state){
  sp.translation_mode = ControlMode::Velocity;
  sp.rotation_mode    = ControlMode::Position;
  sp.gripper_mode    = ControlMode::Position;
  sp.linear_vel = Eigen::Vector3d(0.0, 0.0, -0.01); // m/s
  sp.orientation = Eigen::Vector3d(0.0, 0.0, 0.0); // roll, pitch, yaw
  sp.gripper_position = 1.0; // fully closed

  // TODO: add post condition

  produce_event(eventData, E_M_GRASP_OBJECT_CONFIGURED);
}

void FSMInterface::collaborate_behavior_config(events *eventData, ArmState& arm_state){
  sp.translation_mode = ControlMode::Effort;
  sp.rotation_mode    = ControlMode::Effort;
  sp.gripper_mode    = ControlMode::Position;
  sp.feedforward_force = Eigen::Vector3d(0.0, 0.0, 0.0); // Newton
  sp.feedforward_torque = Eigen::Vector3d(0.0, 0.0, 0.0); // Newton-meter
  sp.gripper_position = 1.0; // fully closed

  // TODO: add post condition
  
  produce_event(eventData, E_M_COLLABORATE_CONFIGURED);
}

void FSMInterface::release_object_behavior_config(events *eventData, ArmState& arm_state){
  sp.translation_mode = ControlMode::None;
  sp.rotation_mode    = ControlMode::None;
  sp.gripper_mode    = ControlMode::Position;
  sp.gripper_position = 0.0; // fully open

  produce_event(eventData, E_M_RELEASE_OBJECT_CONFIGURED);
}

void FSMInterface::exit(events *eventData, ArmState& arm_state){
  // stop the arm and shutdown communication
  robif2b_kinova_gen3_stop(&rob);
  if (!rob.success) {
      printf("Error during gen3_stop\n");
  }
  printf("Stopped\n");

  robif2b_kinova_gen3_shutdown(&rob);
  if (!rob.success) {
      printf("Error during gen3_shutdown\n");
  }
  printf("Shutdown\n");

  // check: 
}

// decision of which behavior to execute based on events and arm state
void FSMInterface::fsm_behavior(events *eventData, ArmState& arm_state){

  if (consume_event(eventData, E_ENTER_IDLE)) {
      idle(eventData, arm_state);
  }
  if (consume_event(eventData, E_ENTER_CONFIGURE)) {
      configure(eventData, arm_state);
  }
  if (consume_event(eventData, E_ENTER_EXECUTE)) {
      execute(eventData, arm_state);
  }
  if (consume_event(eventData, E_ENTER_M_TOUCH_TABLE)) {
      touch_table_behavior_config(eventData, arm_state);
  }
  if (consume_event(eventData, E_ENTER_M_SLIDE_ALONG_TABLE)) {
      slide_on_table_behavior_config(eventData, arm_state);
  }
  if (consume_event(eventData, E_ENTER_M_GRASP_OBJECT)) {
      grasp_object_behavior_config(eventData, arm_state);
  }
  if (consume_event(eventData, E_ENTER_M_COLLABORATE)) {
      collaborate_behavior_config(eventData, arm_state);
  }
  if (consume_event(eventData, E_ENTER_M_RELEASE_OBJECT)) {
      release_object_behavior_config(eventData, arm_state);
  }
  if (consume_event(eventData, E_ENTER_EXIT)) {
      exit(eventData, arm_state);
  }
}

void FSMInterface::compute_gravity_comp(events *eventData, ArmState& arm_state){

}

void FSMInterface::compute_cartesian_ctrl(events *eventData, ArmState& arm_state){

}

void FSMInterface::run_fsm(){
  produce_event(&eventData, E_STEP);
  fsm_behavior(&eventData, arm_state);
  fsm_step_nbx(&fsm);
  reconfig_event_buffers(&eventData);
};