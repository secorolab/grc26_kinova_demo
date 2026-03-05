
#ifndef FSM_INTERFACE_HPP
#define FSM_INTERFACE_HPP

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
#include "kdl/chainjnttojacdotsolver.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

#include "robif2b/functions/ethercat.h"
#include "robif2b/functions/kinova_gen3.h"

#include "grc26/kinova_single_arm_demo.fsm.hpp"
#include "grc26/pid_controller.hpp"
#include "grc26/system_state.hpp"
#include "grc26/arm_kdl_model.hpp"
#include "grc26/task_status.hpp"
#include "grc26/achd_solver.hpp"
#include "grc26/task_spec.hpp"
#include "grc26/compute_controller_command.hpp"
#include "grc26/arm_kinematics.hpp"
#include "grc26/controller_config.hpp"

#define NUM_JOINTS 7

#define KINOVA_TAU_CMD_LIMIT 30.0


class FSMInterface
{
public:
  explicit FSMInterface(SystemState& system_state,
                          robif2b_kinova_gen3_nbx& rob, 
                          robif2b_kg3_robotiq_gripper_nbx& gripper,
                          robif2b_robotiq_ft_nbx& ft_sensor,
                          TaskStatusData& status);
  ~FSMInterface();

  int get_current_state() const { return fsm.currentStateIndex; }
  bool is_in_comm_with_hw() const { return in_comm_with_hw; }
  e_states get_fsm_execution_state() const { return fsm_execution_state; }

  // FSM methods
  void configure(events *eventData, SystemState& system_state);
  void idle(events *eventData, const SystemState& system_state);
  void execute(events *eventData, SystemState& system_state);

  void touch_table_behavior_config(events *eventData, SystemState& system_state);
  void slide_on_table_behavior_config(events *eventData, SystemState& system_state);
  void grasp_object_behavior_config(events *eventData, SystemState& system_state);
  void collaborate_behavior_config(events *eventData, SystemState& system_state);
  void release_object_behavior_config(events *eventData, SystemState& system_state);
  void check_post_condition(events *eventData, const SystemState& system_state, const TaskSpec& task_spec);
  void exit(events *eventData, SystemState& system_state);

  // decision of which behavior to execute based on events and arm state
  void fsm_behavior(events *eventData, SystemState& system_state);

  void compute_gravity_comp(events *eventData, SystemState& system_state);
  void run_fsm();

public:
  bool enable_arm_ctrl = true;

private:
  SystemState& system_state;
  robif2b_kinova_gen3_nbx& rob;
  robif2b_kg3_robotiq_gripper_nbx& gripper;
  robif2b_robotiq_ft_nbx& ft_sensor;
  TaskStatusData& task_status;
  TaskSpec task_spec;
  ControllerConfig config;
  ComputeControllerCommand compute_ctr_cmd_obj;

  bool in_comm_with_hw;
  e_states fsm_execution_state;

  std::unique_ptr<ArmKDLModel> model_;
  std::unique_ptr<VereshchaginSolver> solver_;
  std::unique_ptr<ArmKinematics> arm_kinematics_;

};

#endif // FSM_INTERFACE_HPP