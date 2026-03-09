
#ifndef FSM_INTERFACE_HPP
#define FSM_INTERFACE_HPP

#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <array>
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
#include "grc26/debug_signals.hpp"
#include "grc26/trajectory.hpp"

#define NUM_JOINTS 7

#define KINOVA_TAU_CMD_LIMIT 30.0
// To avoid singularity, using -45.0 (lt), when starting from 'B' joystick position
#define JOINT_3_ANGLE_LIMIT_DEG_UL -45      // 147.83 (lt)
#define JOINT_3_ANGLE_LIMIT_DEG_LL -144     // -148.8 (gt)
#define JOINT_5_ANGLE_LIMIT_DEG_UL 116.0    // 120.32 (lt)
#define JOINT_5_ANGLE_LIMIT_DEG_LL -117.0   //-121 (gt)

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
  void idle(events *eventData, SystemState& system_state);
  void execute(events *eventData, SystemState& system_state);

  void touch_table_behavior_config(events *eventData, SystemState& system_state);
  void slide_on_table_behavior_config(events *eventData, SystemState& system_state);
  void grasp_object_behavior_config(events *eventData, SystemState& system_state);
  void collaborate_behavior_config(events *eventData, SystemState& system_state);
  void release_object_behavior_config(events *eventData, SystemState& system_state);
  void check_post_condition(events *eventData, const SystemState& system_state, const TaskSpec& task_spec);
  void exit(events *eventData, SystemState& system_state);
  void avoid_joint_limits(SystemState& system_state);

  // decision of which behavior to execute based on events and arm state
  void fsm_behavior(events *eventData, SystemState& system_state);

  void normalize_angle_diff(double& angle_diff);
  void compute_trajectory();
  void human_interaction_monitoring(double corrected_external_force_magnitude);
  void transform_ft_readings_to_BL_update_state(SystemState& system_state,
                                               ArmKinematics& arm_kinematics);
  void reset_ft_force_estimator();
  bool update_ft_force_estimate(const SystemState& system_state,
                                            std::array<double, 6>& corrected_ft_wrt_init_ref,
                                            std::array<double,6>& corrected_ft_wrt_prev_ref);
  void run_fsm();
  bool getLatestDebugSample(DebugSample& out) const;

public:
  bool enable_arm_ctrl = true;

private:
  SystemState& system_state;
  robif2b_kinova_gen3_nbx& rob;
  robif2b_kg3_robotiq_gripper_nbx& gripper;
  robif2b_robotiq_ft_nbx& ft_sensor;
  TaskStatusData& task_status;
  TaskSpec task_spec;
  ControllerConfig ctr_config_idle;
  ControllerConfig ctr_config_touch_table;
  ControllerConfig ctr_config_slide_on_table;
  ControllerConfig ctr_config_grasp_object;
  ControllerConfig ctr_config_collaborate;
  ControllerConfig ctr_config_traj_tracking;
  ControllerConfig ctr_config_release_object;
  ComputeControllerCommand compute_ctr_cmd_obj;
  bool task_triggered = false;

  KDL::Frame final_ee_pose_;
  std::unique_ptr<TrajectoryGenerator> trajectory_object_;
  KDL::Trajectory* trajectory_ = nullptr;
  bool is_trajectory_computed_ = false;
  std::chrono::steady_clock::time_point trajectory_start_time_{};
  bool human_interaction_detected = false;
  int interaction_detection_counter_limit = 100;
  int loss_of_interaction_detection_counter_limit = 3000;
  int interaction_counter = 0;

  bool in_comm_with_hw;
  e_states fsm_execution_state;

  std::unique_ptr<ArmKDLModel> model_;
  std::unique_ptr<VereshchaginSolver> solver_;
  std::unique_ptr<ArmKinematics> arm_kinematics_;

  static constexpr std::size_t FT_WINDOW_SIZE = 100;
  bool ft_reference_ready_ = false;
  std::size_t ft_reference_count_ = 0;
  std::array<double, 6> ft_reference_sum_{};
  std::array<double, 6> ft_reference_mean_{};
  std::array<double, 6> ft_rolling_mean_{};

  std::array<std::array<double, 6>, FT_WINDOW_SIZE> ft_window_samples_{};
  std::array<double, 6> ft_window_sum_{};
  std::size_t ft_window_count_ = 0;
  std::size_t ft_window_index_ = 0;

  DebugSample latest_debug_sample_{};
  bool debug_sample_valid_ = false;
  std::uint64_t debug_sequence_counter_ = 0;

};

#endif // FSM_INTERFACE_HPP