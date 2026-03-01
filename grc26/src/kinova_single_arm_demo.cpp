
#include <atomic>
#include <csignal>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "grc26/task_status.hpp"
#include "grc26/task_status_node.hpp"
#include "grc26/msg/task_status.hpp"
#include "robif2b/functions/kinova_gen3.h"
#include "grc26/kinova_single_arm_demo.fsm.hpp"
#include "grc26/fsm_interface.hpp"


#define LOG_INFO(node, msg, ...) RCLCPP_INFO(node->get_logger(), msg, ##__VA_ARGS__)
#define LOG_ERROR(node, msg, ...) RCLCPP_ERROR(node->get_logger(), msg, ##__VA_ARGS__)

#define LOG_INFO_S(node, expr) RCLCPP_INFO_STREAM((node)->get_logger(), expr)
#define LOG_ERROR_S(node, expr) RCLCPP_ERROR_STREAM((node)->get_logger(), expr)

#define NUM_JOINTS 7

std::atomic_bool shutting_down{false};

void signal_handler(int /*signum*/) {
    shutting_down.store(true);
}



int main(int argc, char ** argv)
{
  // --------------------- Signal handling ---------------------

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  
  // --------------------- robot communication setup ---------------------

  bool success = false;
  double cycle_time = 0.001;
  enum robif2b_ctrl_mode ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
  double pos_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double vel_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double eff_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double cur_msr[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double pos_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double vel_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double eff_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double cur_cmd[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double imu_ang_vel_msr[] = { 0.0, 0.0, 0.0 };
  double imu_lin_acc_msr[] = { 0.0, 0.0, 0.0 };

  struct robif2b_kinova_gen3_nbx rob = {
      // Configuration
      .conf = {
          .ip_address = "192.168.1.10",
          .port = 10000,
          .port_real_time = 10001,
          .user = "admin",
          .password = "admin",
          .session_timeout = 60000,
          .connection_timeout = 2000
      },
      // Connections
      .cycle_time = &cycle_time,
      .ctrl_mode = &ctrl_mode,
      .jnt_pos_msr = pos_msr,
      .jnt_vel_msr = vel_msr,
      .jnt_trq_msr = eff_msr,
      .act_cur_msr = cur_msr,
      .jnt_pos_cmd = pos_cmd,
      .jnt_vel_cmd = vel_cmd,
      .jnt_trq_cmd = eff_cmd,
      .act_cur_cmd = cur_cmd,
      .imu_ang_vel_msr = imu_ang_vel_msr,
      .imu_lin_acc_msr = imu_lin_acc_msr,
      .success = &success
  };

  // --------------------- ROS related ---------------------

  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);

  
  auto task_status = std::make_shared<TaskStatus>();
  auto fsm_interface = std::make_shared<FSMInterface>(rob);

  auto node = std::make_shared<TaskStatusROSNode>(task_status);
  // TODO: initialise action server node here

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  // TODO: add action server node here

  std::thread ros_thread([&executor]() {
      executor.spin();
  });

  // --------------------- control loop ---------------------

  int n = 0;
  TaskStatusData status;
  constexpr double DT = 0.001; // 1000 Hz control loop
  auto desired_loop_rate = std::chrono::microseconds(static_cast<int>(DT * 1e6));
  auto now = std::chrono::high_resolution_clock::now();
  auto deadline = now + desired_loop_rate;

  while (n < 30000 && !shutting_down.load()){
    fsm_interface->run_fsm();
    printf("hello world grc26 package\n");
    status.human_initiation = false;
    status.task_completion = false;
    status.obj_held_by_human = false;
    task_status->update(status);
    n++;

    if (fsm_interface->get_current_state() == S_EXIT) {
        LOG_INFO(node, "FSM reached exit state, breaking control loop");
        break;
    }

    while (now < deadline) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000)); // sleep for 1 ms to avoid busy waiting
        now = std::chrono::high_resolution_clock::now();
    }
    while (deadline < now) {
        deadline += desired_loop_rate;
    }
  }

  if (fsm_interface->is_in_comm_with_hw() == true) {
      std::cout << "Shutting down arm..." << std::endl;
      robif2b_kinova_gen3_stop(&rob);
      robif2b_kinova_gen3_shutdown(&rob);
  }

  std::cout << "Shutting down node..." << std::endl;
  executor.cancel();
  rclcpp::shutdown();
  if (ros_thread.joinable())
    ros_thread.join();
  return 0;
}