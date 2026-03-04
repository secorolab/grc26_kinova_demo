
#include <atomic>
#include <csignal>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "grc26/task_status.hpp"
#include "grc26/task_status_node.hpp"
#include "grc26/msg/task_status.hpp"
#include "grc26/fsm_interface.hpp"
#include "grc26/system_state.hpp"
#include "grc26/hardware_binding.hpp"
#include "robif2b/functions/kinova_gen3.h"
#include "robif2b/functions/robotiq_ft_sensor.h"
#include "grc26/kinova_single_arm_demo.fsm.hpp"

#define LOG_INFO(node, msg, ...) RCLCPP_INFO(node->get_logger(), msg, ##__VA_ARGS__)
#define LOG_ERROR(node, msg, ...) RCLCPP_ERROR(node->get_logger(), msg, ##__VA_ARGS__)

#define LOG_INFO_S(node, expr) RCLCPP_INFO_STREAM((node)->get_logger(), expr)
#define LOG_ERROR_S(node, expr) RCLCPP_ERROR_STREAM((node)->get_logger(), expr)

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

  SystemState system_state;
  system_state.gripper.present = false;
  system_state.arm.present = true;
  system_state.ft_sensor.present = false;
  TaskStatusData status;

  robif2b_kinova_gen3_nbx arm;
  arm.conf = {
            .ip_address = "192.168.1.10",
            .port = 10000,
            .port_real_time = 10001,
            .user = "admin",
            .password = "admin",
            .session_timeout = 60000,
            .connection_timeout = 2000
        };
  bindKinovaArm(arm, system_state);

  robif2b_kg3_robotiq_gripper_nbx gripper;
  bindRobotiqGripper(gripper, system_state);

  robif2b_robotiq_ft_nbx ft_sensor;
  ft_sensor.conf.device = "/dev/ttyUSB0";
  ft_sensor.conf.baudrate = 19200;
  bindRobotiqFT(ft_sensor, system_state);

  // --------------------- ROS related ---------------------

  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);

  auto task_status = std::make_shared<TaskStatus>();
  auto fsm_interface = std::make_shared<FSMInterface>(system_state, arm, gripper, ft_sensor, status);

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

  constexpr double DT = 0.001; // 1000 Hz control loop
  auto desired_loop_rate = std::chrono::microseconds(static_cast<int>(DT * 1e6));
  auto now = std::chrono::high_resolution_clock::now();
  auto deadline = now + desired_loop_rate;

  while (!shutting_down.load()){
    fsm_interface->run_fsm();
    task_status->update(status);

    // print current time
    auto now_time = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    LOG_INFO(node, "Current time: %ld ms", now_time);
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
    if (system_state.ft_sensor.present) {
      std::cout << "Stopping FT sensor..." << std::endl;
      robif2b_robotiq_ft_stop(&ft_sensor);
      robif2b_robotiq_ft_shutdown(&ft_sensor);
    }
    if (system_state.gripper.present) {
      std::cout << "Stopping gripper..." << std::endl;
      robif2b_kg3_robotiq_gripper_stop(&gripper);
    }
    std::cout << "Shutting down arm..." << std::endl;
    robif2b_kinova_gen3_stop(&arm);
    robif2b_kinova_gen3_shutdown(&arm);
  }

  std::cout << "Shutting down node..." << std::endl;
  executor.cancel();
  rclcpp::shutdown();
  if (ros_thread.joinable())
    ros_thread.join();
  return 0;
}