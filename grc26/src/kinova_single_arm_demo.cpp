
#include <atomic>
#include <csignal>
#include <chrono>
#include <cmath>
#include <cstdint>
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

  constexpr auto desired_loop_period = std::chrono::microseconds(1000);  // 1000 Hz control loop
  constexpr double desired_period_us = 1000.0;

  auto last_cycle_time = std::chrono::steady_clock::now();
  auto next_cycle_time = last_cycle_time + desired_loop_period;
  auto last_stats_log_time = last_cycle_time;
  std::uint64_t cycles_since_stats_log = 0;
  double max_abs_jitter_us = 0.0;

  while (!shutting_down.load()){
    fsm_interface->run_fsm();
    task_status->update(status);

    if (fsm_interface->get_current_state() == S_EXIT) {
      LOG_INFO(node, "FSM reached exit state, breaking control loop");
      break;
    }

    std::this_thread::sleep_until(next_cycle_time);
    const auto now = std::chrono::steady_clock::now();

    const auto dt_us = std::chrono::duration<double, std::micro>(now - last_cycle_time).count();
    const auto abs_jitter_us = std::abs(dt_us - desired_period_us);
    max_abs_jitter_us = std::max(max_abs_jitter_us, abs_jitter_us);

    ++cycles_since_stats_log;
    const auto stats_elapsed = now - last_stats_log_time;
    if (stats_elapsed >= std::chrono::seconds(1)) {
      const auto elapsed_s = std::chrono::duration<double>(stats_elapsed).count();
      const auto freq_hz = static_cast<double>(cycles_since_stats_log) / elapsed_s;
      LOG_INFO(node,
               "Control loop: %.1f Hz (target 1000.0), max abs jitter: %.1f us",
               freq_hz,
               max_abs_jitter_us);
      // log current state of FSM
      const auto current_state = fsm_interface->get_fsm_execution_state();
        const char* state_name =
          (current_state >= 0 && current_state < NUM_STATES)
            ? states[current_state].name
            : "UNKNOWN";
        LOG_INFO(node, "Current FSM state: %s (%d)", state_name, current_state);

      cycles_since_stats_log = 0;
      max_abs_jitter_us = 0.0;
      last_stats_log_time = now;
    }

    last_cycle_time = now;
    next_cycle_time += desired_loop_period;

    if (next_cycle_time <= now) {
      const auto behind = now - next_cycle_time;
      const auto missed_cycles = static_cast<std::int64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(behind).count() /
          desired_loop_period.count()) + 1;
      next_cycle_time += desired_loop_period * missed_cycles;
      RCLCPP_WARN_THROTTLE(node->get_logger(),
                           *node->get_clock(),
                           2000,
                           "Control loop overrun: missed %ld cycles",
                           missed_cycles);
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