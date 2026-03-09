
#include <atomic>
#include <csignal>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "grc26/action_server_node.hpp"
#include "grc26/task_status.hpp"
#include "grc26/task_status_node.hpp"
#include "grc26/debug_state_node.hpp"
#include "grc26/debug_signals.hpp"
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

void signal_handler(int /*signum*/) 
{
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
  system_state.ft_sensor.present = true;

  TaskStatusData status;

  robif2b_kinova_gen3_nbx arm;
  arm.conf = {
        .ip_address = "192.168.1.10",
        .port = 10000,
        .port_real_time = 10001,
        .user = "admin",
        .password = "admin",
        .session_timeout = 60000,
        .connection_timeout = 2000};

  bindKinovaArm(arm, system_state);

  robif2b_kg3_robotiq_gripper_nbx gripper;
  bindRobotiqGripper(gripper, system_state);

  robif2b_robotiq_ft_nbx ft_sensor;
  ft_sensor.conf.device = "/dev/ttyUSB0";
  ft_sensor.conf.baudrate = 19200;
  FTIOBuffer ft_io_buffer;
  ft_sensor.force_x = &ft_io_buffer.fx;
  ft_sensor.force_y = &ft_io_buffer.fy;
  ft_sensor.force_z = &ft_io_buffer.fz;
  ft_sensor.torque_x = &ft_io_buffer.tx;
  ft_sensor.torque_y = &ft_io_buffer.ty;
  ft_sensor.torque_z = &ft_io_buffer.tz;
  ft_sensor.wrench = &ft_io_buffer.wrench[0];
  ft_sensor.new_data = &ft_io_buffer.new_data;
  ft_sensor.state = &ft_io_buffer.ft_state;
  ft_sensor.success = &ft_io_buffer.success;

  FTSnapshot ft_snapshot;
  std::mutex ft_snapshot_mutex;
  std::atomic_bool ft_reader_stop{false};
  std::atomic_bool ft_hw_active{false};

  std::thread ft_reader_thread([&]() {
    constexpr auto ft_period = std::chrono::milliseconds(10); // 100 Hz
    const auto round2 = [](float value) -> float {
      return std::round(value * 100.0f) / 100.0f;
    };
    auto next_ft_cycle = std::chrono::steady_clock::now() + ft_period;
    while (!ft_reader_stop.load() && !shutting_down.load()) {
      if (ft_hw_active.load(std::memory_order_relaxed) && system_state.ft_sensor.present) {
        robif2b_robotiq_ft_update(&ft_sensor);

        std::lock_guard<std::mutex> lock(ft_snapshot_mutex);
        ft_snapshot.fx = round2(ft_io_buffer.fx);
        ft_snapshot.fy = round2(ft_io_buffer.fy);
        ft_snapshot.fz = round2(ft_io_buffer.fz);
        ft_snapshot.tx = round2(ft_io_buffer.tx);
        ft_snapshot.ty = round2(ft_io_buffer.ty);
        ft_snapshot.tz = round2(ft_io_buffer.tz);
        for (int i = 0; i < 6; ++i) {
          ft_snapshot.wrench[i] = round2(ft_io_buffer.wrench[i]);
        }
        ft_snapshot.new_data = ft_io_buffer.new_data;
        ft_snapshot.success = ft_io_buffer.success;
        ft_snapshot.ft_state = ft_io_buffer.ft_state;
        ft_snapshot.sample_time = std::chrono::steady_clock::now();
        ++ft_snapshot.sequence;
      }

      std::this_thread::sleep_until(next_ft_cycle);
      next_ft_cycle += ft_period;
      const auto now = std::chrono::steady_clock::now();
      if (next_ft_cycle <= now) {
        next_ft_cycle = now + ft_period;
      }
    }
  });

  // --------------------- ROS related ---------------------

  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);

  auto task_status = std::make_shared<TaskStatus>();
  auto debug_buffer = std::make_shared<DebugSignalBuffer>(4000);
  auto fsm_interface = std::make_shared<FSMInterface>(system_state, arm, gripper, ft_sensor, status);
  auto bhv_state = std::make_shared<BehaviourState>();

  auto node = std::make_shared<TaskStatusROSNode>(task_status);
  auto debug_node = std::make_shared<DebugStateROSNode>(debug_buffer);
  auto action_server_node = std::make_shared<ActionServerNode>(task_status);

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.add_node(debug_node);
  executor.add_node(action_server_node);

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
  std::uint64_t last_ft_sequence = 0;
  std::uint64_t last_debug_sequence = 0;
  auto latest_ft_sample_time = std::chrono::steady_clock::time_point{};

  while (!shutting_down.load()){
    ft_hw_active.store(fsm_interface->is_in_comm_with_hw(), std::memory_order_relaxed);

    if (system_state.ft_sensor.present) {
      std::lock_guard<std::mutex> lock(ft_snapshot_mutex);
      if (ft_snapshot.sequence != last_ft_sequence) {
        last_ft_sequence = ft_snapshot.sequence;
        latest_ft_sample_time = ft_snapshot.sample_time;

        system_state.ft_sensor.fx = ft_snapshot.fx;
        system_state.ft_sensor.fy = ft_snapshot.fy;
        system_state.ft_sensor.fz = ft_snapshot.fz;
        system_state.ft_sensor.tx = ft_snapshot.tx;
        system_state.ft_sensor.ty = ft_snapshot.ty;
        system_state.ft_sensor.tz = ft_snapshot.tz;
        for (int i = 0; i < 6; ++i) {
          system_state.ft_sensor.wrench[i] = ft_snapshot.wrench[i];
        }
        system_state.ft_sensor.new_data = ft_snapshot.new_data;
        system_state.ft_sensor.success = ft_snapshot.success;
        system_state.ft_sensor.ft_state = ft_snapshot.ft_state;
      }
    }

    if (last_ft_sequence > 0) {
      const auto sample_age = std::chrono::steady_clock::now() - latest_ft_sample_time;
      if (sample_age > std::chrono::milliseconds(50)) {
        RCLCPP_WARN_THROTTLE(node->get_logger(),
                             *node->get_clock(),
                             2000,
                             "FT sample is stale (age: %.1f ms)",
                             std::chrono::duration<double, std::milli>(sample_age).count());
      }
    }

    TaskStatusData latest_status;
    task_status->getLatest(latest_status);

    // Sync action-server-owned fields into the FSM-visible status
    status.goal_in = latest_status.goal_in;
    status.bhv_ctx_id = latest_status.bhv_ctx_id;
    status.task_completed = latest_status.task_completed;

    fsm_interface->run_fsm();

    DebugSample debug_sample;
    if (fsm_interface->getLatestDebugSample(debug_sample) &&
        debug_sample.sequence != last_debug_sequence) {
      last_debug_sequence = debug_sample.sequence;
      debug_buffer->push(debug_sample);
    }

    TaskStatusData latest_status_after_fsm;
    task_status->getLatest(latest_status_after_fsm);

    TaskStatusData merged_status = status;
    merged_status.goal_in = latest_status_after_fsm.goal_in;
    merged_status.bhv_ctx_id = latest_status_after_fsm.bhv_ctx_id;
    merged_status.task_completed = status.task_completed || latest_status_after_fsm.task_completed;

    task_status->update(merged_status);

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
    if (stats_elapsed >= std::chrono::seconds(2)) {
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

      // log FT sensor state
      if (system_state.ft_sensor.present) {
        LOG_INFO(node, "F: [%6.2f, %6.2f, %6.2f] N  T: [%6.3f, %6.3f, %6.3f] Nm\n",
                system_state.ft_sensor.wrench[0],
                system_state.ft_sensor.wrench[1],
                system_state.ft_sensor.wrench[2],
                system_state.ft_sensor.wrench[3],
                system_state.ft_sensor.wrench[4],
                system_state.ft_sensor.wrench[5]);
      }
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

  ft_reader_stop.store(true);
  if (ft_reader_thread.joinable()) {
    ft_reader_thread.join();
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