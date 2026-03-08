#ifndef DEBUG_STATE_NODE_HPP
#define DEBUG_STATE_NODE_HPP

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "grc26/debug_signals.hpp"
#include "grc26/msg/cartesian_pid_debug.hpp"

class DebugStateROSNode : public rclcpp::Node
{
public:
  explicit DebugStateROSNode(std::shared_ptr<DebugSignalBuffer> debug_buffer);

private:
  void publishCallback();
  void publishSample(const DebugSample& sample);

  std::shared_ptr<DebugSignalBuffer> debug_buffer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ee_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_sig_pub_;
  rclcpp::Publisher<grc26::msg::CartesianPIDDebug>::SharedPtr pid_debug_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  int publish_rate_hz_ = 100;
  std::size_t max_samples_per_tick_ = 20;
  std::vector<std::string> joint_names_;
};

#endif // DEBUG_STATE_NODE_HPP