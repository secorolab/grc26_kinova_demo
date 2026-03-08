#include "grc26/debug_state_node.hpp"

namespace {
grc26::msg::PIDDebug toPidMsg(const PIDAxisDebug& axis)
{
  grc26::msg::PIDDebug msg;
  msg.p = axis.p;
  msg.i = axis.i;
  msg.d = axis.d;
  msg.error = axis.error;
  msg.control_sig = axis.control_sig;
  return msg;
}
}

DebugStateROSNode::DebugStateROSNode(std::shared_ptr<DebugSignalBuffer> debug_buffer)
  : Node("debug_state_node"),
    debug_buffer_(std::move(debug_buffer)),
    joint_names_{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"}
{
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/debug_js", 10);
  ee_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/debug_ee_vel", 10);
  error_pub_ = create_publisher<geometry_msgs::msg::Twist>("/debug_ee_vel_error", 10);
  control_sig_pub_ = create_publisher<geometry_msgs::msg::Twist>("/debug_ee_vel_cntrl_sig", 10);
  pid_debug_pub_ = create_publisher<grc26::msg::CartesianPIDDebug>("/pid_debug", 10);

  publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_)),
    std::bind(&DebugStateROSNode::publishCallback, this));
}

void DebugStateROSNode::publishCallback()
{
  const auto samples = debug_buffer_->drain(max_samples_per_tick_);
  for (const auto& sample : samples) {
    publishSample(sample);
  }
}

void DebugStateROSNode::publishSample(const DebugSample& sample)
{
  const auto stamp = now();

  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = stamp;
  joint_msg.name = joint_names_;
  joint_msg.position.assign(sample.joint_position.begin(), sample.joint_position.end());
  joint_msg.velocity.assign(sample.joint_velocity.begin(), sample.joint_velocity.end());
  joint_state_pub_->publish(joint_msg);

  geometry_msgs::msg::Twist ee_vel_msg;
  ee_vel_msg.linear.x = sample.ee_vel[0];
  ee_vel_msg.linear.y = sample.ee_vel[1];
  ee_vel_msg.linear.z = sample.ee_vel[2];
  ee_vel_msg.angular.x = sample.ee_vel[3];
  ee_vel_msg.angular.y = sample.ee_vel[4];
  ee_vel_msg.angular.z = sample.ee_vel[5];
  ee_vel_pub_->publish(ee_vel_msg);

  geometry_msgs::msg::Twist err_msg;
  err_msg.linear.x = sample.ee_vel_error[0];
  err_msg.linear.y = sample.ee_vel_error[1];
  err_msg.linear.z = sample.ee_vel_error[2];
  err_msg.angular.x = sample.ee_vel_error[3];
  err_msg.angular.y = sample.ee_vel_error[4];
  err_msg.angular.z = sample.ee_vel_error[5];
  error_pub_->publish(err_msg);

  geometry_msgs::msg::Twist control_msg;
  control_msg.linear.x = sample.control_signal[0];
  control_msg.linear.y = sample.control_signal[1];
  control_msg.linear.z = sample.control_signal[2];
  control_msg.angular.x = sample.control_signal[3];
  control_msg.angular.y = sample.control_signal[4];
  control_msg.angular.z = sample.control_signal[5];
  control_sig_pub_->publish(control_msg);

  grc26::msg::CartesianPIDDebug pid_msg;
  pid_msg.x = toPidMsg(sample.pid_axes[0]);
  pid_msg.y = toPidMsg(sample.pid_axes[1]);
  pid_msg.z = toPidMsg(sample.pid_axes[2]);
  pid_msg.roll = toPidMsg(sample.pid_axes[3]);
  pid_msg.pitch = toPidMsg(sample.pid_axes[4]);
  pid_msg.yaw = toPidMsg(sample.pid_axes[5]);
  pid_debug_pub_->publish(pid_msg);
}