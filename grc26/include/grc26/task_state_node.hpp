#ifndef TASK_STATE_NODE_HPP
#define TASK_STATE_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <grc26/msg/task_status.hpp>

#include "grc26/task_state.hpp"

class TaskStateROSNode : public rclcpp::Node
{
public:
    explicit TaskStateROSNode(std::shared_ptr<TaskState> task_state);

private:
    void publishCallback();
    void publishState(const TaskStatusData& state);

    std::shared_ptr<TaskState> task_state_;

    rclcpp::Publisher<grc26::msg::TaskStatus>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    int publish_rate_hz_ = 20;
};

#endif // TASK_STATE_NODE_HPP