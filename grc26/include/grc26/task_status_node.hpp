#ifndef TASK_STATUS_NODE_HPP
#define TASK_STATUS_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <grc26/msg/task_status.hpp>

#include "grc26/task_status.hpp"

#include "bdd_ros2_interfaces/msg/event.hpp"
#include "bdd_ros2_interfaces/msg/trinary_stamped.hpp"

class TaskStatusROSNode : public rclcpp::Node
{
public:
    explicit TaskStatusROSNode(std::shared_ptr<TaskStatus> task_status);

private:
    void publishCallback();
    void publishStatus(const TaskStatusData& status);

    std::shared_ptr<TaskStatus> task_status_;

    rclcpp::Publisher<bdd_ros2_interfaces::msg::Event>::SharedPtr event_pub_;

    rclcpp::Publisher<bdd_ros2_interfaces::msg::TrinaryStamped>::SharedPtr located_pick_pub_;
    rclcpp::Publisher<bdd_ros2_interfaces::msg::TrinaryStamped>::SharedPtr is_held_pub_;
    rclcpp::Publisher<bdd_ros2_interfaces::msg::TrinaryStamped>::SharedPtr located_place_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    int publish_rate_hz_ = 20;
};

#endif // TASK_STATUS_NODE_HPP