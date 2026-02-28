#include "grc26/task_state_node.hpp"

TaskStateROSNode::TaskStateROSNode(std::shared_ptr<TaskState> task_state)
    : Node("task_state_node"),
      task_state_(task_state)
{
    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();

    state_pub_ = create_publisher<grc26::msg::TaskStatus>(
        "/task_status", qos);

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(1000.0 / publish_rate_hz_)),
        std::bind(&TaskStateROSNode::publishCallback, this));
}

void TaskStateROSNode::publishCallback()
{
    TaskStatusData state;
    task_state_->getLatest(state);
    publishState(state);
}

void TaskStateROSNode::publishState(const TaskStatusData& state)
{
    grc26::msg::TaskStatus msg;

    msg.idle = state.idle;
    msg.human_initiation = state.human_initiation;
    msg.task_completion = state.task_completion;
    msg.obj_held_by_human = state.obj_held_by_human;

    state_pub_->publish(msg);
}