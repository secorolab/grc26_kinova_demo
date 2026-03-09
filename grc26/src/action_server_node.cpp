#include "grc26/action_server_node.hpp"

#include <functional>
#include <thread>

ActionServerNode::ActionServerNode(std::shared_ptr<TaskStatus> task_status)
    : Node("action_server_node"), task_status_(task_status)
{
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Behaviour>(
        this,
        "bhv_server",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Behaviour::Goal> goal) {
            return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandleBehaviour> goal_handle) {
            return this->handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<GoalHandleBehaviour> goal_handle) {
            this->handle_accepted(goal_handle);
        }
    );
}

rclcpp_action::GoalResponse ActionServerNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Behaviour::Goal> goal)
{
    (void)uuid;
    TaskStatusData current_status;
    task_status_->getLatest(current_status);
    if (current_status.goal_in) {
        RCLCPP_WARN(this->get_logger(), "Received new goal while another is still active. Rejecting.");
        return rclcpp_action::GoalResponse::REJECT;
    }

    current_status.goal_in = true;
    task_status_->update(current_status);

    RCLCPP_INFO(this->get_logger(), "Behavior execution started");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServerNode::handle_cancel(
    const std::shared_ptr<GoalHandleBehaviour> goal_handle)
{
    (void)goal_handle;
    TaskStatusData current_status;
    task_status_->getLatest(current_status);
    RCLCPP_INFO(this->get_logger(), "Behavior execution canceled");
    current_status.goal_in = false;
    task_status_->update(current_status);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServerNode::handle_accepted(
    const std::shared_ptr<GoalHandleBehaviour> goal_handle)
{
    TaskStatusData current_status;
    task_status_->getLatest(current_status);
    current_status.goal_in = true;
    task_status_->update(current_status);
    std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
}

void ActionServerNode::execute(
    const std::shared_ptr<GoalHandleBehaviour> goal_handle)
{
    TaskStatusData current_status;
    task_status_->getLatest(current_status);
    const auto goal = goal_handle->get_goal();
    current_status.bhv_ctx_id = goal->scenario_context_id;
    current_status.goal_in = true;
    current_status.task_completed = false;
    task_status_->update(current_status);

    auto response = std::make_shared<Behaviour::Result>();
    response->result.scenario_context_id = current_status.bhv_ctx_id;

    auto feedback = std::make_shared<Behaviour::Feedback>();
    feedback->scenario_context_id = current_status.bhv_ctx_id;

    rclcpp::WallRate rate(100.0);
    while (rclcpp::ok())
    {
        task_status_->getLatest(current_status);
        if (current_status.task_completed) {
            break;
        }

        if (goal_handle->is_canceling()) {
            current_status.goal_in = false;
            task_status_->update(current_status);
            goal_handle->canceled(response);
            RCLCPP_INFO(this->get_logger(), "Behavior execution cancel requested");
            return;
        }
        rate.sleep();
    }

    response->result.stamp = this->now();
    task_status_->getLatest(current_status);
    response->result.trinary.value = current_status.task_completed ? bdd_ros2_interfaces::msg::Trinary::TRUE : bdd_ros2_interfaces::msg::Trinary::FALSE;

    goal_handle->succeed(response);
    current_status.goal_in = false;
    task_status_->update(current_status);
    RCLCPP_INFO(this->get_logger(), "Behavior execution succeeded");
}