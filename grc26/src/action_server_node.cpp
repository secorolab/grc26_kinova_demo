#include "grc26/action_server_node.hpp"

#include <functional>
#include <thread>

ActionServerNode::ActionServerNode(std::shared_ptr<BehaviourState> bhv_state)
    : Node("action_server_node"), bhv_state_(bhv_state)
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
    if (!bhv_state_->goal_done) {
        RCLCPP_WARN(this->get_logger(), "Received new goal while another is still active. Rejecting.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Behavior execution started");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServerNode::handle_cancel(
    const std::shared_ptr<GoalHandleBehaviour> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Behavior execution canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServerNode::handle_accepted(
    const std::shared_ptr<GoalHandleBehaviour> goal_handle)
{
    std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
}

void ActionServerNode::execute(
    const std::shared_ptr<GoalHandleBehaviour> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    bhv_state_->bhv_ctx_id = goal->scenario_context_id;
    bhv_state_->goal_done = false;

    auto response = std::make_shared<Behaviour::Result>();
    response->result.scenario_context_id = bhv_state_->bhv_ctx_id;

    auto feedback = std::make_shared<Behaviour::Feedback>();
    feedback->scenario_context_id = bhv_state_->bhv_ctx_id;

    bhv_state_->goal_in = true;

    rclcpp::WallRate rate(100.0);
    while (rclcpp::ok() && !bhv_state_->goal_done)
    {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(response);
            RCLCPP_INFO(this->get_logger(), "Behavior execution cancel requested");
            return;
        }

        rate.sleep();
    }
    
    response->result.stamp = this->now();
    response->result.trinary.value = bdd_ros2_interfaces::msg::Trinary::TRUE;

    goal_handle->succeed(response);
    RCLCPP_INFO(this->get_logger(), "Behavior execution succeeded");
}