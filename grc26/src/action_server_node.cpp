#include "grc26/action_server_node.hpp"

#include <functional>
#include <thread>

ActionServerNode::ActionServerNode()
    : Node("action_server_node")
{
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Behaviour>(
        this,
        "bhv_server",
        std::bind(&ActionServerNode::handle_goal, this, _1, _2),
        std::bind(&ActionServerNode::handle_cancel, this, _1),
        std::bind(&ActionServerNode::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse ActionServerNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Behaviour::Goal> goal)
{
    (void)uuid;
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
    auto bhv_ctx_id = goal->scenario_context_id;

    auto response = std::make_shared<Behaviour::Result>();
    response->result.scenario_context_id = bhv_ctx_id;

    auto feedback = std::make_shared<Behaviour::Feedback>();
    feedback->scenario_context_id = bhv_ctx_id;

    if (goal_handle->is_canceling()) {
        goal_handle->canceled(response);
        RCLCPP_INFO(this->get_logger(), "Behavior execution cancel requested");
        return;
    }

    //TODO: wait for goal done

    goal_handle->succeed(response);
    RCLCPP_INFO(this->get_logger(), "Behavior execution succeeded");
}