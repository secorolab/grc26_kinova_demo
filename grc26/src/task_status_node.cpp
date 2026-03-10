#include "grc26/task_status_node.hpp"

#include <string>

namespace {

const std::string kBddTemplateNamespace =
    "https://secorolab.github.io/models/acceptance-criteria/bdd/templates/";

std::string makeTemplateEventUri(const char * event_name)
{
    return kBddTemplateNamespace + event_name;
}

}  // namespace

TaskStatusROSNode::TaskStatusROSNode(std::shared_ptr<TaskStatus> task_status)
    : Node("task_status_node"),
      task_status_(task_status)
{
    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();

    event_pub_ = create_publisher<bdd_ros2_interfaces::msg::Event>(
        "/bdd/events", qos);

    located_pick_pub_ = create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>(
        "/obs_policy/located_at_pick_ws", qos);
    is_held_pub_ = create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>(
        "/obs_policy/is_held_ws", qos);
    located_place_pub_ = create_publisher<bdd_ros2_interfaces::msg::TrinaryStamped>(
        "/obs_policy/located_at_place_ws", qos);

    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(1000.0 / publish_rate_hz_)),
        std::bind(&TaskStatusROSNode::publishCallback, this));
}

void TaskStatusROSNode::publishCallback()
{
    TaskStatusData status;
    task_status_->consumeLatest(status);
    publishStatus(status);
}

void TaskStatusROSNode::publishStatus(const TaskStatusData& status)
{
    if (status.is_obj_located_at_pick_location) {
        bdd_ros2_interfaces::msg::TrinaryStamped msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.trinary.value = bdd_ros2_interfaces::msg::Trinary::TRUE;

        located_pick_pub_->publish(msg);
    }
    if (status.is_obj_held_by_robot) {
        bdd_ros2_interfaces::msg::TrinaryStamped msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.trinary.value = bdd_ros2_interfaces::msg::Trinary::TRUE;
        
        is_held_pub_->publish(msg);
    }
    if (status.is_obj_located_at_place_location) {
        bdd_ros2_interfaces::msg::TrinaryStamped msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.trinary.value =bdd_ros2_interfaces::msg::Trinary::TRUE;

        located_place_pub_->publish(msg);
    }
    if (status.is_pick_start) {
        bdd_ros2_interfaces::msg::Event msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.uri = makeTemplateEventUri("evt-pick-start");
        
        event_pub_->publish(msg);
    }
    if (status.is_pick_end) {
        bdd_ros2_interfaces::msg::Event msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.uri = makeTemplateEventUri("evt-pick-end");
        
        event_pub_->publish(msg);
    }
    if (status.is_place_start) {
        bdd_ros2_interfaces::msg::Event msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.uri = makeTemplateEventUri("evt-place-start");
        
        event_pub_->publish(msg);
    }
    if (status.is_place_end) {
        bdd_ros2_interfaces::msg::Event msg;
        msg.stamp = this->now();
        msg.scenario_context_id = status.bhv_ctx_id;
        msg.uri = makeTemplateEventUri("evt-place-end");
        
        event_pub_->publish(msg);
    }
}