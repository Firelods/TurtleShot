#include "turtleshot_bt/conditions/has_target_condition.hpp"

namespace turtleshot_bt
{

HasTargetCondition::HasTargetCondition(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::ConditionNode(name, config),
  node_(node),
  target_detected_(false)
{
  // Subscribe to detections topic
  detection_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detections",
    10,
    std::bind(&HasTargetCondition::detectionCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "HasTargetCondition initialized");
}

BT::NodeStatus HasTargetCondition::tick()
{
  // Get target type from port
  if (!getInput("target_type", target_type_)) {
    target_type_ = "person";  // default
  }

  if (target_detected_) {
    RCLCPP_INFO(node_->get_logger(), "Target '%s' detected!", target_type_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Waiting for target '%s'...", target_type_.c_str());
  return BT::NodeStatus::FAILURE;
}

void HasTargetCondition::detectionCallback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  target_detected_ = false;

  for (const auto & detection : msg->detections) {
    for (const auto & result : detection.results) {
      std::string class_id = result.hypothesis.class_id;

      // Convert to lowercase for comparison
      std::transform(class_id.begin(), class_id.end(), class_id.begin(), ::tolower);
      std::string target_lower = target_type_;
      std::transform(target_lower.begin(), target_lower.end(), target_lower.begin(), ::tolower);

      if (class_id.find(target_lower) != std::string::npos) {
        target_detected_ = true;
        return;
      }
    }
  }
}

}  // namespace turtleshot_bt
