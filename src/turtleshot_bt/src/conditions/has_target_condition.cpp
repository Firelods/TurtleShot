#include "turtleshot_bt/conditions/has_target_condition.hpp"

namespace turtleshot_bt
{

HasTargetCondition::HasTargetCondition(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::ConditionNode(name, config),
  node_(node),
  target_detected_(false),
  detected_x_(0.0),
  detected_y_(0.0),
  detected_yaw_(0.0)
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
    RCLCPP_INFO(node_->get_logger(), "Target '%s' detected at (%.2f, %.2f, %.2f)!", 
                target_type_.c_str(), detected_x_, detected_y_, detected_yaw_);
    
    // Publish detected position to blackboard
    std::string prefix = (target_type_ == "person") ? "person" : "trash";
    setOutput(prefix + "_x", detected_x_);
    setOutput(prefix + "_y", detected_y_);
    setOutput(prefix + "_yaw", detected_yaw_);
    
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
        
        // Extract position from detection bounding box center
        // TODO: Convert from image coordinates to map coordinates using TF
        // For now, use dummy values (requires integration with TF and camera calibration)
        detected_x_ = detection.bbox.center.position.x / 100.0;  // Placeholder conversion
        detected_y_ = detection.bbox.center.position.y / 100.0;  // Placeholder conversion
        detected_yaw_ = 0.0;  // Placeholder - should compute from robot orientation
        
        RCLCPP_DEBUG(node_->get_logger(), "Detected %s at bbox center (%.1f, %.1f)",
                     target_type_.c_str(), 
                     detection.bbox.center.position.x,
                     detection.bbox.center.position.y);
        return;
      }
    }
  }
}

}  // namespace turtleshot_bt
