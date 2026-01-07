#include "turtleshot_bt/conditions/target_in_range_condition.hpp"
#include <algorithm>

namespace turtleshot_bt
{

TargetInRangeCondition::TargetInRangeCondition(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::ConditionNode(name, config),
  node_(node),
  min_distance_(std::numeric_limits<double>::max())
{
  // Subscribe to laser scan
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    rclcpp::SensorDataQoS(),
    std::bind(&TargetInRangeCondition::scanCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "TargetInRangeCondition initialized");
}

BT::NodeStatus TargetInRangeCondition::tick()
{
  double range_threshold;
  if (!getInput("range", range_threshold)) {
    range_threshold = 0.5;  // default
  }

  if (min_distance_ <= range_threshold) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Target in range: %.2f m (threshold: %.2f m)",
      min_distance_, range_threshold);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_DEBUG(
    node_->get_logger(),
    "Target not in range: %.2f m (threshold: %.2f m)",
    min_distance_, range_threshold);
  return BT::NodeStatus::FAILURE;
}

void TargetInRangeCondition::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Find minimum distance in frontal cone (±30 degrees)
  size_t num_readings = msg->ranges.size();
  size_t angle_range = 30;  // degrees
  size_t readings_per_degree = num_readings / 360;
  size_t front_start = (360 - angle_range / 2) * readings_per_degree;
  size_t front_end = (angle_range / 2) * readings_per_degree;

  min_distance_ = std::numeric_limits<double>::max();

  // Check front portion
  for (size_t i = front_start; i < num_readings; ++i) {
    if (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
      min_distance_ = std::min(min_distance_, static_cast<double>(msg->ranges[i]));
    }
  }
  for (size_t i = 0; i < front_end; ++i) {
    if (msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
      min_distance_ = std::min(min_distance_, static_cast<double>(msg->ranges[i]));
    }
  }
}

}  // namespace turtleshot_bt
