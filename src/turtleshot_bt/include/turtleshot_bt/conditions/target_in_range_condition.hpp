#ifndef TURTLESHOT_BT__CONDITIONS__TARGET_IN_RANGE_CONDITION_HPP_
#define TURTLESHOT_BT__CONDITIONS__TARGET_IN_RANGE_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace turtleshot_bt
{

/**
 * @brief BT Condition that checks if target is within range
 */
class TargetInRangeCondition : public BT::ConditionNode
{
public:
  TargetInRangeCondition(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("range", 0.5, "Target range threshold (meters)")
    };
  }

  BT::NodeStatus tick() override;

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  double min_distance_;
};

}  // namespace turtleshot_bt

#endif  // TURTLESHOT_BT__CONDITIONS__TARGET_IN_RANGE_CONDITION_HPP_
