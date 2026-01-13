#ifndef TURTLESHOT_BT__CONDITIONS__HAS_TARGET_CONDITION_HPP_
#define TURTLESHOT_BT__CONDITIONS__HAS_TARGET_CONDITION_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace turtleshot_bt
{

/**
 * @brief BT Condition that checks if a target (person with ball) is detected
 */
class HasTargetCondition : public BT::ConditionNode
{
public:
  HasTargetCondition(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target_type", "person", "Type of target to detect"),
      BT::OutputPort<double>("target_x", "Detected X position in map frame"),
      BT::OutputPort<double>("target_y", "Detected Y position in map frame"),
      BT::OutputPort<double>("target_yaw", "Detected orientation in radians")
    };
  }

  BT::NodeStatus tick() override;

private:
  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  bool target_detected_;
  std::string target_type_;
  double detected_x_;
  double detected_y_;
  double detected_yaw_;
};

}  // namespace turtleshot_bt

#endif  // TURTLESHOT_BT__CONDITIONS__HAS_TARGET_CONDITION_HPP_
