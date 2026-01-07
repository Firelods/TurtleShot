#ifndef TURTLESHOT_BT__ACTIONS__FIRE_CATAPULT_ACTION_HPP_
#define TURTLESHOT_BT__ACTIONS__FIRE_CATAPULT_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace turtleshot_bt
{

/**
 * @brief BT Action node for firing the catapult
 *
 * Simplified version without behaviortree_ros2 wrappers for Jazzy compatibility
 */
class FireCatapultAction : public BT::SyncActionNode
{
public:
  using Trigger = std_srvs::srv::Trigger;

  FireCatapultAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr service_client_;
};

}  // namespace turtleshot_bt

#endif  // TURTLESHOT_BT__ACTIONS__FIRE_CATAPULT_ACTION_HPP_
