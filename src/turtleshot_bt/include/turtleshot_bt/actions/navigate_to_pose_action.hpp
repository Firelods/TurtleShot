#ifndef TURTLESHOT_BT__ACTIONS__NAVIGATE_TO_POSE_ACTION_HPP_
#define TURTLESHOT_BT__ACTIONS__NAVIGATE_TO_POSE_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace turtleshot_bt
{

/**
 * @brief BT Action node for Nav2 NavigateToPose
 *
 * Simplified version without behaviortree_ros2 wrappers for Jazzy compatibility
 */
class NavigateToPoseAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseAction(
    const std::string & name,
    const BT::NodeConfiguration & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x", "Target X position (meters)"),
      BT::InputPort<double>("y", "Target Y position (meters)"),
      BT::InputPort<double>("yaw", 0.0, "Target yaw orientation (radians)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::shared_future<GoalHandleNav::SharedPtr> goal_handle_future_;
  GoalHandleNav::SharedPtr goal_handle_;
  bool goal_sent_;
  bool goal_result_available_;
  BT::NodeStatus result_status_;
};

}  // namespace turtleshot_bt

#endif  // TURTLESHOT_BT__ACTIONS__NAVIGATE_TO_POSE_ACTION_HPP_
