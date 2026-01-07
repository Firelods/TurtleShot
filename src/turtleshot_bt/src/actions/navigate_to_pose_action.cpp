#include "turtleshot_bt/actions/navigate_to_pose_action.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace turtleshot_bt
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  goal_sent_(false),
  goal_result_available_(false),
  result_status_(BT::NodeStatus::FAILURE)
{
  // Create action client
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

  RCLCPP_INFO(node_->get_logger(), "NavigateToPoseAction initialized");
}

BT::NodeStatus NavigateToPoseAction::onStart()
{
  // Reset state
  goal_sent_ = false;
  goal_result_available_ = false;
  result_status_ = BT::NodeStatus::FAILURE;

  // Get input parameters
  double x, y, yaw;
  if (!getInput("x", x)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [x]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("y", y)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [y]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("yaw", yaw)) {
    yaw = 0.0;
  }

  // Wait for action server
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    return BT::NodeStatus::FAILURE;
  }

  // Create goal
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.position.z = 0.0;

  // Convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  RCLCPP_INFO(
    node_->get_logger(),
    "Sending NavigateToPose goal: x=%.2f, y=%.2f, yaw=%.2f",
    x, y, yaw);

  // Send goal
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult & result) {
      goal_result_available_ = true;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "Navigation succeeded!");
        result_status_ = BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Navigation failed");
        result_status_ = BT::NodeStatus::FAILURE;
      }
    };

  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onRunning()
{
  // Check if result is available
  if (goal_result_available_) {
    return result_status_;
  }

  // Still running
  return BT::NodeStatus::RUNNING;
}

void NavigateToPoseAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "NavigateToPoseAction halted");

  // Cancel goal if it was sent
  if (goal_sent_ && !goal_result_available_) {
    if (goal_handle_future_.valid() &&
        goal_handle_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      auto goal_handle = goal_handle_future_.get();
      if (goal_handle) {
        action_client_->async_cancel_goal(goal_handle);
      }
    }
  }
}

}  // namespace turtleshot_bt
