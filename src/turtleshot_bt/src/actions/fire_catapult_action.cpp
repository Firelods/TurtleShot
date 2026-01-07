#include "turtleshot_bt/actions/fire_catapult_action.hpp"

namespace turtleshot_bt
{

FireCatapultAction::FireCatapultAction(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::SyncActionNode(name, config),
  node_(node)
{
  // Create service client for catapaf arm launch service
  service_client_ = node_->create_client<Trigger>("catapaf_arm/launch");

  RCLCPP_INFO(node_->get_logger(), "FireCatapultAction initialized");
}

BT::NodeStatus FireCatapultAction::tick()
{
  // Wait for service
  if (!service_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_->get_logger(), "Fire catapult service not available");
    return BT::NodeStatus::FAILURE;
  }

  // Create request
  auto request = std::make_shared<Trigger::Request>();

  RCLCPP_INFO(node_->get_logger(), "Firing catapult!");

  // Call service synchronously
  auto future = service_client_->async_send_request(request);

  // Wait for response (with timeout)
  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(node_->get_logger(), "Catapult fired successfully: %s", response->message.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to fire catapult: %s", response->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Service call timed out");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace turtleshot_bt
