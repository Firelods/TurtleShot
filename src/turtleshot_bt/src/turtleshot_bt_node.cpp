#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/decorators/retry_node.h>
#include <behaviortree_cpp/controls/fallback_node.h>
#include <behaviortree_cpp/controls/reactive_sequence.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_srvs/srv/trigger.hpp>

// Include custom BT nodes
#include "turtleshot_bt/actions/navigate_to_pose_action.hpp"
#include "turtleshot_bt/actions/fire_catapult_action.hpp"
#include "turtleshot_bt/conditions/has_target_condition.hpp"
#include "turtleshot_bt/conditions/target_in_range_condition.hpp"

using namespace std::chrono_literals;

// -----------------------------------------------------------------------------
// Attendre que Nav2 (lifecycle_manager_navigation) soit ACTIVE
// -> service /lifecycle_manager_navigation/is_active (std_srvs/Trigger)
// -----------------------------------------------------------------------------
bool waitForNav2Active(const rclcpp::Node::SharedPtr & node,
                       std::chrono::seconds timeout_total = std::chrono::seconds(90))
{
  using Trigger = std_srvs::srv::Trigger;

  auto client =
    node->create_client<Trigger>("/lifecycle_manager_navigation/is_active");

  auto start = std::chrono::steady_clock::now();
  RCLCPP_INFO(node->get_logger(),
              "Waiting for Nav2 lifecycle_manager_navigation to be ACTIVE...");

  // Attendre que le service soit dispo
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for Nav2 is_active service");
      return false;
    }
    if (std::chrono::steady_clock::now() - start > timeout_total) {
      RCLCPP_ERROR(node->get_logger(),
                   "Timeout waiting for Nav2 is_active service");
      return false;
    }
    RCLCPP_INFO(node->get_logger(),
                "is_active service not available yet, retrying...");
  }

  // Interroger jusqu’à ce que Nav2 soit active (Trigger.success == true)
  while (rclcpp::ok()) {
    auto request = std::make_shared<Trigger::Request>();
    auto future = client->async_send_request(request);

    if (future.wait_for(2s) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(node->get_logger(),
                    "Nav2 is ACTIVE, starting BT mission!");
        return true;
      } else {
        RCLCPP_INFO(node->get_logger(),
                    "Nav2 not active yet, waiting... (message: %s)",
                    response->message.c_str());
      }
    } else {
      RCLCPP_WARN(node->get_logger(),
                  "No response from is_active service yet");
    }

    if (std::chrono::steady_clock::now() - start > timeout_total) {
      RCLCPP_ERROR(node->get_logger(),
                   "Timeout waiting for Nav2 to become active");
      return false;
    }

    std::this_thread::sleep_for(1s);
  }

  return false;
}

// -----------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create ROS2 node
  auto node = std::make_shared<rclcpp::Node>("turtleshot_bt");

  RCLCPP_INFO(node->get_logger(),
              "=== TurtleShot Behavior Tree Orchestrator ===");

  // Executor pour traiter les callbacks (actions, services, etc.)
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  // Create BehaviorTree factory
  BT::BehaviorTreeFactory factory;

  // Register Retry decorator (Fallback, ReactiveSequence sont déjà dispo)
  factory.registerNodeType<BT::RetryNode>("Retry");

  // Register custom BT nodes
  RCLCPP_INFO(node->get_logger(), "Registering BT nodes...");

  // Actions
  BT::NodeBuilder navigate_builder =
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<turtleshot_bt::NavigateToPoseAction>(name, config, node);
    };
  factory.registerBuilder<turtleshot_bt::NavigateToPoseAction>("NavigateToPose",
                                                               navigate_builder);

  BT::NodeBuilder fire_builder =
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<turtleshot_bt::FireCatapultAction>(name, config, node);
    };
  factory.registerBuilder<turtleshot_bt::FireCatapultAction>("FireCatapult",
                                                             fire_builder);

  // Conditions
  BT::NodeBuilder has_target_builder =
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<turtleshot_bt::HasTargetCondition>(name, config, node);
    };
  factory.registerBuilder<turtleshot_bt::HasTargetCondition>("HasTarget",
                                                             has_target_builder);

  BT::NodeBuilder target_range_builder =
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<turtleshot_bt::TargetInRangeCondition>(name, config, node);
    };
  factory.registerBuilder<turtleshot_bt::TargetInRangeCondition>("TargetInRange",
                                                                 target_range_builder);

  RCLCPP_INFO(node->get_logger(), "✓ Registered %zu BT node types",
              factory.manifests().size());

  // Load BehaviorTree XML file
  std::string pkg_share_dir;
  try {
    pkg_share_dir = ament_index_cpp::get_package_share_directory("turtleshot_bt");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to find package share directory: %s", e.what());
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  std::string xml_file = pkg_share_dir + "/trees/turtleshot_mission.xml";
  RCLCPP_INFO(node->get_logger(), "Loading BT from: %s", xml_file.c_str());

  // Create the behavior tree
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_file);
    RCLCPP_INFO(node->get_logger(), "✓ Behavior tree loaded successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to load behavior tree: %s", e.what());
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  // Setup Groot2 publisher for live monitoring
  BT::Groot2Publisher publisher_zmq(tree);
  RCLCPP_INFO(node->get_logger(),
              "✓ Groot2 Publisher started (ports: 1666/1667)");
  RCLCPP_INFO(node->get_logger(),
              "  → In Groot2: Monitor → Connect");
  RCLCPP_INFO(node->get_logger(),
              "     Publisher port: 1666");
  RCLCPP_INFO(node->get_logger(),
              "     Server port: 1667");

  // Attendre que Nav2 soit entièrement démarré (bt_navigator ACTIVE)
  if (!waitForNav2Active(node, 90s)) {
    RCLCPP_ERROR(node->get_logger(),
                 "Nav2 was not ready, aborting BT execution");
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  // Main BT execution loop
  RCLCPP_INFO(node->get_logger(), "Starting behavior tree execution...");
  RCLCPP_INFO(node->get_logger(),
              "========================================\n");

  rclcpp::Rate rate(10);  // 10 Hz tick rate
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  BT::NodeStatus last_status = BT::NodeStatus::IDLE;

  while (rclcpp::ok()) {
    // Tick the tree une fois
    status = tree.tickOnce();

    // Log à chaque changement d’état
    if (status != last_status) {
      RCLCPP_INFO(node->get_logger(),
                  "BT Status changed: %s",
                  BT::toStr(status, true).c_str());
      last_status = status;
    }

    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(),
              "\n========================================");
  RCLCPP_WARN(node->get_logger(),
              "Mission interrupted (node shutdown)");

  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}
