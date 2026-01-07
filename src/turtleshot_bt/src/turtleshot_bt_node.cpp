#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/decorators/retry_node.h>
#include <behaviortree_cpp/controls/fallback_node.h>
#include <behaviortree_cpp/controls/reactive_sequence.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Include custom BT nodes
#include "turtleshot_bt/actions/navigate_to_pose_action.hpp"
#include "turtleshot_bt/actions/fire_catapult_action.hpp"
#include "turtleshot_bt/conditions/has_target_condition.hpp"
#include "turtleshot_bt/conditions/target_in_range_condition.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create ROS2 node
  auto node = std::make_shared<rclcpp::Node>("turtleshot_bt");

  RCLCPP_INFO(node->get_logger(), "=== TurtleShot Behavior Tree Orchestrator ===");

  // Create BehaviorTree factory
  BT::BehaviorTreeFactory factory;

  // Register Retry decorator (others like Fallback, ReactiveSequence are already registered)
  factory.registerNodeType<BT::RetryNode>("Retry");

  // Register custom BT nodes
  RCLCPP_INFO(node->get_logger(), "Registering BT nodes...");

  // Register Actions (using registerBuilder for custom constructors)
  BT::NodeBuilder navigate_builder = [node](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<turtleshot_bt::NavigateToPoseAction>(name, config, node);
  };
  factory.registerBuilder<turtleshot_bt::NavigateToPoseAction>("NavigateToPose", navigate_builder);

  BT::NodeBuilder fire_builder = [node](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<turtleshot_bt::FireCatapultAction>(name, config, node);
  };
  factory.registerBuilder<turtleshot_bt::FireCatapultAction>("FireCatapult", fire_builder);

  // Register Conditions (using registerBuilder for custom constructors)
  BT::NodeBuilder has_target_builder = [node](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<turtleshot_bt::HasTargetCondition>(name, config, node);
  };
  factory.registerBuilder<turtleshot_bt::HasTargetCondition>("HasTarget", has_target_builder);

  BT::NodeBuilder target_range_builder = [node](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<turtleshot_bt::TargetInRangeCondition>(name, config, node);
  };
  factory.registerBuilder<turtleshot_bt::TargetInRangeCondition>("TargetInRange", target_range_builder);

  RCLCPP_INFO(node->get_logger(), "✓ Registered %zu BT node types", factory.manifests().size());

  // Load BehaviorTree XML file
  std::string pkg_share_dir;
  try {
    pkg_share_dir = ament_index_cpp::get_package_share_directory("turtleshot_bt");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to find package share directory: %s", e.what());
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
    RCLCPP_ERROR(node->get_logger(), "Failed to load behavior tree: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // Setup Groot2 publisher for live monitoring
  BT::Groot2Publisher publisher_zmq(tree);
  RCLCPP_INFO(node->get_logger(), "✓ Groot2 Publisher started (ports: 1666/1667)");
  RCLCPP_INFO(node->get_logger(), "  → In Groot2: Monitor → Connect");
  RCLCPP_INFO(node->get_logger(), "     Publisher port: 1666");
  RCLCPP_INFO(node->get_logger(), "     Server port: 1667");

  // Optional: File logger for post-execution analysis
  // BT::FileLogger file_logger(tree, "turtleshot_bt_log.fbl");

  // Main BT execution loop
   // Main BT execution loop
  RCLCPP_INFO(node->get_logger(), "Starting behavior tree execution...");
  RCLCPP_INFO(node->get_logger(), "========================================\n");

  rclcpp::Rate rate(10);  // 10 Hz tick rate
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  BT::NodeStatus last_status = BT::NodeStatus::IDLE;

  while (rclcpp::ok()) {
    // Tick the tree une fois
    status = tree.tickOnce();

    // Process ROS2 callbacks
    rclcpp::spin_some(node);

    // Log à chaque changement d’état
    if (status != last_status) {
      RCLCPP_INFO(
        node->get_logger(),
        "BT Status changed: %s",
        BT::toStr(status, true).c_str());
      last_status = status;
    }

    // 👉 MODE DEBUG : on NE QUITTE PAS, même en SUCCESS/FAILURE
    // Tu peux éventuellement faire une pause si l’arbre a fini :
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      // Option 1 : on continue à ticker, l’arbre décidera quoi faire
      // Option 2 : on ralentit un peu
      // rate = rclcpp::Rate(1);
    }

    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "\n========================================");
  RCLCPP_WARN(node->get_logger(), "Mission interrupted (node shutdown)");

  rclcpp::shutdown();
  return 0;
}

