    #include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <algorithm>
#include <cstdlib>

#include "catapaf_interfaces/srv/get_detection.hpp"
#include "catapaf_interfaces/srv/get_random_goal.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace BT;

// ----------------------------------------------------------------------------
// Generic Service Client Node for GetDetection
// ----------------------------------------------------------------------------
class DetectObject : public StatefulActionNode
{
public:
    DetectObject(const std::string& name, const NodeConfig& config, 
                 rclcpp::Node::SharedPtr node, std::string label)
      : StatefulActionNode(name, config), node_(node), label_(label)
    {
        client_ = node_->create_client<catapaf_interfaces::srv::GetDetection>("get_detection");
    }

    static PortsList providedPorts()
    {
        return { OutputPort<geometry_msgs::msg::PoseStamped>("output_pose"),
                 OutputPort<bool>("is_detected") };
    }

    NodeStatus onStart() override
    {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "GetDetection service not available");
            setOutput("is_detected", false);
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<catapaf_interfaces::srv::GetDetection::Request>();
        request->label = label_;

        // Store the shared future and start time
        future_ = client_->async_send_request(request).share();
        start_time_ = std::chrono::steady_clock::now();
        
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        // Check for timeout (2 seconds)
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        if (elapsed > std::chrono::seconds(2)) {
            RCLCPP_WARN(node_->get_logger(), "Detection service timeout for label: %s", label_.c_str());
            setOutput("is_detected", false);
            return NodeStatus::FAILURE;
        }

        if (future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            auto result = future_.get(); // result is SharedPtr to Response
            if (result->is_detected) {
                setOutput("output_pose", result->pose);
                setOutput("is_detected", true);
                return NodeStatus::SUCCESS;
            } else {
                setOutput("is_detected", false);
                return NodeStatus::FAILURE;
            }
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        // Cancel logic if needed
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<catapaf_interfaces::srv::GetDetection>::SharedPtr client_;
    std::string label_;
    rclcpp::Client<catapaf_interfaces::srv::GetDetection>::SharedFuture future_;
    std::chrono::steady_clock::time_point start_time_;
};


// ----------------------------------------------------------------------------
// Simple Trigger Service Client
// ----------------------------------------------------------------------------
class TriggerArm : public SyncActionNode
{
public:
    TriggerArm(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
      : SyncActionNode(name, config), node_(node)
    {
        client_ = node_->create_client<std_srvs::srv::Trigger>("trigger_arm");
    }

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "TriggerArm service not available");
            return NodeStatus::FAILURE;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = client_->async_send_request(request); 
        
        // Sync call for simplicity in SyncActionNode (blocking tick)
        if (result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto res = result.get();
            if (res->success) return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};


// ----------------------------------------------------------------------------
// Random Walk: Get Goal + Navigate
// ----------------------------------------------------------------------------
class RandomWalk : public StatefulActionNode
{
public:
    RandomWalk(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
      : StatefulActionNode(name, config), node_(node)
    {
        scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&RandomWalk::scan_callback, this, std::placeholders::_1));
            
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Seed random
        std::srand(std::time(nullptr));
    }

    static PortsList providedPorts() { return {}; }

    NodeStatus onStart() override
    {
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (!latest_scan_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Waiting for laser scan...");
            return NodeStatus::RUNNING;
        }

        // Check for obstacles in front (-30 to +30 degrees approx)
        bool obstacle_detected = false;
        
        // Scan indices: 0 is front? Or it depends on the robot. Usually 0 is front.
        // Let's assume standard behavior: 0 is front, positive left, negative right (or 0-2pi).
        // Safest is to check a range.
        // Assuming scan.ranges is 360 size (1 degree res).
        // Front sector: [0, 30] and [330, 360] indices roughly.
        // Better: use angle_min/max/increment.
        
        int fov_indices = 30; // Check +/- 30 indices around 0
        int n_ranges = latest_scan_->ranges.size();
        
        // Logic: if any range in front < 0.5, stop/turn.
        for (int i = 0; i < n_ranges; ++i) {
             if (i < fov_indices || i >= n_ranges - fov_indices) {
                 float r = latest_scan_->ranges[i];
                 if (r < 0.5 && r > latest_scan_->range_min) {
                     obstacle_detected = true;
                     break;
                 }
             }
        }
        
        auto twist = geometry_msgs::msg::Twist();
        
        if (obstacle_detected) {
            // Find direction of max clearance
            float max_range = 0.0;
            int best_index = -1;
            
            for (int i = 0; i < n_ranges; ++i) {
                float r = latest_scan_->ranges[i];
                if (!std::isinf(r) && r > max_range) {
                    max_range = r;
                    best_index = i;
                }
            }
            
            if (best_index != -1) {
                // Calculate angle to best index
                float angle = latest_scan_->angle_min + best_index * latest_scan_->angle_increment;
                
                // turn towards that angle
                // P-controller
                twist.linear.x = 0.0;
                twist.angular.z = std::max(-1.0f, std::min(1.0f, 2.0f * angle));
            } else {
                 // Fallback if no valid range found (all inf or too close?)
                 twist.linear.x = 0.0;
                 twist.angular.z = 0.5;
            }

        } else {
            twist.linear.x = 0.3; // Move forward slightly faster
            
            // Add some noise to wandering
            double noise = ((std::rand() % 100) - 50) / 1000.0; // +/- 0.05
            twist.angular.z = noise; 
        }
        
        vel_pub_->publish(twist);
        
        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        // Stop robot
        auto twist = geometry_msgs::msg::Twist();
        vel_pub_->publish(twist);
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

// ----------------------------------------------------------------------------
// Simple GoToPose utilizing Nav2
// ----------------------------------------------------------------------------
class GoToPose : public StatefulActionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::Client<NavigateToPose>::GoalHandle;

    GoToPose(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
      : StatefulActionNode(name, config), node_(node)
    {
        action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    }

    static PortsList providedPorts()
    {
        return { InputPort<geometry_msgs::msg::PoseStamped>("target") };
    }

    NodeStatus onStart() override
    {
        geometry_msgs::msg::PoseStamped target;
        if (!getInput("target", target)) {
            RCLCPP_ERROR(node_->get_logger(), "GoToPose missing target input");
            return NodeStatus::FAILURE;
        }

        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 Action Server not available");
            return NodeStatus::FAILURE;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target;
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&GoToPose::result_callback, this, std::placeholders::_1);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
        done_ = false;
        
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (done_) {
             return success_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {}

    void result_callback(const GoalHandleNav::WrappedResult & result) {
        done_ = true;
        success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    bool done_ = false;
    bool success_ = false;
};

// ----------------------------------------------------------------------------
// Wait Action
// ----------------------------------------------------------------------------
class WaitAction : public StatefulActionNode
{
public:
    WaitAction(const std::string& name, const NodeConfig& config)
      : StatefulActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return { InputPort<int>("duration") };
    }

    NodeStatus onStart() override
    {
        int duration_sec = 0;
        if (!getInput("duration", duration_sec)) {
            throw RuntimeError("missing duration");
        }
        completion_time_ = std::chrono::system_clock::now() + std::chrono::seconds(duration_sec);
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (std::chrono::system_clock::now() >= completion_time_) {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::RUNNING;
    }

    void onHalted() override {}

private:
    std::chrono::system_clock::time_point completion_time_;
};


// ----------------------------------------------------------------------------
// MAIN
// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("catapaf_bt_executor");

    BehaviorTreeFactory factory;

    // Register generic nodes
    factory.registerNodeType<WaitAction>("Wait");

    // Register ROS-dependent nodes by lambda or explicit builder
    factory.registerBuilder<DetectObject>("DetectTrashCan", 
        [node](const std::string& name, const NodeConfig& config) {
            return std::make_unique<DetectObject>(name, config, node, "trashcan"); 
        });

    factory.registerBuilder<DetectObject>("DetectHumanWithBall", 
        [node](const std::string& name, const NodeConfig& config) {
            // Label matching what the YOLO node returns.
            return std::make_unique<DetectObject>(name, config, node, "person"); 
        });

    factory.registerBuilder<GoToPose>("GoToPose", 
        [node](const std::string& name, const NodeConfig& config) {
            return std::make_unique<GoToPose>(name, config, node); 
        });

    factory.registerBuilder<RandomWalk>("RandomWalk", 
        [node](const std::string& name, const NodeConfig& config) {
            return std::make_unique<RandomWalk>(name, config, node); 
        });
        
    factory.registerBuilder<TriggerArm>("TriggerArm", 
        [node](const std::string& name, const NodeConfig& config) {
            return std::make_unique<TriggerArm>(name, config, node); 
        });

    // Load XML
    // Assuming file is at a known path, or we can use parameter
    // For this demo, let's look in package share or hardcoded path (user provided)
    // The user's file is in src/catapaf_gazebo/behavior_trees/...
    // Ideally we install it. For now let's use the absolute path for dev speed.
    std::string xml_path = "/home/bilbo/TurtleShot/src/catapaf_gazebo/behavior_trees/patrol_and_interact.xml";
    
    // Check if file exists
    // (Omitted check for brevity)

    auto tree = factory.createTreeFromFile(xml_path);
    
    // Groot2 ZMQ Publisher (Updated for BT.CPP v4.3+)
    Groot2Publisher publisher_zmq(tree);

    RCLCPP_INFO(node->get_logger(), "Behavior Tree Started. Connect Groot2 to monitor.");

    // Spin ROS node in a separate thread to handle callbacks (like actions/service responses)
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    rclcpp::Rate rate(10);
    while(rclcpp::ok()) {
        tree.tickOnce();
        // rclcpp::spin_some(node); // Not needed with separate thread
        rate.sleep();
    }

    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    return 0;
}
