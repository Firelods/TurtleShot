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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

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
// Get Patrol Point - Cycles through predefined waypoints
// ----------------------------------------------------------------------------
class GetPatrolPoint : public SyncActionNode
{
public:
    GetPatrolPoint(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
      : SyncActionNode(name, config), node_(node), current_waypoint_index_(0)
    {
        // Load waypoints from parameter (will be set from YAML)
        loadWaypoints();
    }

    static PortsList providedPorts() {
        return { OutputPort<geometry_msgs::msg::PoseStamped>("output_pose") };
    }

    NodeStatus tick() override
    {
        if (waypoints_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "No patrol waypoints defined!");
            return NodeStatus::FAILURE;
        }

        // Get current waypoint
        auto& wp = waypoints_[current_waypoint_index_];
        
        // Create PoseStamped message
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = node_->now();
        pose.pose.position.x = wp.x;
        pose.pose.position.y = wp.y;
        pose.pose.position.z = 0.0;
        
        // Convert theta to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, wp.theta);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        // Set output
        setOutput("output_pose", pose);
        
        RCLCPP_INFO(node_->get_logger(), "Patrol waypoint %zu/%zu: (%.2f, %.2f, %.2f)", 
                    current_waypoint_index_ + 1, waypoints_.size(), wp.x, wp.y, wp.theta);
        
        // Move to next waypoint (cycle)
        current_waypoint_index_ = (current_waypoint_index_ + 1) % waypoints_.size();
        
        return NodeStatus::SUCCESS;
    }

private:
    struct Waypoint {
        double x;
        double y;
        double theta;
    };

    void loadWaypoints() {
        // Try to load from parameter server
        // For now, use hardcoded waypoints that match the YAML file
        // In production, you'd load these from the parameter server
        
        // These should match config/patrol_waypoints.yaml
        waypoints_ = {
            {0.0, 0.0, 0.0},      // Center
            {2.0, 2.0, 1.57},     // North
            {2.0, -2.0, 0.0},     // East
            {-2.0, -2.0, -1.57},  // South
            {-2.0, 2.0, 3.14}     // West
        };
        
        RCLCPP_INFO(node_->get_logger(), "Loaded %zu patrol waypoints", waypoints_.size());
    }

    rclcpp::Node::SharedPtr node_;
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
};


// ----------------------------------------------------------------------------
// Random Walk: Get Goal + Navigate
// ----------------------------------------------------------------------------
class RandomWalk : public StatefulActionNode
{
public:
    // Exploration states
    enum class ExplorationState {
        FORWARD_EXPLORATION,
        WALL_FOLLOWING,
        SPIRAL_PATTERN,
        OBSTACLE_AVOIDANCE
    };

    // Wall following side
    enum class WallSide {
        LEFT,
        RIGHT,
        NONE
    };

    RandomWalk(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
      : StatefulActionNode(name, config), node_(node),
        state_(ExplorationState::FORWARD_EXPLORATION),
        wall_side_(WallSide::LEFT),
        spiral_radius_(0.0),
        state_duration_(0.0),
        last_time_(node->now())
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
        state_ = ExplorationState::FORWARD_EXPLORATION;
        spiral_radius_ = 0.0;
        state_duration_ = 0.0;
        last_time_ = node_->now();
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        if (!latest_scan_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Waiting for laser scan...");
            return NodeStatus::RUNNING;
        }

        // Update time
        auto current_time = node_->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        state_duration_ += dt;

        // Analyze laser scan into sectors
        ScanSectors sectors = analyzeScan();

        // Determine state transitions
        updateState(sectors);

        // Execute behavior based on current state
        auto twist = executeBehavior(sectors, dt);

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
    // Scan sector data
    struct ScanSectors {
        float front;           // Minimum distance in front (±15°)
        float front_left;      // Minimum distance front-left (30-60°)
        float front_right;     // Minimum distance front-right (300-330°)
        float left;            // Minimum distance left side (80-100°)
        float right;           // Minimum distance right side (260-280°)
        float back;            // Minimum distance back (170-190°)
        bool has_left_wall;    // Wall detected on left
        bool has_right_wall;   // Wall detected on right
    };


    ScanSectors analyzeScan() {
        ScanSectors sectors;
        int n_ranges = latest_scan_->ranges.size();
        
        // Helper to get sector minimum
        auto getSectorMin = [&](int start_deg, int end_deg) -> float {
            float min_dist = std::numeric_limits<float>::max();
            int valid_count = 0;
            for (int deg = start_deg; deg <= end_deg; ++deg) {
                int idx = angleToIndex(deg);
                if (idx >= 0 && idx < n_ranges) {
                    float r = latest_scan_->ranges[idx];
                    if (!std::isinf(r) && !std::isnan(r) && r > latest_scan_->range_min && r < latest_scan_->range_max) {
                        if (r < min_dist) {
                            min_dist = r;
                        }
                        valid_count++;
                    }
                }
            }
            // If no valid readings, return a large distance (assume clear)
            return (valid_count > 0) ? min_dist : 10.0f;
        };

        // Analyze sectors (assuming 0° is front, counterclockwise positive)
        sectors.front = getSectorMin(-15, 15);
        sectors.front_left = getSectorMin(30, 60);
        sectors.front_right = getSectorMin(-60, -30);
        sectors.left = getSectorMin(80, 100);
        sectors.right = getSectorMin(-100, -80);
        sectors.back = getSectorMin(170, 190);

        // Detect walls
        sectors.has_left_wall = sectors.left < WALL_DETECT_DISTANCE;
        sectors.has_right_wall = sectors.right < WALL_DETECT_DISTANCE;

        // Debug logging (throttled)
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "Scan: F=%.2f FL=%.2f FR=%.2f L=%.2f R=%.2f | Walls: L=%d R=%d",
            sectors.front, sectors.front_left, sectors.front_right,
            sectors.left, sectors.right,
            sectors.has_left_wall, sectors.has_right_wall);

        return sectors;
    }


    int angleToIndex(int degrees) {
        // Convert degrees to radians
        float angle_rad = degrees * M_PI / 180.0;
        
        // Calculate index from angle
        int idx = static_cast<int>((angle_rad - latest_scan_->angle_min) / latest_scan_->angle_increment);
        
        return idx;
    }

    void updateState(const ScanSectors& sectors) {
        ExplorationState old_state = state_;
        
        // Simplified state machine - prioritize forward movement
        
        // Always prioritize obstacle avoidance
        if (sectors.front < OBSTACLE_DISTANCE) {
            state_ = ExplorationState::OBSTACLE_AVOIDANCE;
            if (old_state != state_) {
                RCLCPP_INFO(node_->get_logger(), "State: OBSTACLE_AVOIDANCE (front=%.2f)", sectors.front);
                state_duration_ = 0.0;
            }
            return;
        }

        // If we were avoiding and now clear, go to forward exploration
        if (state_ == ExplorationState::OBSTACLE_AVOIDANCE) {
            state_ = ExplorationState::FORWARD_EXPLORATION;
            RCLCPP_INFO(node_->get_logger(), "State: FORWARD_EXPLORATION (obstacle cleared)");
            state_duration_ = 0.0;
            return;
        }

        // Default: forward exploration
        if (state_ != ExplorationState::FORWARD_EXPLORATION) {
            state_ = ExplorationState::FORWARD_EXPLORATION;
            RCLCPP_INFO(node_->get_logger(), "State: FORWARD_EXPLORATION (default)");
            state_duration_ = 0.0;
        }
    }

    geometry_msgs::msg::Twist executeBehavior(const ScanSectors& sectors, double dt) {
        auto twist = geometry_msgs::msg::Twist();

        if (state_ == ExplorationState::OBSTACLE_AVOIDANCE) {
            // Find best direction to turn
            float left_clearance = std::min(sectors.front_left, sectors.left);
            float right_clearance = std::min(sectors.front_right, sectors.right);
            
            // Slow down based on distance
            if (sectors.front > 0.3) {
                twist.linear.x = 0.15;  // Slow forward
            } else {
                twist.linear.x = 0.0;   // Stop
            }
            
            // Turn toward more open side
            if (left_clearance > right_clearance) {
                twist.angular.z = TURN_SPEED;  // Turn left
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "Avoiding: Turning LEFT (L=%.2f R=%.2f)", left_clearance, right_clearance);
            } else {
                twist.angular.z = -TURN_SPEED; // Turn right
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "Avoiding: Turning RIGHT (L=%.2f R=%.2f)", left_clearance, right_clearance);
            }
        } else {
            // Forward exploration - just move forward with slight random turns
            twist.linear.x = FORWARD_SPEED;
            
            // Add small random angular noise for exploration
            double noise = ((std::rand() % 100) - 50) / 500.0; // +/- 0.1
            twist.angular.z = noise;
            
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "Exploring: Moving forward (v=%.2f, w=%.2f)", twist.linear.x, twist.angular.z);
        }

        return twist;
    }

    // Constants
    static constexpr float OBSTACLE_DISTANCE = 0.5f;
    static constexpr float WALL_FOLLOW_DISTANCE = 0.4f;
    static constexpr float WALL_DETECT_DISTANCE = 0.8f;
    static constexpr float FORWARD_SPEED = 0.3f;
    static constexpr float TURN_SPEED = 0.8f;
    static constexpr float SPIRAL_INCREMENT = 0.05f;

    // Member variables
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    
    ExplorationState state_;
    WallSide wall_side_;
    double spiral_radius_;
    double state_duration_;
    rclcpp::Time last_time_;
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

    factory.registerBuilder<GetPatrolPoint>("GetPatrolPoint", 
        [node](const std::string& name, const NodeConfig& config) {
            return std::make_unique<GetPatrolPoint>(name, config, node); 
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
