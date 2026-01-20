    #include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <atomic>

#include "catapaf_interfaces/srv/get_detection.hpp"
#include "catapaf_interfaces/srv/get_random_goal.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace BT;

// ----------------------------------------------------------------------------
// Generic Service Client Node for GetDetection
// ----------------------------------------------------------------------------
class DetectObject : public StatefulActionNode
{
public:
    DetectObject(const std::string& name, const NodeConfig& config, 
                 rclcpp::Node::SharedPtr node, std::string label,
                 std::shared_ptr<tf2_ros::Buffer> tf_buffer)
      : StatefulActionNode(name, config), node_(node), label_(label), tf_buffer_(tf_buffer),
        last_attempt_time_(std::chrono::steady_clock::time_point::min())
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
        // Check throttle - minimum 2 seconds between attempts
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_attempt_time_).count();
        
        if (elapsed < 2000) {  // Less than 2 seconds since last attempt
            // Set flag to indicate we're in throttle wait mode
            waiting_for_throttle_ = true;
            throttle_start_time_ = now;
            setOutput("is_detected", false);
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
                "[%s] Throttling detection (waiting %.1fs)", label_.c_str(), (2000 - elapsed) / 1000.0);
            return NodeStatus::RUNNING;
        }
        
        waiting_for_throttle_ = false;
        last_attempt_time_ = now;
        
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
        
        RCLCPP_INFO(node_->get_logger(), "[%s] Detection request sent", label_.c_str());
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        // If we're waiting for throttle period to expire
        if (waiting_for_throttle_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - throttle_start_time_).count();
            
            if (elapsed < 2000) {
                // Still waiting, return RUNNING (this gives RandomWalk time to execute)
                return NodeStatus::RUNNING;
            } else {
                // Throttle period expired, restart the node to make a new attempt
                // We do this by halting and letting the tree restart us
                waiting_for_throttle_ = false;
                // Don't return FAILURE - that would make RetryUntilSuccessful retry immediately
                // Instead, we need to signal that we should restart
                // Actually, let's just proceed to make the detection call now
                last_attempt_time_ = now;
                
                if (!client_->wait_for_service(std::chrono::seconds(1))) {
                    RCLCPP_WARN(node_->get_logger(), "GetDetection service not available");
                    setOutput("is_detected", false);
                    return NodeStatus::FAILURE;
                }

                auto request = std::make_shared<catapaf_interfaces::srv::GetDetection::Request>();
                request->label = label_;

                future_ = client_->async_send_request(request).share();
                start_time_ = std::chrono::steady_clock::now();
                
                RCLCPP_INFO(node_->get_logger(), "[%s] Detection request sent (after throttle)", label_.c_str());
                // Continue to normal detection flow below
            }
        }
        
        // Check for timeout (3 seconds)
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        if (elapsed > std::chrono::seconds(3)) {
            RCLCPP_WARN(node_->get_logger(), "Detection service timeout for label: %s", label_.c_str());
            setOutput("is_detected", false);
            return NodeStatus::FAILURE;
        }

        if (future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            auto result = future_.get(); // result is SharedPtr to Response
            if (result->is_detected) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Detection SUCCESS raw: x=%.2f, y=%.2f, z=%.2f frame=%s", 
                            label_.c_str(), 
                            result->pose.pose.position.x, 
                            result->pose.pose.position.y, 
                            result->pose.pose.position.z,
                            result->pose.header.frame_id.c_str());
                
                // Transform to map frame
                geometry_msgs::msg::PoseStamped pose_in_map;
                try {
                    // Force use of latest available transform
                    result->pose.header.stamp = rclcpp::Time(0);

                    // Check if transform is available
                    if (!tf_buffer_->canTransform("map", result->pose.header.frame_id, tf2::TimePointZero, std::chrono::seconds(1))) {
                         RCLCPP_ERROR(node_->get_logger(), "Transform to map not available");
                         setOutput("is_detected", false);
                         return NodeStatus::FAILURE;
                    }
                    
                    tf_buffer_->transform(result->pose, pose_in_map, "map");

                    RCLCPP_INFO(node_->get_logger(), "[%s] Transformed to map: x=%.2f, y=%.2f, z=%.2f",
                        label_.c_str(),
                        pose_in_map.pose.position.x,
                        pose_in_map.pose.position.y,
                        pose_in_map.pose.position.z);
                    
                    // Now calculate the goal point: 50cm short of the target
                    // We need the robot's current position to know the vector
                    geometry_msgs::msg::TransformStamped transform = 
                        tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                    
                    double robot_x = transform.transform.translation.x;
                    double robot_y = transform.transform.translation.y;
                    
                    // Get Yaw from quaternion
                    tf2::Quaternion q_robot(
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w);
                    tf2::Matrix3x3 m(q_robot);
                    double roll, pitch, robot_yaw;
                    m.getRPY(roll, pitch, robot_yaw);

                    RCLCPP_INFO(node_->get_logger(), "Robot Pose in Map: x=%.2f, y=%.2f, yaw=%.2f", robot_x, robot_y, robot_yaw);
                    
                    double target_x = pose_in_map.pose.position.x;
                    double target_y = pose_in_map.pose.position.y;
                    
                    double dx = target_x - robot_x;
                    double dy = target_y - robot_y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    RCLCPP_INFO(node_->get_logger(), "Vector to Object: dx=%.2f, dy=%.2f, dist=%.2f", dx, dy, dist);

                    // Safe distance: how far the robot center should stay from the human
                    // Account for: safe_distance + robot_radius + position_tolerance
                    // TurtleBot radius ~0.18m, position tolerance 0.15m, desired clearance 0.5m
                    // Total: 0.5 + 0.18 + 0.15 = 0.83m, round up to 1.0m for safety
                    const double safe_distance = 1.5;

                    if (dist > safe_distance) {
                        // Calculate goal position: safe_distance away from human, along the robot->human vector
                        // Start from human position, move back toward robot by safe_distance
                        double unit_dx = dx / dist;  // Unit vector from robot to human
                        double unit_dy = dy / dist;

                        pose_in_map.pose.position.x = target_x - unit_dx * safe_distance;
                        pose_in_map.pose.position.y = target_y - unit_dy * safe_distance;
                    } else {
                        // Already within safe distance, stay put (use current robot pose)
                        pose_in_map.pose.position.x = robot_x;
                        pose_in_map.pose.position.y = robot_y;
                        RCLCPP_WARN(node_->get_logger(), "Object too close (%.2fm), staying put.", dist);
                    }
                    
                    RCLCPP_INFO(node_->get_logger(), "Final Goal: x=%.2f, y=%.2f", pose_in_map.pose.position.x, pose_in_map.pose.position.y);

                    pose_in_map.pose.position.z = 0.0;
                    
                    // Orientation: Face the object
                    // Calculate yaw from robot to ORIGINAL target
                    // Wait, we want to face the object (target_x, target_y)
                    double yaw = std::atan2(target_y - robot_y, target_x - robot_x);
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    pose_in_map.pose.orientation = tf2::toMsg(q);

                    setOutput("output_pose", pose_in_map);
                    setOutput("is_detected", true);
                    return NodeStatus::SUCCESS;

                } catch (const tf2::TransformException & ex) {
                    RCLCPP_ERROR(node_->get_logger(), "TF Exception: %s", ex.what());
                    setOutput("is_detected", false);
                    return NodeStatus::FAILURE;
                }

            } else {
                // Throttle failure logs to avoid spamming (every 2 seconds)
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
                                     "[%s] Detection FAILED: Object not found by service.", label_.c_str());
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
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::chrono::steady_clock::time_point last_attempt_time_;  // For throttling
    bool waiting_for_throttle_ = false;  // Flag for throttle wait state
    std::chrono::steady_clock::time_point throttle_start_time_;  // When throttle wait started
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
// GoToPose using Nav2 NavigateToPose action for accurate navigation
// ----------------------------------------------------------------------------
class GoToPose : public StatefulActionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoToPose(const std::string& name, const NodeConfig& config,
             rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
      : StatefulActionNode(name, config), node_(node), tf_buffer_(tf_buffer),
        goal_done_(false), goal_result_available_(false), goal_succeeded_(false)
    {
        // Create the action client
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
            RCLCPP_ERROR(node_->get_logger(), "GoToPose: Missing target input");
            return NodeStatus::FAILURE;
        }

        // Wait for action server
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "GoToPose: NavigateToPose action server not available");
            return NodeStatus::FAILURE;
        }

        // Prepare the goal
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target;
        goal_msg.pose.header.stamp = node_->now();

        // Ensure frame_id is set
        if (goal_msg.pose.header.frame_id.empty()) {
            goal_msg.pose.header.frame_id = "map";
        }

        RCLCPP_INFO(node_->get_logger(), "=== GoToPose (Nav2) STARTED: target at (%.2f, %.2f) in frame %s ===",
                    goal_msg.pose.pose.position.x,
                    goal_msg.pose.pose.position.y,
                    goal_msg.pose.header.frame_id.c_str());

        // Reset state
        goal_done_ = false;
        goal_result_available_ = false;
        goal_succeeded_ = false;

        // Send goal options
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(node_->get_logger(), "GoToPose: Goal was rejected by server");
                    goal_done_ = true;
                    goal_succeeded_ = false;
                } else {
                    RCLCPP_INFO(node_->get_logger(), "GoToPose: Goal accepted by server");
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleNavigateToPose::SharedPtr,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
                double dist = feedback->distance_remaining;
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "GoToPose: Distance remaining: %.2f m", dist);
            };

        send_goal_options.result_callback =
            [this](const GoalHandleNavigateToPose::WrappedResult & result) {
                goal_result_available_ = true;
                goal_done_ = true;
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(node_->get_logger(), "GoToPose: Goal reached successfully!");
                        goal_succeeded_ = true;
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(node_->get_logger(), "GoToPose: Goal was aborted");
                        goal_succeeded_ = false;
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(node_->get_logger(), "GoToPose: Goal was canceled");
                        goal_succeeded_ = false;
                        break;
                    default:
                        RCLCPP_ERROR(node_->get_logger(), "GoToPose: Unknown result code");
                        goal_succeeded_ = false;
                        break;
                }
            };

        // Send the goal
        goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
        start_time_ = std::chrono::steady_clock::now();

        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        // Timeout check (180 seconds / 3 minutes)
        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        if (elapsed > std::chrono::seconds(180)) {
            RCLCPP_ERROR(node_->get_logger(), "GoToPose: Timeout after 180 seconds, canceling goal");
            cancelGoal();
            return NodeStatus::FAILURE;
        }

        // Check if the goal is done
        if (goal_done_) {
            if (goal_succeeded_) {
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        }

        return NodeStatus::RUNNING;
    }

    void onHalted() override {
        RCLCPP_WARN(node_->get_logger(), "GoToPose: Halted, canceling navigation goal");
        cancelGoal();
    }

private:
    void cancelGoal() {
        if (action_client_ && !goal_done_) {
            // Try to cancel the goal
            try {
                auto future = action_client_->async_cancel_all_goals();
                // Don't wait for result - just fire and forget
                RCLCPP_INFO(node_->get_logger(), "GoToPose: Cancel request sent");
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(), "GoToPose: Failed to cancel goal: %s", e.what());
            }
        }
        goal_done_ = true;
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    std::shared_future<GoalHandleNavigateToPose::SharedPtr> goal_handle_future_;

    std::chrono::steady_clock::time_point start_time_;
    std::atomic<bool> goal_done_;
    std::atomic<bool> goal_result_available_;
    std::atomic<bool> goal_succeeded_;
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
// Stop Robot Action
// ----------------------------------------------------------------------------
class StopRobot : public SyncActionNode
{
public:
    StopRobot(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
      : SyncActionNode(name, config), node_(node)
    {
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    static PortsList providedPorts() { return {}; }

    NodeStatus tick() override
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        vel_pub_->publish(msg);
        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};


// ----------------------------------------------------------------------------
// Step Back Action - Moves robot backward by specified distance
// ----------------------------------------------------------------------------
class StepBack : public StatefulActionNode
{
public:
    StepBack(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
      : StatefulActionNode(name, config), node_(node)
    {
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    static PortsList providedPorts()
    {
        return { InputPort<double>("distance", 0.5, "Distance to step back in meters") };
    }

    NodeStatus onStart() override
    {
        double distance = 0.5;
        getInput("distance", distance);
        target_distance_ = distance;
        
        // Get initial position
        try {
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            start_x_ = transform.transform.translation.x;
            start_y_ = transform.transform.translation.y;
            
            RCLCPP_INFO(node_->get_logger(), "=== StepBack STARTED: Target distance %.2f m from (%.2f, %.2f) ===", 
                       target_distance_, start_x_, start_y_);
            return NodeStatus::RUNNING;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "StepBack: Waiting for TF map->base_link: %s", ex.what());
            return NodeStatus::RUNNING;
        }
    }

    NodeStatus onRunning() override
    {
        // Get current position
        try {
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            double current_x = transform.transform.translation.x;
            double current_y = transform.transform.translation.y;
            
            // Calculate distance traveled
            double dx = current_x - start_x_;
            double dy = current_y - start_y_;
            double distance_traveled = std::sqrt(dx*dx + dy*dy);
            
            // Check if we've traveled the target distance
            if (distance_traveled >= target_distance_) {
                stopRobot();
                RCLCPP_INFO(node_->get_logger(), "=== StepBack COMPLETED: Distance %.2f m ===", distance_traveled);
                return NodeStatus::SUCCESS;
            }
            
            // Move backward
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = -0.2;  // Move backward at 0.2 m/s
            cmd.angular.z = 0.0;
            vel_pub_->publish(cmd);
            
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "StepBack: %.2f/%.2f m", distance_traveled, target_distance_);
            
            return NodeStatus::RUNNING;
            
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "TF Exception during StepBack: %s", ex.what());
            return NodeStatus::RUNNING;
        }
    }

    void onHalted() override {
        stopRobot();
    }

private:
    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        vel_pub_->publish(cmd);
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    double target_distance_;
    double start_x_;
    double start_y_;
};


// ----------------------------------------------------------------------------
// Turn In Place Action - Rotates robot by specified angle
// ----------------------------------------------------------------------------
class TurnInPlace : public StatefulActionNode
{
public:
    TurnInPlace(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
      : StatefulActionNode(name, config), node_(node)
    {
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    static PortsList providedPorts()
    {
        return { InputPort<double>("angle", 3.14159, "Angle to turn in radians") };
    }

    NodeStatus onStart() override
    {
        double angle = 3.14159;
        getInput("angle", angle);
        target_angle_ = angle;

        // Get initial orientation
        try {
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            start_yaw_ = yaw;
            angle_rotated_ = 0.0;
            last_yaw_ = yaw;

            RCLCPP_INFO(node_->get_logger(), "=== TurnInPlace STARTED: Target angle %.2f rad (%.1f deg) from yaw %.2f ===",
                       target_angle_, target_angle_ * 180.0 / M_PI, start_yaw_ * 180.0 / M_PI);
            return NodeStatus::RUNNING;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "Waiting for TF map->base_link: %s", ex.what());
            return NodeStatus::RUNNING;
        }
    }

    NodeStatus onRunning() override
    {
        // Get current orientation
        try {
            auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, current_yaw;
            m.getRPY(roll, pitch, current_yaw);

            // Calculate the change in yaw since last iteration
            double delta_yaw = current_yaw - last_yaw_;
            
            // Normalize delta to [-pi, pi] to handle wrap-around
            while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
            while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;
            
            // Accumulate the total rotation
            angle_rotated_ += delta_yaw;
            last_yaw_ = current_yaw;

            // Calculate remaining angle to rotate
            double remaining_angle = target_angle_ - angle_rotated_;

            // Check if we've reached the target orientation
            if (std::abs(remaining_angle) < ANGLE_TOLERANCE) {
                stopRobot();
                RCLCPP_INFO(node_->get_logger(), "=== TurnInPlace COMPLETED: Rotated %.3f rad (%.1f deg), Target: %.3f rad ===",
                           angle_rotated_, angle_rotated_ * 180.0 / M_PI, target_angle_);
                return NodeStatus::SUCCESS;
            }

            // Calculate angular velocity with proportional control
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;

            // Proportional control with limits
            double angular_vel = K_TURN * remaining_angle;
            angular_vel = std::clamp(angular_vel, -MAX_TURN_SPEED, MAX_TURN_SPEED);

            // Ensure minimum velocity to overcome friction
            if (std::abs(angular_vel) < MIN_TURN_SPEED) {
                angular_vel = (angular_vel > 0) ? MIN_TURN_SPEED : -MIN_TURN_SPEED;
            }

            cmd.angular.z = angular_vel;
            vel_pub_->publish(cmd);

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                "Turning: rotated=%.2f/%.2f rad, vel=%.2f rad/s", 
                                angle_rotated_, target_angle_, angular_vel);

            return NodeStatus::RUNNING;

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "TF Exception during TurnInPlace: %s", ex.what());
            return NodeStatus::RUNNING;
        }
    }

    void onHalted() override {
        stopRobot();
    }

private:
    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        vel_pub_->publish(cmd);
    }

    static constexpr double K_TURN = 1.5;             // Turn gain
    static constexpr double MAX_TURN_SPEED = 0.6;     // Max turn speed (rad/s)
    static constexpr double MIN_TURN_SPEED = 0.1;     // Min turn speed to overcome friction
    static constexpr double ANGLE_TOLERANCE = 0.05;   // Angle tolerance (rad) ~2.86 degrees

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double target_angle_;
    double start_yaw_;
    double last_yaw_;
    double angle_rotated_;
};


// ----------------------------------------------------------------------------
// Get Trash Pose - Returns hardcoded trash position from World
// ----------------------------------------------------------------------------
class GetTrashPose : public StatefulActionNode
{
public:
    GetTrashPose(const std::string& name, const NodeConfig& config, 
                 rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
      : StatefulActionNode(name, config), node_(node), tf_buffer_(tf_buffer)
    {}

    static PortsList providedPorts()
    {
        return { OutputPort<geometry_msgs::msg::PoseStamped>("output_pose") };
    }

    NodeStatus onStart() override { return NodeStatus::RUNNING; }

    NodeStatus onRunning() override
    {
        RCLCPP_INFO(node_->get_logger(), "=== GetTrashPose: STARTING ===");
        
        // Get robot's current position to determine best approach
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "GetTrashPose: Waiting for TF map->base_link: %s", ex.what());
            return NodeStatus::RUNNING;
        }

        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;

        // Get robot orientation
        tf2::Quaternion q_robot(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 m(q_robot);
        double roll, pitch, robot_yaw;
        m.getRPY(roll, pitch, robot_yaw);

        // Trash can position (matching world file at -4, -2, 0.25)
        const double trash_x = -4.0;
        const double trash_y = -2.0;

        // Calculate distance to trash
        double dx = trash_x - robot_x;
        double dy = trash_y - robot_y;
        double dist = std::sqrt(dx*dx + dy*dy);

        RCLCPP_INFO(node_->get_logger(), "GetTrashPose: Robot at (%.2f, %.2f, yaw=%.2f), Trash at (%.2f, %.2f), dist=%.2fm",
                    robot_x, robot_y, robot_yaw * 180.0 / M_PI, trash_x, trash_y, dist);

        geometry_msgs::msg::PoseStamped pose_in_map;
        pose_in_map.header.frame_id = "map";
        pose_in_map.header.stamp = node_->now();

        // Calculate approach position: stop 0.6m away from trash to avoid collision
        const double approach_distance = 0.6;  // meters
        double approach_x, approach_y;
        
        if (dist > approach_distance) {
            // Calculate unit vector from robot to trash
            double ux = dx / dist;
            double uy = dy / dist;
            // Position that's approach_distance away from trash
            approach_x = trash_x - ux * approach_distance;
            approach_y = trash_y - uy * approach_distance;
        } else {
            // Already close enough, just go to current position
            approach_x = robot_x;
            approach_y = robot_y;
        }

        pose_in_map.pose.position.x = approach_x;
        pose_in_map.pose.position.y = approach_y;
        pose_in_map.pose.position.z = 0.0;

        // Face toward the trash can from the approach position
        double goal_to_trash_x = trash_x - approach_x;
        double goal_to_trash_y = trash_y - approach_y;
        double yaw = std::atan2(goal_to_trash_y, goal_to_trash_x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose_in_map.pose.orientation = tf2::toMsg(q);

        setOutput("output_pose", pose_in_map);
        RCLCPP_INFO(node_->get_logger(), "=== GetTrashPose: SUCCESS - Goal at (%.2f, %.2f), 0.6m from trash, facing yaw=%.2f rad (%.1f deg) ===",
                    pose_in_map.pose.position.x, pose_in_map.pose.position.y, yaw, yaw * 180.0 / M_PI);

        return NodeStatus::SUCCESS;
    }
    
    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};


// ----------------------------------------------------------------------------
// MAIN
// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("catapaf_bt_executor");

    // Initialize TF buffer and listener
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    BehaviorTreeFactory factory;

    // Register generic nodes
    factory.registerNodeType<WaitAction>("Wait");

    // Register ROS-dependent nodes by lambda or explicit builder
    factory.registerBuilder<DetectObject>("DetectTrashCan", 
        [node, tf_buffer](const std::string& name, const NodeConfig& config) {
            return std::make_unique<DetectObject>(name, config, node, "trashcan", tf_buffer); 
        });

    factory.registerBuilder<DetectObject>("DetectHumanWithBall", 
        [node, tf_buffer](const std::string& name, const NodeConfig& config) {
            // Label matching what the YOLO node returns.
            return std::make_unique<DetectObject>(name, config, node, "human", tf_buffer); 
        });

    factory.registerBuilder<GoToPose>("GoToPose",
        [node, tf_buffer](const std::string& name, const NodeConfig& config) {
            return std::make_unique<GoToPose>(name, config, node, tf_buffer);
        });

    factory.registerBuilder<StopRobot>("StopRobot", 
        [node](const std::string& name, const NodeConfig& config) {
            return std::make_unique<StopRobot>(name, config, node); 
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

    factory.registerBuilder<StepBack>("StepBack",
        [node](const std::string& name, const NodeConfig& config) {
            return std::make_unique<StepBack>(name, config, node);
        });

    factory.registerBuilder<TurnInPlace>("TurnInPlace",
        [node](const std::string& name, const NodeConfig& config) {
            return std::make_unique<TurnInPlace>(name, config, node);
        });

    factory.registerBuilder<GetTrashPose>("GetTrashPose",
        [node, tf_buffer](const std::string& name, const NodeConfig& config) {
            return std::make_unique<GetTrashPose>(name, config, node, tf_buffer);
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
