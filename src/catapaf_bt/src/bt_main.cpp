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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

                    // Increase safe distance to 1.0m to avoid potential costmap conflicts
                    if (dist > 1.0) {
                        // Move target towards robot by 1.0m
                        double move_ratio = (dist - 1.0) / dist;
                        pose_in_map.pose.position.x = robot_x + dx * move_ratio;
                        pose_in_map.pose.position.y = robot_y + dy * move_ratio;
                    } else {
                        // Already within 1.0m, stay put (use current robot pose)
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
// Simple GoToPose utilizing direct cmd_vel (Visual Servoing / P-Controller)
// This replaces Nav2 to ensure forward motion and simple behavior.
// ----------------------------------------------------------------------------
class GoToPose : public StatefulActionNode
{
public:
    GoToPose(const std::string& name, const NodeConfig& config, rclcpp::Node::SharedPtr node)
      : StatefulActionNode(name, config), node_(node)
    {
        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Safety scan subscription
        scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&GoToPose::scan_callback, this, std::placeholders::_1));
    }

    static PortsList providedPorts()
    {
        return { InputPort<geometry_msgs::msg::PoseStamped>("target") };
    }

    NodeStatus onStart() override
    {
        if (!getInput("target", target_pose_)) {
            RCLCPP_ERROR(node_->get_logger(), "GoToPose missing target input");
            return NodeStatus::FAILURE;
        }
        
        RCLCPP_INFO(node_->get_logger(), "GoToPose started. Target: (%.2f, %.2f)", 
            target_pose_.pose.position.x, target_pose_.pose.position.y);
            
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        // 1. Get current robot pose
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "Waiting for TF map->base_link: %s", ex.what());
            return NodeStatus::RUNNING;
        }

        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        
        // Get Robot Yaw
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, robot_yaw;
        m.getRPY(roll, pitch, robot_yaw);

        // 2. Calculate error
        double dx = target_pose_.pose.position.x - robot_x;
        double dy = target_pose_.pose.position.y - robot_y;
        double dist = std::sqrt(dx*dx + dy*dy);
        double target_yaw = std::atan2(dy, dx);
        double angle_diff = target_yaw - robot_yaw;

        // Normalize angle to [-PI, PI]
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

        // 3. Check for completion (0.1m tolerance)
        if (dist < 0.1) {
            stopRobot();
            RCLCPP_INFO(node_->get_logger(), "Target reached!");
            return NodeStatus::SUCCESS;
        }

        // 4. Safety Check (Frontal collision)
        if (isPathBlocked()) {
            stopRobot();
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Path blocked by obstacle! Waiting...");
            return NodeStatus::RUNNING; // Keep trying, or return FAILURE?
        }

        // 5. Compute Control (Simple P-Controller)
        geometry_msgs::msg::Twist cmd;
        
        // Turn first if angle is large (> 30 deg)
        if (std::abs(angle_diff) > 0.5) {
            cmd.linear.x = 0.0;
            cmd.angular.z = (angle_diff > 0 ? 1.0 : -1.0) * 0.5; // Rotate 0.5 rad/s
        } else {
            // Drive and correct angle
            cmd.linear.x = std::min(0.3, dist); // Max 0.3 m/s, slow down near target
            cmd.angular.z = angle_diff * 1.5;   // P-gain for angle
        }

        vel_pub_->publish(cmd);
        return NodeStatus::RUNNING;
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
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }
    
    bool isPathBlocked() {
        if (!latest_scan_) return false;
        
        // Check front cone (-20 to +20 degrees)
        // Adjust indices based on angle_min/increment
        // Simplified: iterate ranges
        // Assuming Standard scan (-PI to PI or similar)
        
        // Find indices for -20 deg to +20 deg
        // If 0 is front.
        
        float min_dist = 100.0;
        
        // Simple iteration over center of array (assuming 0 is index 0 or center?)
        // Safer: Convert angles.
        float angle_min = latest_scan_->angle_min;
        float angle_inc = latest_scan_->angle_increment;
        
        for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
            float angle = angle_min + i * angle_inc;
            // Normalize angle
            while (angle > M_PI) angle -= 2*M_PI;
            while (angle < -M_PI) angle += 2*M_PI;
            
            if (std::abs(angle) < 0.35) { // ~20 degrees
                float r = latest_scan_->ranges[i];
                if (r > latest_scan_->range_min && r < 0.4) { // Blocked if < 40cm
                    return true;
                }
            }
        }
        return false;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    geometry_msgs::msg::PoseStamped target_pose_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
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
            
            RCLCPP_INFO(node_->get_logger(), "StepBack started. Target distance: %.2f m", target_distance_);
            return NodeStatus::RUNNING;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(node_->get_logger(), "Waiting for TF map->base_link: %s", ex.what());
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
                RCLCPP_INFO(node_->get_logger(), "StepBack completed. Distance: %.2f m", distance_traveled);
                return NodeStatus::SUCCESS;
            }
            
            // Move backward
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = -0.2;  // Move backward at 0.2 m/s
            cmd.angular.z = 0.0;
            vel_pub_->publish(cmd);
            
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
        [node](const std::string& name, const NodeConfig& config) {
            return std::make_unique<GoToPose>(name, config, node); 
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
