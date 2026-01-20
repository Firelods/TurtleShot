# Catapaf Behavior Trees

This package implements the behavior tree logic for autonomous robot control using the BehaviorTree.CPP library.

## Overview

The `catapaf_bt` package provides a C++ behavior tree executor that orchestrates high-level robot behaviors including:

- Object detection (trash, balls)
- Autonomous navigation
- Catapult arm control
- Patrol and exploration behaviors

## Dependencies

- `rclcpp` - ROS 2 C++ client library
- `behaviortree_cpp` - BehaviorTree.CPP library
- `catapaf_interfaces` - Custom service definitions
- `std_srvs` - Standard ROS 2 services
- `geometry_msgs` / `sensor_msgs` - ROS 2 message types
- `tf2` / `tf2_ros` - Transform library

## Building

```bash
colcon build --packages-select catapaf_bt
source install/setup.bash
```

## Usage

### Run the Behavior Tree Executor

```bash
ros2 run catapaf_bt bt_executor --ros-args -p use_sim_time:=true
```

### Monitoring with Groot2

The behavior tree publishes to Groot2 for visualization and debugging. Connect Groot2 to monitor the tree execution in real-time.

## Behavior Tree Actions

The package implements several custom BT nodes:

- **DetectObject** - Calls the `get_detection` service to detect objects (trash, balls)
- **GoToPose** - Navigates to a specified pose using Nav2 or direct `cmd_vel` control
- **TurnInPlace** - Rotates the robot to face a target direction
- **StepBack** - Moves the robot backward
- **LaunchCatapult** - Triggers the catapult arm to launch

## Services Used

- `/get_detection` (`catapaf_interfaces/srv/GetDetection`) - Object detection service
- `/get_random_goal` (`catapaf_interfaces/srv/GetRandomGoal`) - Random navigation goal generation
- `/catapaf_arm/launch` (`std_srvs/srv/Trigger`) - Trigger catapult launch
- `/catapaf_arm/reset` (`std_srvs/srv/Trigger`) - Reset arm position

## Topics

### Subscribed
- `/scan` - LiDAR data for obstacle detection

### Published
- `/cmd_vel` - Velocity commands for robot movement

## Architecture

The behavior tree uses a hierarchical structure:
1. **Root Sequence** - Main execution flow
2. **Patrol Behavior** - Random walking and exploration
3. **Detection Branch** - Object detection with throttling
4. **Interaction Branch** - Approach, pick up, and launch behaviors

## License

Apache License 2.0
