# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

TurtleShot is a ROS2 robotics workspace featuring a TurtleBot3 robot with a custom catapult attachment ("catapaf"). The system includes Gazebo simulation, sensor drivers (OAK-D camera, LiDAR), autonomous navigation, and AI vision capabilities.

## Build System

This is a ROS2 workspace using the standard colcon build system. All packages are Python-based using `ament_python`.

### Build Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Build with symbolic links (faster iteration for Python)
colcon build --symlink-install

# Clean build
rm -rf build/ install/ log/
colcon build
```

### Source the workspace after building
```bash
source install/setup.bash
```

## Testing

Packages use pytest for testing and include standard ROS2 linting tools.

```bash
# Run tests for all packages
colcon test

# Run tests for specific package
colcon test --packages-select <package_name>

# Run linting (flake8, pep257, copyright checks)
colcon test --packages-select <package_name> --ctest-args -R "flake8|pep257|copyright"
```

## Key Packages

### robot_gazebo_sim
Main Gazebo simulation package for TurtleBot3 with the catapaf attachment.

- **Launch**: `ros2 launch robot_gazebo_sim gazebo_wsl.launch.py`
- **URDFs**: `urdf/turtlebot_with_catapaf.urdf`, `urdf/turtlebot3_burger.urdf`
- **World**: `worlds/turtlebot3_world.world`
- **Nodes**: `random_walker` - autonomous obstacle avoidance behavior
- **WSL-optimized**: Launch file includes hardware acceleration settings for WSL2 environments

The launch file starts:
1. Gazebo server and client
2. robot_state_publisher (publishes TF transforms from URDF)
3. Spawns robot at specified pose
4. random_walker node (autonomous navigation)
5. Foxglove bridge (web visualization on port 8765)

### catapaf_description
URDF/XACRO descriptions for the catapult mechanism.

- **URDFs**: `urdf/catapaf.urdf.xacro`, `urdf/turtlebot_with_catapaf.urdf`
- **Launch**: `ros2 launch catapaf_description display.launch.py` (RViz2 visualization)

### catapaf_gazebo
Modern Gazebo (Ignition/Gz) simulation variant.

- **Launch**: `ros2 launch catapaf_gazebo simulation.launch.py`
- **Uses**: ros_gz_sim, ros_gz_bridge for ROS2-Gazebo integration

### fake_lidar
Simulates a 2D LiDAR sensor using ray-casting against predefined obstacles.

- **Nodes**:
  - `fake_lidar` - Publishes LaserScan messages by ray-casting
  - `scene_publisher` - Publishes obstacle markers for visualization
- **Topics**: Publishes to `/scan` (sensor_msgs/LaserScan)
- **Parameters**: scan_rate, range_min/max, angle_min/max, num_readings

### robot_sim
Simple physics simulator for testing without Gazebo.

- **Node**: `simple_robot_simulator` - Combines odometry simulation with random walking behavior
- **Functionality**:
  - Subscribes to `/cmd_vel`, simulates robot motion
  - Publishes `/odom` and TF (odom → base_footprint)
  - Built-in random walker with obstacle avoidance
- **Launch**: `ros2 launch robot_sim simulation.launch.py` or `robot_with_camera.launch.py`

### oakd_camera_driver
Driver for OAK-D camera hardware.

- **Nodes**:
  - `imu_tf_publisher` - Publishes TF from IMU data (/oak/imu/data)
  - `camera_publisher` - Camera image publishing
- **Launch**: `ros2 launch oakd_camera_driver camera.launch.py`

### video_to_ai
AI vision inference pipeline.

- **Node**: `video_inference_node` - Processes camera images for AI inference

### py_talker / py_listener
Basic ROS2 example nodes for pub/sub communication.

## Architecture Patterns

### Dual Simulation Strategy
The workspace supports two simulation approaches:

1. **Gazebo Classic (robot_gazebo_sim)**: Full physics simulation with meshes, sensors, and random walker
2. **Simple Simulator (robot_sim)**: Lightweight kinematic simulation for rapid testing

Both publish compatible odometry and TF, allowing interchangeable use.

### Sensor Integration
- **Real Hardware**: OAK-D camera provides visual and IMU data
- **Simulated Sensors**: fake_lidar provides LaserScan data for obstacle detection
- **TF Tree**: Consistent frame structure (odom → base_footprint → base_link → sensors)

### Random Walker Behavior
Two implementations share the same obstacle avoidance logic:
- `robot_gazebo_sim/random_walker.py` - For Gazebo simulation
- `robot_sim/simple_robot_simulator.py` - Built into simple simulator

Both use a state machine (FORWARD/TURN) that reacts to LaserScan data within a 60° frontal cone.

### Mesh Organization
The robot_gazebo_sim package includes organized mesh directories:
- `meshes/bases/` - TurtleBot base STL
- `meshes/wheels/` - Wheel STLs
- `meshes/sensors/` - LiDAR, cameras (STL and DAE formats)
- `meshes/catapaf/` - Custom catapult mechanism

## Running the System

### Full Gazebo Simulation
```bash
source install/setup.bash
ros2 launch robot_gazebo_sim gazebo_wsl.launch.py
```

Arguments: `namespace:=<ns>`, `x_pose:=<x>`, `y_pose:=<y>`, `gui:=true/false`

### Lightweight Simulation
```bash
ros2 launch robot_sim simulation.launch.py
```

### Visualization
- **Gazebo GUI**: Included in gazebo_wsl.launch.py
- **Foxglove Studio**: Connect to ws://localhost:8765
- **RViz2**: `ros2 launch catapaf_description display.launch.py`

### Manual Control
```bash
# Override random walker
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

## Important Notes

- **WSL2 Users**: The gazebo_wsl.launch.py includes optimized settings for hardware acceleration
- **Simulation Time**: Most nodes use `use_sim_time:=true` parameter for synchronized simulation
- **QoS Profiles**: LaserScan subscriptions use `qos_profile_sensor_data` (Best Effort) to match Gazebo publishers
- **Coordinate Frames**:
  - World frame: `odom`
  - Robot base: `base_footprint` → `base_link`
  - Sensors: `base_scan` (LiDAR), camera frames
