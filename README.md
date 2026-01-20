# TurtleShot

TurtleShot is a modular robotics software suite for simulation, control, and perception of a mobile robot platform with an articulated arm (catapult). It integrates ROS 2-based navigation, behavior trees, Gazebo Fortress simulation, robot description (URDF/Xacro), and custom hardware drivers. The project is also the main workspace for training and deploying visual segmentation models, in connection with [ball-segmentation](https://github.com/Antoine-MR/ball-segmentation).

## Features

- Modern Gazebo Fortress simulation (Ignition Gazebo 6)
- ROS 2 Humble integration and full ROS-Gazebo bridge
- Differential drive, LiDAR, IMU, RGB/Depth camera simulation
- Articulated arm (catapult) control and physics-based ball launching
- Autonomous exploration and behavior trees
- Modular robot description (URDF/Xacro, meshes)
- Custom hardware and interface drivers
- Visual segmentation and AI inference modules
- Ready for training and deploying segmentation models

## Project Structure

- `src/catapaf_description/` — Robot description (URDF/Xacro, meshes, launch)
- `src/catapaf_gazebo/` — Gazebo Fortress simulation, launch files, models, arm controller, configs
- `src/catapaf_bt/` — Behavior trees and C++ main for autonomous behaviors
- `src/catapaf_interfaces/` — Custom ROS 2 service definitions
- `src/distance_to_pwm/` — Hardware interface for distance-to-PWM conversion
- `src/inference_ia_camera/` — AI camera inference node, model configs
- `src/video_to_ai/` — Video-based AI inference node
- `src/yolo_oak_driver/` — YOLO OAK-D camera driver and helpers

Each subpackage contains its own README for detailed usage and API.

## Getting Started

### Prerequisites

- Ubuntu 22.04 (WSL2 recommended)
- ROS 2 Humble
- Gazebo Fortress 6.16.0
- Python 3.10+
- colcon, and required ROS/ROS-Gazebo packages
- CycloneDDS (recommended for multiple participants)

### Build

```bash
cd ~/TurtleShot
source /opt/ros/humble/setup.bash
colcon build --packages-select catapaf_interfaces catapaf_description catapaf_gazebo catapaf_bt distance_to_pwm video_to_ai
source install/setup.bash
```

## Launching the Full System

The recommended way to launch all components is using the provided script:

```bash
./launch_all.sh
```

This script handles:
1. Killing any existing ROS 2/Gazebo processes
2. Building the required packages
3. Configuring CycloneDDS for handling multiple participants
4. Launching Gazebo simulation
5. Starting Nav2 navigation
6. Running AI inference node
7. Starting the behavior tree executor

### Manual Launch (Step-by-Step)

If you prefer to launch components manually:

**1. Configure DDS (optional but recommended)**

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/cyclonedds.xml
```

**2. Launch Gazebo Simulation**

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

**3. Launch Navigation (after Gazebo is ready)**

```bash
ros2 launch catapaf_gazebo navigation.launch.py
```

**4. Launch AI Inference**

```bash
ros2 run video_to_ai video_inference_node --ros-args -p use_sim_time:=true
```

**5. Launch Behavior Tree Executor**

```bash
ros2 run catapaf_bt bt_executor --ros-args -p use_sim_time:=true
```

### Monitoring

- **Groot2**: Monitor and debug behavior trees
- **RViz2**: Visualize robot state, TF, and sensor data
- **Foxglove Studio**: Connect to ws://localhost:8765

## Manual Control

### Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Arm Controller

```bash
ros2 run catapaf_gazebo catapaf_arm_controller
```

### Ball Spawning

```bash
ros2 run ros_gz_sim create -name ball_1 \
	-file $(ros2 pkg prefix catapaf_gazebo)/share/catapaf_gazebo/models/ball/model.sdf \
	-x -0.05 -y -0.09 -z 0.15
```

### Catapult Control

```bash
ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger
ros2 service call /catapaf_arm/reset std_srvs/srv/Trigger
ros2 topic pub /catapaf_arm/position std_msgs/msg/Float64 "data: -1.2"
```

## Troubleshooting

- Ensure `IGN_GAZEBO_RESOURCE_PATH` includes the meshes directory.
- Check ROS-Gazebo bridge status if topics are missing.
- If you get DDS participant limit errors, ensure CycloneDDS is configured correctly.
- See each subpackage README for advanced usage and troubleshooting.

## Visual Segmentation Training

This repository is the main workspace for training and deploying visual segmentation models. See [ball-segmentation](https://github.com/Antoine-MR/ball-segmentation) for dataset preparation and model training.

## Documentation

- [QUICKSTART.md](QUICKSTART.md) — Step-by-step quick start guide
- [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) — Technical migration details
- [src/catapaf_gazebo/README.md](src/catapaf_gazebo/README.md) — Simulation package documentation
- [SUMMARY.md](SUMMARY.md) — Project summary and migration status

---

For further details, refer to the documentation files and each package’s README.