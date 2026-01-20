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

### Build

```bash
cd ~/catapaf_ws/TurtleShot
source /opt/ros/humble/setup.bash
colcon build --packages-select catapaf_description catapaf_gazebo
source install/setup.bash
```

### Launch Simulation

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

### Arm Controller

```bash
ros2 run catapaf_gazebo catapaf_arm_controller
```

### Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

### Troubleshooting

- Ensure `IGN_GAZEBO_RESOURCE_PATH` includes the meshes directory.
- Check ROS-Gazebo bridge status if topics are missing.
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