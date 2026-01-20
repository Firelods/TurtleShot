# Catapaf Description

This package contains the URDF/Xacro robot description files and meshes for the TurtleBot3 with catapult attachment.

## Overview

The `catapaf_description` package provides:

- URDF and Xacro files for the robot model
- 3D mesh files for visualization
- Launch files for RViz2 visualization

## Contents

### URDF Files

- `urdf/catapaf.urdf.xacro` - Catapult arm description (Xacro)
- `urdf/turtlebot_with_catapaf.urdf` - Complete robot URDF for classic Gazebo
- `urdf/turtlebot_with_catapaf_gz.urdf.xacro` - Complete robot for Gazebo Fortress

### Meshes

The `meshes/` directory contains STL and DAE files organized by component:

- `meshes/bases/` - TurtleBot3 base
- `meshes/wheels/` - Wheel meshes
- `meshes/sensors/` - LiDAR, camera sensors
- `meshes/catapaf/` - Catapult mechanism

## Dependencies

- `xacro` - Xacro macro processor
- `robot_state_publisher` - Publishes TF from URDF
- `joint_state_publisher_gui` - Interactive joint control
- `rviz2` - 3D visualization

## Building

```bash
colcon build --packages-select catapaf_description
source install/setup.bash
```

## Usage

### Visualize in RViz2

```bash
ros2 launch catapaf_description display.launch.py
```

This launches:
- `robot_state_publisher` with the URDF
- `joint_state_publisher_gui` for interactive joint control
- RViz2 with a pre-configured view

### Using in Other Packages

To use this description in another launch file:

```python
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

pkg_dir = get_package_share_directory('catapaf_description')
urdf_file = os.path.join(pkg_dir, 'urdf', 'turtlebot_with_catapaf_gz.urdf.xacro')
robot_description = xacro.process_file(urdf_file).toxml()

robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]
)
```

## TF Frames

The robot description publishes the following TF tree:

```
base_footprint
└── base_link
    ├── wheel_left_link
    ├── wheel_right_link
    ├── caster_back_link
    ├── base_scan (LiDAR)
    ├── camera_link
    │   └── camera_optical_frame
    └── catapaf_base_link
        └── catapaf_arm_link
```

## License

Apache License 2.0
