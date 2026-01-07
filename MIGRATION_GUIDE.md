# Migration Guide: Gazebo Classic → Gazebo Fortress

This guide documents the migration from Gazebo Classic (v11) to Gazebo Fortress (Ignition Gazebo 6) for the TurtleShot project.

## Summary

✅ **Migration Complete!**

The TurtleShot robot with catapaf attachment has been successfully migrated to Gazebo Fortress with modern ROS 2 Humble integration.

## What Changed

### 1. Robot Description Files

**New Files Created:**

- [src/catapaf_description/urdf/turtlebot_with_catapaf_gz.urdf.xacro](src/catapaf_description/urdf/turtlebot_with_catapaf_gz.urdf.xacro)
  - Modern URDF/XACRO with Gazebo Fortress plugins
  - Based on your existing `turtlebot_with_catapaf.urdf`
  - Uses Ignition Gazebo plugin syntax

**Old Files (Kept for Reference):**

- `src/robot_gazebo_sim/urdf/turtlebot_with_catapaf.urdf` - Gazebo Classic version
- `src/catapaf_description/urdf/catapaf.urdf.xacro` - Generic XACRO (still used)

### 2. Simulation Package Updates

**catapaf_gazebo Package:**

```
src/catapaf_gazebo/
├── launch/
│   ├── simulation.launch.py          (old - basic)
│   └── gz_simulation.launch.py       (NEW - full featured)
├── worlds/
│   ├── empty.sdf                     (old - empty)
│   └── turtlebot_world.sdf           (NEW - with obstacles & targets)
├── models/
│   ├── turtlebot_catapaf/            (old - needs update)
│   └── ball/                         (NEW - projectile model)
│       ├── model.sdf
│       └── model.config
├── config/
│   └── catapaf_bridge.yaml           (UPDATED - more topics)
├── catapaf_gazebo/
│   └── catapaf_arm_controller.py     (NEW - arm control node)
└── README.md                          (NEW - documentation)
```

### 3. Plugin Migration

#### Gazebo Classic → Gazebo Fortress Plugin Mapping

| **Function** | **Gazebo Classic** | **Gazebo Fortress** |
|--------------|-------------------|---------------------|
| Differential Drive | `libgazebo_ros_diff_drive.so` | `ignition-gazebo-diff-drive-system` |
| LiDAR Sensor | `libgazebo_ros_ray_sensor.so` | `gpu_lidar` sensor type |
| IMU Sensor | `libgazebo_ros_imu_sensor.so` | `imu` sensor type |
| Camera | `libgazebo_ros_camera.so` | `camera` sensor type |
| Joint State Publisher | Built-in | `ignition-gazebo-joint-state-publisher-system` |
| Joint Position Control | N/A (added) | `ignition-gazebo-joint-position-controller-system` |

### 4. Launch File Changes

**Old (Gazebo Classic):**

```python
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('gazebo_ros'),
                     'launch', 'gazebo.launch.py')
    ])
)
```

**New (Gazebo Fortress):**

```python
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('ros_gz_sim'),
                     'launch', 'gz_sim.launch.py')
    ]),
    launch_arguments={'gz_args': [world, ' -r -v 4']}.items()
)
```

### 5. ROS-Gazebo Bridge

**Configuration File:** [src/catapaf_gazebo/config/catapaf_bridge.yaml](src/catapaf_gazebo/config/catapaf_bridge.yaml)

**New Topics Bridged:**

- `/cmd_vel` - Command velocity (ROS → Gazebo)
- `/odom` - Odometry (Gazebo → ROS)
- `/scan` - LiDAR scan (Gazebo → ROS)
- `/imu` - IMU data (Gazebo → ROS)
- `/camera/image_raw` - Camera image (Gazebo → ROS)
- `/camera/camera_info` - Camera info (Gazebo → ROS)
- `/joint_states` - Joint states (Gazebo → ROS)
- `/catapaf_arm/position` - Arm control (ROS → Gazebo)
- `/tf` - Transforms (Gazebo → ROS)

## New Features

### 1. Catapaf Arm Controller

**Node:** `catapaf_arm_controller`

**Services:**

- `/catapaf_arm/launch` - Trigger catapult launch
- `/catapaf_arm/reset` - Reset arm to rest position

**Topics:**

- `/catapaf_arm/position` - Set arm position (radians)

**Usage:**

```bash
# Start controller
ros2 run catapaf_gazebo catapaf_arm_controller

# Launch catapult
ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger

# Set custom position
ros2 topic pub /catapaf_arm/position std_msgs/msg/Float64 "data: -1.0"
```

### 2. Ball Projectile Model

**Location:** `src/catapaf_gazebo/models/ball/model.sdf`

**Features:**

- Realistic physics (mass: 50g, radius: 5cm)
- Bounce coefficient: 0.7
- Orange color for visibility
- Friction and contact properties

**Spawn Command:**

```bash
ros2 run ros_gz_sim create \
  -name ball_1 \
  -file $(ros2 pkg prefix catapaf_gazebo)/share/catapaf_gazebo/models/ball/model.sdf \
  -x -0.05 -y -0.09 -z 0.15
```

### 3. Enhanced World

**Features:**

- Ground plane with realistic materials
- Static obstacles (boxes, cylinders) for testing
- Target zone for catapult aiming
- Proper lighting and shadows
- GUI plugins (3D view, world control, stats, entity tree)
- Image display for camera feed
- LiDAR visualization

## Installation & Usage

### Build the Packages

```bash
cd ~/catapaf_ws/TurtleShot
colcon build --packages-select catapaf_description catapaf_gazebo
source install/setup.bash
```

### Launch Simulation

```bash
# Full simulation with GUI
ros2 launch catapaf_gazebo gz_simulation.launch.py

# Custom spawn position
ros2 launch catapaf_gazebo gz_simulation.launch.py x_pose:=2.0 y_pose:=1.0

# Headless (no GUI)
ros2 launch catapaf_gazebo gz_simulation.launch.py gui:=false
```

### Quick Test

```bash
# Run test script
./test_gazebo_fortress.sh
```

## Verification

Run the test script to verify everything is working:

```bash
cd ~/catapaf_ws/TurtleShot
source install/setup.bash
./test_gazebo_fortress.sh
```

Expected output:

```
✓ ROS 2 Humble sourced
✓ Workspace sourced
✓ Gazebo Fortress found
✓ catapaf_description package found
✓ catapaf_gazebo package found
✓ URDF file found
✓ World file found
✓ Launch file found
All checks passed! ✓
```

## Architecture Comparison

### Old Architecture (Gazebo Classic)

```
robot_gazebo_sim
├── urdf/turtlebot_with_catapaf.urdf (monolithic)
├── launch/gazebo_wsl.launch.py
└── Uses libgazebo_ros_* plugins
```

### New Architecture (Gazebo Fortress)

```
catapaf_description               catapaf_gazebo
├── urdf/                          ├── launch/
│   ├── catapaf.urdf.xacro         │   └── gz_simulation.launch.py
│   └── turtlebot_with_catapaf_gz  ├── worlds/
│       .urdf.xacro                │   └── turtlebot_world.sdf
├── meshes/                        ├── models/
│   ├── bases/                     │   └── ball/
│   ├── wheels/                    ├── config/
│   ├── sensors/                   │   └── catapaf_bridge.yaml
│   └── catapaf/                   └── catapaf_gazebo/
                                       └── catapaf_arm_controller.py
```

## Compatibility Notes

### Keep Both Versions?

**Yes!** Both simulation systems can coexist:

- **Gazebo Classic** (`robot_gazebo_sim`) - For existing workflows
- **Gazebo Fortress** (`catapaf_gazebo`) - For new development

### Which Should I Use?

| **Use Case** | **Recommendation** |
|--------------|-------------------|
| New development | **Gazebo Fortress** (catapaf_gazebo) |
| Production stability | Gazebo Classic (robot_gazebo_sim) |
| Learning modern ROS 2 | **Gazebo Fortress** (catapaf_gazebo) |
| WSL2 performance | **Gazebo Fortress** (better GPU support) |

## Next Steps

### Recommended Enhancements

1. **Add catapult physics simulation**
   - Spring/tension mechanism
   - Projectile trajectory calculation
   - Impact detection

2. **Implement auto-aiming**
   - Target detection with camera
   - Trajectory planning
   - Closed-loop control

3. **Multiple projectile types**
   - Heavy ball, light ball, etc.
   - Different bounce coefficients

4. **Scoring system**
   - Detect hits on target
   - Track accuracy statistics

5. **Advanced behaviors**
   - Autonomous navigation to firing position
   - Multiple targets
   - Moving targets

## Troubleshooting

### Issue: Meshes not loading

**Solution:**

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix catapaf_description)/share/catapaf_description/meshes
```

### Issue: Bridge not working

**Check topics:**

```bash
ros2 topic list
ign topic -l  # or gz topic -l
```

**Restart bridge:**

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=<path_to_bridge.yaml>
```

### Issue: Robot not moving

1. Check cmd_vel: `ros2 topic echo /cmd_vel`
2. Check wheel joints are moving in Gazebo GUI
3. Verify friction parameters in URDF

### Issue: Arm not responding

1. Ensure controller is running: `ros2 run catapaf_gazebo catapaf_arm_controller`
2. Check service: `ros2 service list | grep catapaf`
3. Verify bridge config includes arm topics

## References

- **Gazebo Fortress Documentation:** https://gazebosim.org/docs/fortress
- **ROS 2 Humble:** https://docs.ros.org/en/humble/
- **ros_gz Integration:** https://github.com/gazebosim/ros_gz
- **TurtleBot3:** https://emanual.robotis.com/docs/en/platform/turtlebot3/

## Credits

- **Original robot_gazebo_sim:** Your friend
- **Migration & enhancements:** Claude Code
- **Project:** TurtleShot - TurtleBot3 with Catapaf

---

**Status:** ✅ Migration Complete - Ready for testing!

**Date:** 2025-12-07

**Gazebo Version:** Fortress (Ignition Gazebo 6.16.0)

**ROS Version:** ROS 2 Humble

**Platform:** WSL2 Ubuntu 22.04
