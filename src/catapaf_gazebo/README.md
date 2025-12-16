# Catapaf Gazebo - Modern Gazebo Fortress Simulation

This package provides a modern Gazebo Fortress (Ignition Gazebo 6) simulation for the TurtleBot3 robot with the catapaf (catapult) attachment.

## Features

- **Gazebo Fortress** (Ignition Gazebo 6.16.0) - Modern physics simulation
- **Differential drive control** - Move the robot with `cmd_vel` topic
- **LiDAR sensor** - 360° laser scanner for obstacle detection
- **IMU sensor** - Inertial measurement unit
- **RGB Camera** - OAK-D Pro camera simulation
- **Catapaf arm control** - Controllable catapult mechanism
- **Ball projectile** - Physics-based ball model for testing catapult
- **ROS 2 Humble integration** - Full ROS-Gazebo bridge

## Prerequisites

Ensure you have the following installed:

```bash
sudo apt-get update
sudo apt-get install ros-humble-ros-gz ros-humble-robot-state-publisher
```

## Building

```bash
cd ~/catapaf_ws/TurtleShot
colcon build --packages-select catapaf_description catapaf_gazebo
source install/setup.bash
```

## Running the Simulation

### Launch Gazebo Fortress with TurtleBot + Catapaf

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py
```

### Launch with custom spawn position

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py x_pose:=2.0 y_pose:=1.0
```

### Launch without GUI (headless)

```bash
ros2 launch catapaf_gazebo gz_simulation.launch.py gui:=false
```

## Robot Control

### Drive the robot

```bash
# Keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish directly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"
```

### Control the catapaf arm

Start the arm controller:

```bash
ros2 run catapaf_gazebo catapaf_arm_controller
```

Trigger a launch:

```bash
# Launch the catapult
ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger

# Reset arm to rest position
ros2 service call /catapaf_arm/reset std_srvs/srv/Trigger
```

Set custom arm position:

```bash
# Position in radians (-1.5 to 1.3)
ros2 topic pub /catapaf_arm/position std_msgs/msg/Float64 "data: -1.0"
```

## Spawning the Ball

To spawn a ball for catapult testing:

```bash
ros2 run ros_gz_sim create \
  -name ball_1 \
  -file $(ros2 pkg prefix catapaf_gazebo)/share/catapaf_gazebo/models/ball/model.sdf \
  -x -0.05 -y -0.09 -z 0.15
```

Position the ball near the catapult arm, then trigger a launch!

## Topics

### Published Topics

- `/scan` - LaserScan data from LiDAR
- `/imu` - IMU data
- `/camera/image_raw` - RGB camera image
- `/camera/camera_info` - Camera calibration info
- `/odom` - Odometry from differential drive
- `/joint_states` - Robot joint states
- `/tf` - TF transforms

### Subscribed Topics

- `/cmd_vel` - Velocity commands for robot base
- `/catapaf_arm/position` - Position commands for catapult arm (radians)

## Services

- `/catapaf_arm/launch` - Trigger catapult launch
- `/catapaf_arm/reset` - Reset arm to rest position

## Configuration Files

- [worlds/turtlebot_world.sdf](worlds/turtlebot_world.sdf) - Gazebo world with obstacles
- [config/catapaf_bridge.yaml](config/catapaf_bridge.yaml) - ROS-Gazebo topic bridge configuration
- [models/ball/model.sdf](models/ball/model.sdf) - Ball projectile model

## Architecture

### Plugins Used

1. **DiffDrive** - Differential drive controller for wheels
2. **JointStatePublisher** - Publishes joint states
3. **JointPositionController** - Controls catapaf arm position
4. **Sensors**:
   - GPU LiDAR
   - IMU
   - Camera
   - Depth camera

### Physics

- Physics engine: Default Gazebo Fortress physics (Dartsim or TPE)
- Update rate: 1000 Hz (1ms step size)
- Real-time factor: 1.0

## Troubleshooting

### Meshes not loading

Ensure the `IGN_GAZEBO_RESOURCE_PATH` environment variable is set:

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix catapaf_description)/share/catapaf_description/meshes
```

### Topics not bridging

Check bridge status:

```bash
ros2 topic list
ign topic -l
```

Verify bridge configuration in [config/catapaf_bridge.yaml](config/catapaf_bridge.yaml)

### Robot not moving

1. Check if cmd_vel is being published: `ros2 topic echo /cmd_vel`
2. Verify differential drive plugin is loaded in Gazebo
3. Check wheel friction parameters in URDF

## Next Steps

- Add advanced catapult trajectories
- Implement auto-aiming system
- Create target detection with camera
- Add multiple projectile types
- Implement scoring system

## License

Apache License 2.0

## Authors

- TurtleShot Team
- Clément (original robot_gazebo_sim)
