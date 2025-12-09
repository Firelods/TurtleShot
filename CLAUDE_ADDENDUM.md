# CLAUDE.md Addendum - Gazebo Fortress Support

**Note:** Ajoutez cette section au fichier CLAUDE.md principal après la section "Key Packages".

---

## Gazebo Fortress Support (NEW)

### catapaf_gazebo (Modern Gazebo)
Modern Gazebo Fortress simulation package with full ROS 2 integration.

- **Launch**: `ros2 launch catapaf_gazebo gz_simulation.launch.py`
- **URDF**: `catapaf_description/urdf/turtlebot_with_catapaf_gz.urdf.xacro`
- **World**: `catapaf_gazebo/worlds/turtlebot_world.sdf`
- **Nodes**:
  - `catapaf_arm_controller` - Service-based catapult controller
- **Features**:
  - Ignition Gazebo 6 (Fortress) plugins
  - Modern sensor stack (GPU LiDAR, IMU, Camera)
  - Joint position control for catapult arm
  - Ball projectile model with realistic physics
  - ROS-Gazebo bridge configuration

The launch file starts:
1. Gazebo Fortress server and client
2. robot_state_publisher (URDF → TF)
3. Spawns robot at specified pose
4. ROS-Gazebo bridge (topic translation)
5. Catapaf arm controller (optional)

**Quick Start:**
```bash
# Terminal 1: Launch simulation
ros2 launch catapaf_gazebo gz_simulation.launch.py

# Terminal 2: Control arm
ros2 run catapaf_gazebo catapaf_arm_controller

# Terminal 3: Launch catapult
ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger
```

**Documentation:**
- [src/catapaf_gazebo/README.md](src/catapaf_gazebo/README.md) - Full package documentation
- [MIGRATION_GUIDE.md](MIGRATION_GUIDE.md) - Gazebo Classic → Fortress migration
- [QUICKSTART.md](QUICKSTART.md) - Quick start guide in French

### catapaf_description (Updated)
Now includes Gazebo Fortress-compatible URDF files.

- **New File**: `urdf/turtlebot_with_catapaf_gz.urdf.xacro`
  - Modern Gazebo Fortress plugins
  - DiffDrive, sensors (LiDAR, IMU, camera), joint controllers
  - Based on existing `turtlebot_with_catapaf.urdf` structure
- **Existing Files**: Still compatible with Gazebo Classic

### Simulation Comparison

| Feature | robot_gazebo_sim (Classic) | catapaf_gazebo (Fortress) |
|---------|---------------------------|---------------------------|
| Gazebo Version | Classic 11.10.2 (EOL) | Fortress 6.16.0 (Modern) |
| ROS 2 Support | Basic | Full integration |
| Catapult Control | Manual joint control | Service-based controller |
| Physics Engine | ODE | Dartsim/TPE |
| GPU Acceleration | Limited | Full support |
| Sensors | Ray sensor, Camera | GPU LiDAR, Modern sensors |
| Ball Model | No | Yes (projectile physics) |
| Recommended For | Existing workflows | New development |

### Running Gazebo Fortress

```bash
# Build packages
colcon build --packages-select catapaf_description catapaf_gazebo
source install/setup.bash

# Launch simulation
ros2 launch catapaf_gazebo gz_simulation.launch.py

# Options
ros2 launch catapaf_gazebo gz_simulation.launch.py \
  x_pose:=2.0 y_pose:=1.0 gui:=true
```

### Catapaf Arm Control (New)

The catapaf arm is now controlled via ROS 2 services and topics:

**Services:**
- `/catapaf_arm/launch` - Trigger catapult launch sequence
- `/catapaf_arm/reset` - Reset arm to rest position

**Topics:**
- `/catapaf_arm/position` - Direct position control (Float64, radians)
  - Range: -1.5 (rest/loaded) to 1.3 (launched)

**Example Usage:**
```bash
# Launch catapult
ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger

# Set position manually
ros2 topic pub /catapaf_arm/position std_msgs/msg/Float64 "data: -1.2"

# Reset to rest
ros2 service call /catapaf_arm/reset std_srvs/srv/Trigger
```

### Ball Projectile Model

A physics-based ball model is available for testing the catapult:

**Spawn Command:**
```bash
ros2 run ros_gz_sim create \
  -name ball_1 \
  -file $(ros2 pkg prefix catapaf_gazebo)/share/catapaf_gazebo/models/ball/model.sdf \
  -x -0.05 -y -0.09 -z 0.15
```

**Properties:**
- Mass: 50g
- Radius: 5cm
- Bounce coefficient: 0.7
- Friction: 1.0
- Color: Orange

### Testing

A test script is available to verify the installation:

```bash
./test_gazebo_fortress.sh
```

This checks for:
- ROS 2 environment
- Gazebo Fortress installation
- Package builds
- Required files (URDF, world, launch)
