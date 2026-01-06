#!/bin/bash
# Test script for Gazebo Fortress simulation

echo "=========================================="
echo "TurtleShot Gazebo Fortress Test Script"
echo "=========================================="
echo ""

# Source ROS 2 and workspace
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

echo "✓ ROS 2 Humble sourced"
echo "✓ Workspace sourced"
echo ""

# Check if Gazebo is installed
if ! which gz > /dev/null 2>&1; then
    echo "✗ Gazebo (gz) command not found!"
    echo "  Install with: sudo apt install ros-humble-ros-gz"
    echo "  Current PATH: $PATH"
    exit 1
fi

GZ_VERSION=$(gz sim --versions 2>&1 | head -1 || echo "Gazebo Fortress")
echo "✓ Gazebo Fortress found: $GZ_VERSION"
echo ""

# Check if packages are built
if [ ! -d "install/catapaf_description" ]; then
    echo "✗ catapaf_description package not found!"
    echo "  Build with: colcon build --packages-select catapaf_description"
    exit 1
fi

if [ ! -d "install/catapaf_gazebo" ]; then
    echo "✗ catapaf_gazebo package not found!"
    echo "  Build with: colcon build --packages-select catapaf_gazebo"
    exit 1
fi

echo "✓ catapaf_description package found"
echo "✓ catapaf_gazebo package found"
echo ""

# Check URDF file exists
URDF_FILE="install/catapaf_description/share/catapaf_description/urdf/turtlebot_with_catapaf_gz.urdf.xacro"
if [ ! -f "$URDF_FILE" ]; then
    echo "✗ URDF file not found: $URDF_FILE"
    exit 1
fi

echo "✓ URDF file found"
echo ""

# Check world file exists
WORLD_FILE="install/catapaf_gazebo/share/catapaf_gazebo/worlds/turtlebot_world.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "✗ World file not found: $WORLD_FILE"
    exit 1
fi

echo "✓ World file found"
echo ""

# Check launch file exists
LAUNCH_FILE="install/catapaf_gazebo/share/catapaf_gazebo/launch/gz_simulation.launch.py"
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "✗ Launch file not found: $LAUNCH_FILE"
    exit 1
fi

echo "✓ Launch file found"
echo ""

echo "=========================================="
echo "All checks passed! ✓"
echo "=========================================="
echo ""
echo "To launch the simulation:"
echo ""
echo "  ros2 launch catapaf_gazebo gz_simulation.launch.py"
echo ""
echo "To control the robot:"
echo ""
echo "  # Terminal 1: Launch simulation"
echo "  ros2 launch catapaf_gazebo gz_simulation.launch.py"
echo ""
echo "  # Terminal 2: Control arm"
echo "  ros2 run catapaf_gazebo catapaf_arm_controller"
echo ""
echo "  # Terminal 3: Launch catapult"
echo "  ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger"
echo ""
echo "  # Terminal 4: Drive robot"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
