#!/bin/bash
# Quick launch script for headless Gazebo simulation

cd ~/catapaf_ws/TurtleShot
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "================================================"
echo "Launching Gazebo Fortress (Headless Mode)"
echo "================================================"
echo ""
echo "This mode is recommended for WSL2"
echo "No GUI, but all sensors and physics work!"
echo ""
echo "To visualize, open RViz2 in another terminal:"
echo "  rviz2"
echo ""
echo "To control the arm:"
echo "  ros2 run catapaf_gazebo catapaf_arm_controller"
echo ""
echo "================================================"
echo ""

ros2 launch catapaf_gazebo gz_simulation_headless.launch.py
