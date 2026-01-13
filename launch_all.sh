#!/bin/bash

# Kill existing processes
echo "Killing existing ROS 2 processes..."
pkill -f ros2
pkill -f nav2
pkill -f gz
pkill -f bt_executor
pkill -f video_inference_node
pkill -f autonomous_explorer
pkill -f arm_controller

echo "Building packages..."
colcon build --packages-select catapaf_interfaces catapaf_description catapaf_gazebo catapaf_bt distance_to_pwm video_to_ai

echo "Sourcing environment..."
source install/setup.bash

# Export DDS Config to handle participant limit
echo "Configuring CycloneDDS..."
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/cyclonedds.xml

# Check if cyclonedds.xml exists, if not create it
if [ ! -f "cyclonedds.xml" ]; then
    echo "Creating cyclonedds.xml..."
    echo '<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.eclipse.org/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.eclipse.org/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
            <Peers>
                <Peer address="localhost"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>' > cyclonedds.xml
fi

echo "Launching Gazebo Simulation..."
ros2 launch catapaf_gazebo gz_simulation.launch.py &
PID_GZ=$!

echo "Waiting for Gazebo to start (60s)..."
sleep 60

echo "Launching Navigation..."
ros2 launch catapaf_gazebo navigation.launch.py &
PID_NAV=$!

echo "Waiting for Nav2 to be ready..."
if [ -f "./wait_for_nav2.sh" ]; then
    ./wait_for_nav2.sh 120
    if [ $? -ne 0 ]; then
        echo "ERROR: Nav2 failed to start properly!"
        exit 1
    fi
else
    echo "Warning: wait_for_nav2.sh not found, waiting 30 seconds manually..."
    sleep 30
fi

echo "Launching Auxiliary Nodes (AI, PWM)..."
ros2 run video_to_ai video_inference_node &
# Add distance_to_pwm only if a specific launch file or node is needed, usually part of sim launch but safe to keep noted
# ros2 launch distance_to_pwm converter.launch.py & 

echo "Launching Behavior Tree Executor..."
ros2 run catapaf_bt bt_executor &
PID_BT=$!

echo "System Launched. Monitor via Groot2."
echo "Press Ctrl+C to stop all processes."

wait $PID_BT
