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
colcon build --packages-select catapaf_interfaces video_to_ai catapaf_gazebo catapaf_bt

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
./wait_for_nav2.sh 120
if [ $? -ne 0 ]; then
    echo "ERROR: Nav2 failed to start properly!"
    exit 1
fi

echo "Launching Behavior Tree Nodes..."
ros2 run video_to_ai video_inference_node &
ros2 run catapaf_gazebo autonomous_explorer &
ros2 run catapaf_gazebo arm_controller &

echo "Launching BT Executor..."
ros2 run catapaf_bt bt_executor &

echo "System Launched. Monitor via Groot2."
wait $PID_NAV
