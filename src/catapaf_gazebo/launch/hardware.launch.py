#!/usr/bin/env python3
"""
Hardware Bringup Launch File for TurtleBot3 with Catapaf
This launch file starts all necessary nodes for the real robot (no Gazebo simulation)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # --- PACKAGE DIRECTORIES ---
    pkg_catapaf_gazebo = get_package_share_directory('catapaf_gazebo')
    pkg_turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')

    # --- LAUNCH ARGUMENTS ---
    namespace = LaunchConfiguration('namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_navigation = LaunchConfiguration('with_navigation', default='false')
    with_camera = LaunchConfiguration('with_camera', default='true')
    with_catapaf = LaunchConfiguration('with_catapaf', default='true')
    with_foxglove = LaunchConfiguration('with_foxglove', default='true')

    # --- URDF FILE ---
    # Using the catapaf URDF that includes both TurtleBot3 and the catapult
    urdf_file = os.path.join(pkg_catapaf_gazebo, 'urdf', 'turtlebot_catapaf_simple.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # --- TURTLEBOT3 BRINGUP ---
    # This launches the base TurtleBot3 drivers (motors, sensors, etc.)
    turtlebot3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_bringup, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # --- ROBOT STATE PUBLISHER ---
    # Publishes TF transforms for the robot + catapaf
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # --- OAK-D CAMERA DRIVER ---
    # Launches the YOLO OAK-D camera driver for object detection
    oak_camera_driver = Node(
        package='yolo_oak_driver',
        executable='yolo_oak_driver',
        name='yolo_oak_driver',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(with_camera)
    )

    # --- CATAPAF PWM CONVERTER ---
    # Converts distance commands to PWM signals for the catapult servo
    catapaf_pwm_converter = Node(
        package='distance_to_pwm',
        executable='converter',
        name='command_to_pwm_converter',
        namespace=namespace,
        output='screen',
        parameters=[
            {'input_topic': '/motor_commands'},
            {'left_motor_topic': '/left_motor_pwm'},
            {'right_motor_topic': '/right_motor_pwm'},
            {'pwm_speed': 150},
            {'linear_speed': 0.5},
            {'angular_speed': 1.0}
        ],
        condition=IfCondition(with_catapaf)
    )

    # --- FOXGLOVE BRIDGE ---
    # Web-based visualization tool
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        namespace=namespace,
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(with_foxglove)
    )

    # --- NAVIGATION (OPTIONAL) ---
    # Launch Nav2 with SLAM if requested
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_catapaf_gazebo, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(with_navigation)
    )

    # --- LAUNCH DESCRIPTION ---
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for all nodes'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (should be false for real hardware)'
        ),
        DeclareLaunchArgument(
            'with_navigation',
            default_value='false',
            description='Launch Nav2 navigation stack with SLAM'
        ),
        DeclareLaunchArgument(
            'with_camera',
            default_value='true',
            description='Launch OAK-D camera driver'
        ),
        DeclareLaunchArgument(
            'with_catapaf',
            default_value='true',
            description='Launch catapaf PWM converter'
        ),
        DeclareLaunchArgument(
            'with_foxglove',
            default_value='true',
            description='Launch Foxglove Bridge for visualization'
        ),

        # Launch nodes
        turtlebot3_bringup,
        robot_state_publisher,
        # oak_camera_driver,
        # catapaf_pwm_converter,
        foxglove_bridge,
        navigation_launch,
    ])
