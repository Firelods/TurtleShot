#!/usr/bin/env python3
"""
Autonomous Exploration Launch File
Launches Nav2 + SLAM Toolbox + explore_lite for autonomous map building
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_catapaf_gazebo = get_package_share_directory('catapaf_gazebo')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Explore lite parameters
    explore_params_file = os.path.join(
        pkg_catapaf_gazebo, 'config', 'nav2', 'explore_lite_params.yaml'
    )

    # Include navigation_slam launch (SLAM + Nav2)
    navigation_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_catapaf_gazebo, 'launch', 'navigation_slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Explore lite node - delayed start to let Nav2 initialize
    explore_lite_node = TimerAction(
        period=10.0,  # Wait 10 seconds for Nav2 to be ready
        actions=[
            Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                output='screen',
                parameters=[
                    explore_params_file,
                    {'use_sim_time': use_sim_time}
                ],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # Launch Nav2 + SLAM
        navigation_slam_launch,

        # Launch explore_lite after delay
        explore_lite_node,
    ])
