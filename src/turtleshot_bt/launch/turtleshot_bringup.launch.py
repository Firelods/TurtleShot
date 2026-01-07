#!/usr/bin/env python3
"""
TurtleShot Complete Bringup Launch File

This launch file starts:
1. Gazebo simulation (optional, via argument)
2. Navigation stack (Nav2)
3. Vision AI node (if available)
4. Behavior Tree orchestrator

Usage:
    ros2 launch turtleshot_bt turtleshot_bringup.launch.py
    ros2 launch turtleshot_bt turtleshot_bringup.launch.py sim:=false  # Without Gazebo
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for TurtleShot mission."""

    # Package directories
    turtleshot_bt_dir = get_package_share_directory('turtleshot_bt')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    launch_sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Launch Gazebo simulation'
    )

    launch_nav_arg = DeclareLaunchArgument(
        'nav',
        default_value='false',
        description='Launch Nav2 navigation stack'
    )

    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_sim = LaunchConfiguration('sim')
    launch_nav = LaunchConfiguration('nav')

    # === Gazebo Simulation (optional) ===
    try:
        catapaf_gazebo_dir = get_package_share_directory('catapaf_gazebo')
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(catapaf_gazebo_dir, 'launch', 'gz_simulation.launch.py')
            ),
            condition=IfCondition(launch_sim)
        )
    except:
        gazebo_launch = None

    # === Nav2 Navigation (optional) ===
    # Uncomment when Nav2 is configured
    # try:
    #     nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    #     nav2_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    #         ),
    #         condition=IfCondition(launch_nav),
    #         launch_arguments={
    #             'use_sim_time': use_sim_time,
    #         }.items()
    #     )
    # except:
    #     nav2_launch = None

    # === Behavior Tree Orchestrator ===
    bt_node = Node(
        package='turtleshot_bt',
        executable='turtleshot_bt_node',
        name='turtleshot_bt',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        emulate_tty=True  # For colored output
    )

    # Build launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_sim_arg)
    ld.add_action(launch_nav_arg)

    # Add nodes
    if gazebo_launch:
        ld.add_action(gazebo_launch)
    # if nav2_launch:
    #     ld.add_action(nav2_launch)
    ld.add_action(bt_node)

    return ld
