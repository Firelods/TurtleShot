#!/usr/bin/env python3
"""
Navigation with SLAM Launch File
This launch file starts Nav2 with SLAM Toolbox to build a new map
Use this when you want to explore and map a new environment
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_catapaf_gazebo = get_package_share_directory('catapaf_gazebo')

    # Default to false for real hardware
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')

    # Nav2 parameters
    params_file = LaunchConfiguration('params_file',
                                      default=os.path.join(pkg_catapaf_gazebo, 'config', 'nav2', 'nav2_params.yaml'))

    # SLAM parameters (default to online async mode)
    slam_params_file = LaunchConfiguration('slam_params_file',
                                           default=os.path.join(pkg_catapaf_gazebo, 'config', 'nav2', 'slam_toolbox_params.yaml'))

    container_name = 'nav2_container'

    # Lifecycle nodes (without map_server and amcl - SLAM replaces them)
    # Note: velocity_smoother removed - it interferes with cmd_vel without collision_monitor
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower']

    # SLAM Toolbox Node (replaces map_server + provides localization)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Lifecycle manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    # Nav2 container
    nav2_container = Node(
        name=container_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen')

    # Load composable nodes
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[params_file]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[params_file]),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[params_file]),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[params_file]),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[params_file]),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[params_file]),
            # velocity_smoother removed - interferes with cmd_vel without collision_monitor
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_catapaf_gazebo, 'config', 'nav2', 'nav2_params.yaml'),
            description='Full path to the ROS2 Nav2 parameters file to use'),

        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_catapaf_gazebo, 'config', 'nav2', 'slam_toolbox_params.yaml'),
            description='Full path to the SLAM Toolbox parameters file to use'),

        # Launch SLAM instead of map_server
        slam_toolbox_node,

        # Launch Nav2 components
        nav2_container,
        load_composable_nodes,
        lifecycle_manager_node,
    ])
