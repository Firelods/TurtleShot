#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # --- DIRECTORIES ---
    pkg_catapaf_gazebo = get_package_share_directory('catapaf_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    rviz_config_file = os.path.join(
        pkg_catapaf_gazebo,
        'rviz',
        'tb3_gazebo.rviz'
    )
    
    

    # --- FILES ---
    world_file = os.path.join(pkg_catapaf_gazebo, 'worlds', 'Ubiquarium.world')
    urdf_file = os.path.join(pkg_catapaf_gazebo, 'urdf', 'turtlebot_catapaf_simple.urdf')
    bridge_config = os.path.join(pkg_catapaf_gazebo, 'config', 'catapaf_bridge.yaml')
    with open(urdf_file, 'r') as infp:
            robot_description_content = infp.read()
    # --- SIM OPTIONS ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.5')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')

    # --- GAZEBO RESOURCES ---
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(pkg_catapaf_gazebo)
    )


    # --- GZ SERVER ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world_file]
        }.items()
    )

    # --- GZ CLIENT ---
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v4 '
        }.items()
    )

    # --- ROBOT STATE PUBLISHER ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},{'publish_frequency': 30.0},{'robot_description': robot_description_content}],
        arguments=[urdf_file]
    )

    robot_description_config = xacro.process_file(urdf_file).toxml()

    # --- SPAWN ROBOT ---
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "turtlebot_catapaf",
            "-topic", "robot_description",

            # "-string",
            # robot_description_config,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
        ],
        output="screen",    
    )

    bridge_params = os.path.join(
        pkg_catapaf_gazebo,
        'config',
        'catapaf_bridge.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )


    rviz_cmd = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2',
       output='screen',
       parameters=[{'use_sim_time': use_sim_time}],
       arguments=['-d', rviz_config_file]
    )

    # --- CATAPAF ARM CONTROLLER ---
    catapaf_arm_controller = Node(
        package='catapaf_gazebo',
        executable='catapaf_arm_controller',
        name='catapaf_arm_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- ODOM TO TF ---
    odom_to_tf = Node(
        package='catapaf_gazebo',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- BALL SPAWNER ---
    ball_spawner = Node(
        package='catapaf_gazebo',
        executable='ball_spawner',
        name='ball_spawner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'auto_respawn': True},
            {'spawn_delay': 3.0},  # Wait for simulation to be ready
            # Position above catapaf_arm: arm is at (-0.03, -0.098, 0.11)
            # Arm collision box: 0.12 x 0.05 x 0.03, centered at arm pose + (0.05, 0, 0)
            # Ball radius: 0.03m, so place at arm_z + arm_height/2 + ball_radius + small_gap
            {'ball_x': 0.45},      # Slightly forward from arm center
            {'ball_y': -0.098},    # Same Y as arm
            {'ball_z': 0.145}      # 0.11 (arm base) + 0.015 (half box height) + 0.03 (ball radius) - 0.01 (nestled in)
        ]
    )

    # --- FINAL LAUNCH DESCRIPTION ---
    return LaunchDescription([
        set_env_vars_resources,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        catapaf_arm_controller,
        odom_to_tf,
        # ball_spawner,
        rviz_cmd
    ])
