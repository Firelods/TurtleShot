#!/usr/bin/env python3
"""
TurtleShot Full Mission Launch File

Launches complete mission stack:
1. Gazebo simulation (catapaf_gazebo)
2. Nav2 with SLAM (catapaf_gazebo/navigation.launch.py)
3. Behavior Tree orchestrator (turtleshot_bt)

Usage:
    ros2 launch turtleshot_bt full_mission.launch.py

Optional arguments:
    gui:=false           - Launch Gazebo without GUI (headless)
    vision:=true         - Launch vision AI node (if available)
    groot:=false         - Instructions for Groot2 monitoring
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete TurtleShot mission."""

    # Package directories
    turtleshot_bt_dir = get_package_share_directory('turtleshot_bt')
    catapaf_gazebo_dir = get_package_share_directory('catapaf_gazebo')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    launch_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo with GUI'
    )

    launch_vision_arg = DeclareLaunchArgument(
        'vision',
        default_value='false',
        description='Launch vision AI node'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto-start mission on launch'
    )

    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_gui = LaunchConfiguration('gui')
    launch_vision = LaunchConfiguration('vision')
    auto_start = LaunchConfiguration('auto_start')

    # === 1. Gazebo Simulation ===
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(catapaf_gazebo_dir, 'launch', 'gz_simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': launch_gui,
        }.items()
    )

    # === 2. Nav2 with SLAM ===
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(catapaf_gazebo_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # === 3. Behavior Tree Orchestrator ===
    bt_node = Node(
        package='turtleshot_bt',
        executable='turtleshot_bt_node',
        name='turtleshot_bt',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_start': auto_start,
        }],
        emulate_tty=True
    )

    # === 4. Vision AI (optional) ===
    # Uncomment when vision node is ready
    # try:
    #     vision_ai_dir = get_package_share_directory('video_to_ai')
    #     vision_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(vision_ai_dir, 'launch', 'inference.launch.py')
    #         ),
    #         condition=IfCondition(launch_vision)
    #     )
    # except:
    #     vision_launch = LogInfo(msg="Vision AI package not found")

    # === Startup Info ===
    startup_info = LogInfo(
        msg=[
            '\n',
            '╔══════════════════════════════════════════════════════════════╗\n',
            '║         TurtleShot Behavior Tree Mission Started            ║\n',
            '╚══════════════════════════════════════════════════════════════╝\n',
            '\n',
            '📊 Components launched:\n',
            '   ✓ Gazebo simulation\n',
            '   ✓ Nav2 navigation stack (with SLAM)\n',
            '   ✓ Behavior Tree orchestrator\n',
            '\n',
            '🎨 Groot2 Monitoring:\n',
            '   1. Open Groot2\n',
            '   2. Click "Monitor" tab\n',
            '   3. Click "Connect"\n',
            '   4. Enter:\n',
            '      - Address: 127.0.0.1\n',
            '      - Publisher Port: 1666\n',
            '      - Server Port: 1667\n',
            '   5. Click "Connect"\n',
            '\n',
            '🧪 Testing without vision:\n',
            '   ros2 topic pub /detections vision_msgs/msg/Detection2DArray \\\n',
            '     "detections: [{results: [{hypothesis: {class_id: \'person\'}}]}]" --once\n',
            '\n',
            '🎯 Mission will:\n',
            '   1. Search for person with ball (vision)\n',
            '   2. Navigate to person (Nav2)\n',
            '   3. Wait for person in range (LiDAR)\n',
            '   4. Search for trash bin (vision)\n',
            '   5. Navigate to trash bin (Nav2)\n',
            '   6. Fire catapult\n',
            '\n'
        ]
    )

    # Build launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_gui_arg)
    ld.add_action(launch_vision_arg)
    ld.add_action(auto_start_arg)

    # Add startup info
    ld.add_action(startup_info)

    # Add launch includes
    ld.add_action(gazebo_launch)
    ld.add_action(navigation_launch)

    # Add BT node
    ld.add_action(bt_node)

    # Add vision if requested
    # ld.add_action(vision_launch)

    return ld
