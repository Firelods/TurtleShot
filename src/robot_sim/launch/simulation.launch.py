import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    robot_sim_pkg = get_package_share_directory('robot_sim')
    
    # URDF file
    urdf_file = os.path.join(robot_sim_pkg, 'urdf', 'turtlebot3_burger_simple.urdf')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )
    
    # Get launch configuration
    namespace = LaunchConfiguration('namespace')
    
    # Process URDF
    robot_desc = Command(['xacro ', urdf_file, ' namespace:=', namespace])

    return LaunchDescription([
        namespace_arg,
        
        # 1. Robot State Publisher (Publishes the robot model)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # 2. Joint State Publisher (Publishes wheel states)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            output='screen',
        ),

        # 3. Scene Publisher (Publishes 3D walls and obstacles markers)
        Node(
            package='fake_lidar',
            executable='scene_publisher',
            name='scene_publisher',
            namespace=namespace,
            output='screen'
        ),

        # 4. Fake LiDAR (Raycasts against the scene and publishes /scan)
        Node(
            package='fake_lidar',
            executable='fake_lidar', 
            name='fake_lidar',
            namespace=namespace,
            output='screen',
            parameters=[{
                'scan_rate': 10.0,
                'range_max': 3.5
            }]
        ),

        # 5. Robot Simulator & Random Walker (Handles movement and physics)
        Node(
            package='robot_sim',
            executable='simple_robot_simulator',
            name='simple_robot_simulator',
            namespace=namespace,
            output='screen'
        ),

        # 6. Foxglove Bridge (Visualization)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            namespace=namespace,
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0"
            }]
        )
    ])
