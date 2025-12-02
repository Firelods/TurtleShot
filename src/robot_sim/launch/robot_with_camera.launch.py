import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # Paths
    robot_sim_pkg = get_package_share_directory('robot_sim')
    oakd_camera_pkg = get_package_share_directory('oakd_camera_driver')
    # depthai_prefix = get_package_share_directory('depthai_ros_driver')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot (e.g., "robot1", "turtlebot", etc.)'
    )
    
    # Get launch configuration
    namespace = LaunchConfiguration('namespace')

    # Process Xacro with namespace - using our simplified URDF
    xacro_file = os.path.join(robot_sim_pkg, 'urdf', 'turtlebot3_burger_simple.urdf')
    robot_desc = Command(['xacro ', xacro_file, ' namespace:=', namespace])

    return LaunchDescription([
        # Declare namespace argument
        namespace_arg,
        
        # 1. Publish Robot Model (TurtleBot 3)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # 1.5. Publish Joint States (for wheels and other movable parts)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            output='screen',
        ),

        # 2. Run the Camera Driver
        # TODO: camera.launch.py is missing in oakd_camera_driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(oakd_camera_pkg, 'launch', 'camera.launch.py')
            )
        ),

        # 3. Attach Camera to Robot (Mount it on top of base_link, e.g., 20cm up)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_mount_link',
            namespace=namespace,
            arguments=['0.05', '0', '0.2', '0', '0', '0', 'base_link', 'oak-d-base-frame'],
            output='screen'
        ),

        # 4. Rotate the Robot based on Camera IMU (World -> Base Footprint)
        Node(
            package='oakd_camera_driver',
            executable='imu_tf_publisher',
            name='imu_tf_publisher',
            namespace=namespace,
            output='screen'
        ),


        # 5. 3D Scene/Map Publisher
        # TODO: scene_publisher executable is missing in oakd_camera_driver
        # Node(
        #     package='oakd_camera_driver',
        #     executable='scene_publisher',
        #     name='scene_publisher',
        #     namespace=namespace,
        #     output='screen'
        # ),

        # 6. Foxglove Bridge
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
