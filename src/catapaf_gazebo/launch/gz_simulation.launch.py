import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Packages
    pkg_catapaf_description = get_package_share_directory('catapaf_description')
    pkg_catapaf_gazebo = get_package_share_directory('catapaf_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    world_file = os.path.join(pkg_catapaf_gazebo, 'worlds', 'turtlebot_world.sdf')
    urdf_file = os.path.join(pkg_catapaf_description, 'urdf', 'turtlebot_with_catapaf_gz.urdf.xacro')
    bridge_config = os.path.join(pkg_catapaf_gazebo, 'config', 'catapaf_bridge.yaml')

    # Read and process URDF to replace package:// URIs with file:// URIs
    # This is needed because Gazebo Ignition doesn't support package:// URIs
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # Replace package:// URIs with file:// URIs using absolute paths
    robot_description_content = robot_description_content.replace(
        'package://catapaf_description/',
        f'file://{pkg_catapaf_description}/'
    )

    # Launch Arguments
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='SDF world file'
    )

    declare_x_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of robot spawn'
    )

    declare_y_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of robot spawn'
    )

    declare_z_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.01',
        description='Z position of robot spawn'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    # Set IGN_GAZEBO_RESOURCE_PATH for meshes
    # Point to the share directory so model:// URIs can be resolved
    ign_resource_path = os.pathsep.join([
        os.path.join(pkg_catapaf_description),
        os.path.join(pkg_catapaf_description, 'meshes'),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ])

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=ign_resource_path
    )

    # Also set GZ_SIM_RESOURCE_PATH for newer Gazebo versions
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=ign_resource_path
    )

    # Gazebo Sim Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r -v 4'],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content
        }]
    )

    # Spawn Robot
    # Use -topic instead of -file to get the robot description from robot_state_publisher
    # This allows ROS to properly resolve package:// URIs
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger',
            '-topic', '/robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config}'
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Foxglove Bridge for web visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            {'port': 8765},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'send_buffer_limit': 10000000}
        ]
    )

    # Odometry to TF publisher
    # Converts /odom messages to TF transform (odom -> base_footprint)
    odom_to_tf_node = Node(
        package='catapaf_gazebo',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # RViz2 (optional)
    rviz_config = os.path.join(pkg_catapaf_gazebo, 'config', 'view_robot.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        # Arguments
        declare_world_arg,
        declare_x_arg,
        declare_y_arg,
        declare_z_arg,
        declare_use_sim_time_arg,
        declare_gui_arg,

        # Environment
        set_ign_resource_path,
        set_gz_resource_path,

        # Nodes
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        foxglove_bridge,
        odom_to_tf_node,
        # rviz2,  # Uncomment if you want RViz2
    ])
