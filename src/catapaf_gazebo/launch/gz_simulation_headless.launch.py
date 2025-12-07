import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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

    # Set IGN_GAZEBO_RESOURCE_PATH for meshes
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(pkg_catapaf_description, 'meshes')
    )

    # Gazebo Sim Launch (HEADLESS - no GUI)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r -s -v 4'],  # -s = headless
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
            'robot_description': open(urdf_file, 'r').read()
        }]
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger',
            '-file', urdf_file,
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
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        declare_world_arg,
        declare_x_arg,
        declare_y_arg,
        declare_z_arg,
        declare_use_sim_time_arg,

        # Environment
        set_ign_resource_path,

        # Nodes
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
    ])
