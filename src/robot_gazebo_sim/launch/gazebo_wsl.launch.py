import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_robot_gazebo_sim = get_package_share_directory('robot_gazebo_sim')
    
    # Paths
    urdf_file = os.path.join(pkg_robot_gazebo_sim, 'urdf', 'turtlebot3_burger.urdf')
    world_file = os.path.join(pkg_robot_gazebo_sim, 'worlds', 'turtlebot3_world.world')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable Gazebo GUI'
    )
    
    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    
    # Process URDF
    robot_desc = Command(['xacro ', urdf_file, ' namespace:=', namespace])

    # WSL-optimized environment variables for Gazebo GUI
    gazebo_env = {
        'LIBGL_ALWAYS_SOFTWARE': '0',  # HARDWARE ACCELERATION ENABLED (Disabled forced software rendering)
        # 'MESA_GL_VERSION_OVERRIDE': '3.3',  # Commented out to allow driver defaults
        # 'MESA_GLSL_VERSION_OVERRIDE': '330', # Commented out to allow driver defaults
        'GAZEBO_MODEL_PATH': os.path.join(pkg_robot_gazebo_sim, 'models'),
        'GAZEBO_RESOURCE_PATH': '/usr/share/gazebo-11',
        'OGRE_RTShader_Write': '1',
    }

    # 1. Start Gazebo Server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', world_file],
        output='screen',
        shell=False,
        additional_env=gazebo_env
    )
    
    # 2. Start Gazebo Client with WSL-optimized settings
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        shell=False,
        additional_env=gazebo_env
    )
    
    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }],
    )
    
    # 4. Spawn Robot in Gazebo (wait for gzserver to start)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )
    
    # 5. Random Walker Node
    random_walker = Node(
        package='robot_gazebo_sim',
        executable='random_walker',
        name='random_walker',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 6. Foxglove Bridge (for web-based visualization as backup)
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        namespace=namespace,
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'use_sim_time': use_sim_time,
            'send_buffer_limit': 10000000
        }],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        x_pose_arg,
        y_pose_arg,
        gui_arg,
        
        # Start components
        gzserver,
        gzclient,
        robot_state_publisher,
        
        # Spawn robot after a short delay (managed by spawn_entity itself)
        spawn_robot,
        
        # Start random walker
        random_walker,
        
        # Start Foxglove bridge
        foxglove_bridge,
    ])
