import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_catapaf_gazebo = get_package_share_directory('catapaf_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.expanduser('~/my_costmap.yaml'))
    params_file = LaunchConfiguration('params_file',
                                      default=os.path.join(pkg_catapaf_gazebo, 'config', 'nav2', 'nav2_params.yaml'))
    
    container_name = 'nav2_container'

    lifecycle_nodes = ['map_server',
                       'amcl',
                       'controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}])

    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'alpha1': 0.2},
                    {'alpha2': 0.2},
                    {'alpha3': 0.2},
                    {'alpha4': 0.2},
                    {'alpha5': 0.2},
                    {'base_frame_id': 'base_footprint'},
                    {'beam_skip_distance': 0.5},
                    {'beam_skip_error_threshold': 0.9},
                    {'beam_skip_threshold': 0.3},
                    {'do_beamskip': False},
                    {'global_frame_id': 'map'},
                    {'lambda_short': 0.1},
                    {'laser_likelihood_max_dist': 2.0},
                    {'laser_max_range': 100.0},
                    {'laser_min_range': -1.0},
                    {'laser_model_type': 'likelihood_field'},
                    {'max_beams': 60},
                    {'max_particles': 2000},
                    {'min_particles': 500},
                    {'odom_frame_id': 'odom'},
                    {'pf_err': 0.05},
                    {'pf_z': 0.99},
                    {'recovery_alpha_fast': 0.0},
                    {'recovery_alpha_slow': 0.0},
                    {'resample_interval': 1},
                    {'robot_model_type': 'nav2_amcl::DifferentialMotionModel'},
                    {'save_pose_rate': 0.5},
                    {'sigma_hit': 0.2},
                    {'tf_broadcast': True},
                    {'transform_tolerance': 1.0},
                    {'update_min_a': 0.2},
                    {'update_min_d': 0.25},
                    {'z_hit': 0.5},
                    {'z_max': 0.05},
                    {'z_rand': 0.5},
                    {'z_short': 0.05}])

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
                parameters=[params_file],
                remappings=[('/cmd_vel', '/cmd_vel_nav')]),
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
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[params_file]),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'),
        
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/my_costmap.yaml'),
            description='Full path to map yaml file to load'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_catapaf_gazebo, 'config', 'nav2', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        map_server_node,
        amcl_node,
        nav2_container,
        load_composable_nodes,
        lifecycle_manager_node,
    ])
