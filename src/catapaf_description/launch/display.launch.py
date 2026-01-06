from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg = FindPackageShare('catapaf_description')

    # Fichier Xacro
    xacro_file = PathJoinSubstitution([
        pkg, 'urdf', 'catapaf.urdf.xacro'
    ])

    # robot_description généré dynamiquement
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([

        # Publie la description du robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Sliders pour les joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2'
        ),
    ])
