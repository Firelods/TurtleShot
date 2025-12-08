from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='distance_to_pwm',
            executable='converter',
            name='command_to_pwm_converter',
            output='screen',
            parameters=[
                {'input_topic': '/motor_commands'},
                {'left_motor_topic': '/left_motor_pwm'},
                {'right_motor_topic': '/right_motor_pwm'},
                {'pwm_speed': 150},
                {'linear_speed': 0.5},
                {'angular_speed': 1.0}
            ]
        )
    ])
