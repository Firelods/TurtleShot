#!/usr/bin/env python3
"""
Catapaf Arm Controller Node

This node controls the catapult arm joint to shoot projectiles.
It provides services and topics to control the arm position and trigger launches.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
import math
import time


class CatapafArmController(Node):
    """Controller for the catapaf arm mechanism."""

    def __init__(self):
        super().__init__('catapaf_arm_controller')

        # Parameters
        self.declare_parameter('rest_position', -1.2)  # Arm down (loaded)
        self.declare_parameter('launch_position', 1.0)  # Arm up (released)
        self.declare_parameter('return_delay', 1.0)  # Delay before returning to rest

        self.rest_position = self.get_parameter('rest_position').value
        self.launch_position = self.get_parameter('launch_position').value
        self.return_delay = self.get_parameter('return_delay').value

        # Publisher for arm position
        self.position_pub = self.create_publisher(
            Float64,
            'catapaf_arm/position',
            10
        )

        # Service to trigger launch
        self.launch_srv = self.create_service(
            Trigger,
            'catapaf_arm/launch',
            self.launch_callback
        )

        # Service to reset arm
        self.reset_srv = self.create_service(
            Trigger,
            'catapaf_arm/reset',
            self.reset_callback
        )

        # Current state
        self.current_position = self.rest_position
        self.is_launching = False

        # Move to rest position on startup
        self.set_arm_position(self.rest_position)

        self.get_logger().info(f'Catapaf Arm Controller initialized')
        self.get_logger().info(f'Rest position: {self.rest_position} rad')
        self.get_logger().info(f'Launch position: {self.launch_position} rad')
        self.get_logger().info(f'Call service: ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger')

    def set_arm_position(self, position: float):
        """Set the arm to a specific position."""
        msg = Float64()
        msg.data = position
        self.position_pub.publish(msg)
        self.current_position = position
        self.get_logger().debug(f'Arm position set to: {position:.2f} rad')

    def launch_callback(self, request, response):
        """Service callback to launch the catapult."""
        if self.is_launching:
            response.success = False
            response.message = 'Launch already in progress'
            return response

        self.is_launching = True
        self.get_logger().info('🚀 Launching catapult!')

        try:
            # Move arm to launch position (fast release)
            self.set_arm_position(self.launch_position)

            # Create a timer to return to rest position
            def return_to_rest():
                self.get_logger().info('Returning arm to rest position')
                self.set_arm_position(self.rest_position)
                self.is_launching = False

            self.create_timer(
                self.return_delay,
                return_to_rest,
                callback_group=None,
                clock=self.get_clock()
            )

            response.success = True
            response.message = f'Catapult launched! Returning in {self.return_delay}s'

        except Exception as e:
            self.is_launching = False
            response.success = False
            response.message = f'Launch failed: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def reset_callback(self, request, response):
        """Service callback to reset the arm to rest position."""
        self.get_logger().info('Resetting arm to rest position')
        self.set_arm_position(self.rest_position)
        self.is_launching = False

        response.success = True
        response.message = 'Arm reset to rest position'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CatapafArmController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
