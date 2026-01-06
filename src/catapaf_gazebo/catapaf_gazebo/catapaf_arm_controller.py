#!/usr/bin/env python3
"""
Catapaf Arm Controller Node (Gazebo JointPositionController)

Controls the catapult arm by sending position commands
to a Gazebo JointPositionController.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class CatapafArmController(Node):

    def __init__(self):
        super().__init__('catapaf_arm_controller')

        # Parameters - positions pour le lancement
        self.declare_parameter('joint_name', 'catapaf_joint')
        self.declare_parameter('launch_position', -1.4)
        self.declare_parameter('rest_position', 0.0)
        self.declare_parameter('return_delay', 1.5)

        self.joint_name = self.get_parameter('joint_name').value
        self.launch_position = self.get_parameter('launch_position').value
        self.rest_position = self.get_parameter('rest_position').value
        self.return_delay = self.get_parameter('return_delay').value

        # Publisher → JointPositionController (position command)
        self.cmd_pub = self.create_publisher(
            Float64,
            'catapaf_arm/position',
            10
        )

        # Services
        self.launch_srv = self.create_service(
            Trigger,
            'catapaf_arm/launch',
            self.launch_callback
        )

        self.reset_srv = self.create_service(
            Trigger,
            'catapaf_arm/reset',
            self.reset_callback
        )

        self.is_launching = False
        self._return_timer = None

        # Initialize arm at rest position
        self.publish_position(self.rest_position)

        self.get_logger().info('Catapaf Arm Controller (Position Mode) ready')
        self.get_logger().info(
            'Call: ros2 service call /catapaf_arm/launch std_srvs/srv/Trigger'
        )

    # ----------------------------------------------------
    # Helpers
    # ----------------------------------------------------

    def publish_position(self, value: float):
        msg = Float64()
        msg.data = value
        self.cmd_pub.publish(msg)

    # ----------------------------------------------------
    # Callbacks
    # ----------------------------------------------------

    def launch_callback(self, request, response):
        if self.is_launching:
            response.success = False
            response.message = 'Launch already in progress'
            return response

        # Cancel existing timer
        if self._return_timer is not None:
            self._return_timer.cancel()
            self._return_timer = None

        self.is_launching = True
        self.get_logger().info('Launching catapult')

        # Move to launch position
        self.publish_position(self.launch_position)

        # Schedule return to rest
        self._return_timer = self.create_timer(
            self.return_delay,
            self.return_to_rest
        )

        response.success = True
        response.message = 'Launch triggered'
        return response

    def return_to_rest(self):
        # Cancel return timer
        if self._return_timer is not None:
            self._return_timer.cancel()
            self._return_timer = None

        self.get_logger().info('Returning arm to rest')
        self.publish_position(self.rest_position)
        self.is_launching = False

    def reset_callback(self, request, response):
        self.get_logger().info('Resetting arm')

        # Cancel timer
        if self._return_timer is not None:
            self._return_timer.cancel()
            self._return_timer = None

        self.publish_position(self.rest_position)
        self.is_launching = False

        response.success = True
        response.message = 'Arm reset'
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
