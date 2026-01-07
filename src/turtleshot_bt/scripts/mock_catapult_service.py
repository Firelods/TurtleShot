#!/usr/bin/env python3
"""
Mock Fire Catapult Service

Simule le service de tir de catapulte.

Usage:
    ros2 run turtleshot_bt mock_catapult_service
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MockCatapultService(Node):
    """Mock catapult service for testing."""

    def __init__(self):
        super().__init__('mock_catapult_service')
        self.service = self.create_service(
            Trigger,
            '/fire_catapult',
            self.fire_callback
        )
        self.get_logger().info('Mock catapult service started on /fire_catapult')

    def fire_callback(self, request, response):
        """Handle fire request."""
        self.get_logger().info('🎯 FIRE! Ball launched!')
        response.success = True
        response.message = 'Ball fired successfully'
        return response


def main(args=None):
    rclpy.init(args=args)
    service = MockCatapultService()

    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
