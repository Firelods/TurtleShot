#!/usr/bin/env python3
"""
Mock NavigateToPose Action Server

Simule un serveur Nav2 qui répond toujours SUCCESS après 2 secondes.
Utile pour tester le BT sans avoir Nav2 complètement configuré.

Usage:
    ros2 run turtleshot_bt mock_nav_server
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
import time


class MockNavServer(Node):
    """Mock Nav2 action server for testing."""

    def __init__(self):
        super().__init__('mock_nav_server')
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_callback
        )
        self.get_logger().info('Mock NavigateToPose server started')
        self.get_logger().info('Will accept all navigation goals and return SUCCESS')

    def execute_callback(self, goal_handle):
        """Execute navigation goal (fake)."""
        self.get_logger().info(
            f'Navigating to x={goal_handle.request.pose.pose.position.x:.2f}, '
            f'y={goal_handle.request.pose.pose.position.y:.2f}'
        )

        # Simulate navigation time
        for i in range(10):
            if not goal_handle.is_active:
                self.get_logger().info('Goal canceled')
                return NavigateToPose.Result()

            # Send feedback
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = 5.0 - (i * 0.5)
            goal_handle.publish_feedback(feedback)

            time.sleep(0.2)

        goal_handle.succeed()
        self.get_logger().info('Navigation succeeded!')

        result = NavigateToPose.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    server = MockNavServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
