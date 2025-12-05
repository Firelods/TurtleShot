#!/usr/bin/env python3
"""
Random Walker for Gazebo Simulation
Publishes velocity commands while avoiding obstacles.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import random


class RandomWalker(Node):
    def __init__(self):
        super().__init__('random_walker')

        # ----- State ---------------------------------------------------------
        self.state = 'FORWARD'          # FORWARD or TURN
        self.turn_direction = 1         # 1 = left, -1 = right
        self.turn_start_time = 0
        self.min_obstacle_dist = 0.6    # metres

        # ----- Publishers ----------------------------------------------------
        # Publish on the *global* /cmd_vel topic (matches Gazebo diff_drive_controller)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ----- Subscribers ---------------------------------------------------
        # Use Best‑Effort QoS for laser scans (matches Gazebo)
        self.create_subscription(
            LaserScan,
            'scan',                     # relative topic – will be remapped by launch if needed
            self.scan_callback,
            qos_profile_sensor_data,
        )

        # ----- Timer ---------------------------------------------------------
        # Use the node's clock (will be simulation clock when use_sim_time=True)
        self.create_timer(0.2, self.control_logic, clock=self.get_clock())

        # ----- Logging -------------------------------------------------------
        self.get_logger().info(
            f'Random Walker started (namespace: {self.get_namespace()})'
        )

    # ----------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        """Detect obstacles in front of the robot."""
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Look at a 60° sector centered on the front (+/- 30°)
        sector = int(num_ranges / 6)
        min_dist = float('inf')
        for i in range(sector):
            # Positive angles (left side)
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:
                min_dist = min(min_dist, r)

            # Negative angles (right side) – wrap around
            r = msg.ranges[-1 - i]
            if msg.range_min < r < msg.range_max:
                min_dist = min(min_dist, r)

        # If something is too close, switch to TURN state
        if min_dist < self.min_obstacle_dist and self.state == 'FORWARD':
            self.state = 'TURN'
            self.turn_direction = random.choice([-1, 1])
            self.turn_start_time = self.get_clock().now().nanoseconds
            self.get_logger().info(
                f'Obstacle detected ({min_dist:.2f} m) – turning '
                f'{"left" if self.turn_direction == 1 else "right"}'
            )

    # ----------------------------------------------------------------------
    def control_logic(self):
        """Publish Twist messages based on the current state."""
        msg = Twist()

        if self.state == 'FORWARD':
            msg.linear.x = 0.2          # slow forward motion
            msg.angular.z = 0.0
        elif self.state == 'TURN':
            msg.linear.x = 0.0
            msg.angular.z = 0.5 * self.turn_direction

            # After ~1 s of turning, try to go forward again
            now = self.get_clock().now().nanoseconds
            if (now - self.turn_start_time) > 1_000_000_000:  # 1 s
                self.state = 'FORWARD'
                self.get_logger().info('Path clear – moving forward')

        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()