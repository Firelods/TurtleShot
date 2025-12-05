#!/usr/bin/env python3
"""
Random Walker for Gazebo Simulation
Implements random walking behavior with obstacle avoidance
Works with Gazebo physics - only publishes cmd_vel commands
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
        
        # Random Walker State
        self.state = 'FORWARD'  # FORWARD, TURN
        self.turn_direction = 1  # 1: Left, -1: Right
        self.turn_start_time = 0
        self.min_obstacle_dist = 0.6
        
        # Publishers - Use relative topics
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers - Use SensorData QoS for Gazebo compatibility
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        
        # Timer for random walker logic (10Hz)
        self.create_timer(1.0/10.0, self.control_logic)
        
        self.get_logger().info('Random Walker Started')

    def scan_callback(self, msg):
        """Check for obstacles in front and update state"""
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Check front sector (+/- 30 degrees)
        min_dist = float('inf')
        sector_size = int(num_ranges / 6)  # 60 degrees total
        
        for i in range(sector_size):
            # Check positive angles (left)
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:
                min_dist = min(min_dist, r)
                
            # Check negative angles (right) - wrap around
            r = msg.ranges[num_ranges - 1 - i]
            if msg.range_min < r < msg.range_max:
                min_dist = min(min_dist, r)
        
        # Update state based on obstacles
        old_state = self.state
        if min_dist < self.min_obstacle_dist:
            if self.state == 'FORWARD':
                self.state = 'TURN'
                self.turn_direction = random.choice([-1, 1])
                self.turn_start_time = self.get_clock().now().nanoseconds
                self.get_logger().info(f'Obstacle detected ({min_dist:.2f}m)! Turning...')
        
    def control_logic(self):
        """Generate velocity commands based on current state"""
        msg = Twist()
        
        if self.state == 'FORWARD':
            msg.linear.x = 0.2  # Move forward slowly
            msg.angular.z = 0.0
        elif self.state == 'TURN':
            msg.linear.x = 0.0
            msg.angular.z = 0.5 * self.turn_direction
            
            # Turn for at least 1 second
            current_time = self.get_clock().now().nanoseconds
            if (current_time - self.turn_start_time) > 1e9:  # 1 second
                # Try to go forward again
                self.state = 'FORWARD'
                self.get_logger().info('Path clear, moving forward')
        
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
