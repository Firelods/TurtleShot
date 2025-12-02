#!/usr/bin/env python3
"""
Simple Robot Simulator & Random Walker
1. Subscribes to cmd_vel and updates robot position (Odometry)
2. Publishes TF (odom -> base_footprint)
3. Implements random walking behavior with obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import math
import random
import time

class SimpleRobotSimulator(Node):
    def __init__(self):
        super().__init__('simple_robot_simulator')
        
        # Parameters
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.last_time = self.get_clock().now()
        
        # Random Walker State
        self.state = 'FORWARD' # FORWARD, TURN
        self.turn_direction = 1 # 1: Left, -1: Right
        self.turn_start_time = 0
        self.min_obstacle_dist = 0.6
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Timer for simulation loop (30Hz)
        self.create_timer(1.0/30.0, self.update_physics)
        
        # Timer for random walker logic (10Hz)
        self.create_timer(1.0/10.0, self.control_logic)
        
        self.get_logger().info('Simple Robot Simulator & Random Walker Started')

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def scan_callback(self, msg):
        # Check for obstacles in front
        # Scan ranges are typically counter-clockwise from -PI to +PI
        # Front is around index 0 (if 0 is front) or middle
        
        # Assuming 0 angle is front
        # Let's check a cone of +/- 30 degrees in front
        num_ranges = len(msg.ranges)
        mid_index = 0 # Angle 0
        
        # Check front sector
        min_dist = float('inf')
        
        # Check +/- 30 degrees
        sector_size = int(num_ranges / 6) # 60 degrees total
        
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
        if min_dist < self.min_obstacle_dist:
            if self.state == 'FORWARD':
                self.state = 'TURN'
                self.turn_direction = random.choice([-1, 1])
                self.turn_start_time = self.get_clock().now().nanoseconds
                self.get_logger().info(f'Obstacle detected ({min_dist:.2f}m)! Turning...')

    def control_logic(self):
        msg = Twist()
        
        if self.state == 'FORWARD':
            msg.linear.x = 0.2  # Move forward slowly
            msg.angular.z = 0.0
        elif self.state == 'TURN':
            msg.linear.x = 0.0
            msg.angular.z = 0.5 * self.turn_direction
            
            # Turn for a random duration or until clear?
            # For simplicity, let's turn for at least 1 second
            current_time = self.get_clock().now().nanoseconds
            if (current_time - self.turn_start_time) > 1e9: # 1 second
                # Try to go forward again, scan callback will override if still blocked
                self.state = 'FORWARD'
        
        self.cmd_vel_pub.publish(msg)

    def update_physics(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Update pose
        self.theta += self.wz * dt
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish Transform (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Quaternion from yaw
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
