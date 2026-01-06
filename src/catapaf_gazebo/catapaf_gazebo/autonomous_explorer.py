#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Subscribe to scan to detect obstacles
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
            
        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
            
        # Parameters
        self.declare_parameter('min_dist', 0.8)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.7)
        
        self.min_dist = self.get_parameter('min_dist').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        
        self.get_logger().info(f'Autonomous Explorer Node Started. Obstacle threshold: {self.min_dist}m')

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_readings = len(ranges)
        
        if num_readings == 0:
            return
        
        fov_check = 60 * (math.pi / 180) # 60 degrees total
        
        front_angle = 0.0
        
        try:
            center_index = int((front_angle - msg.angle_min) / msg.angle_increment)
        except ZeroDivisionError:
             return

        half_window = int((fov_check / 2) / msg.angle_increment)
        
        front_ranges = []
        
        for i in range(center_index - half_window, center_index + half_window):
            # Handle wrapping if needed, though for standard -pi to pi it's just indices
            idx = i
            if idx < 0: idx += num_readings
            if idx >= num_readings: idx -= num_readings
            
            r = ranges[idx]
            if not math.isinf(r) and not math.isnan(r):
                front_ranges.append(r)
        
        min_front_dist = min(front_ranges) if front_ranges else float('inf')
        
        twist = Twist()
        
        if min_front_dist < self.min_dist:
            self.get_logger().info(f'Obstacle detected at {min_front_dist:.2f}m! Turning...', throttle_duration_sec=1.0)
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
        else:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    explorer = AutonomousExplorer()
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
