#!/usr/bin/env python3
"""
Fake LiDAR Simulator
Simulates a 2D LiDAR sensor by ray-casting against obstacles in the scene
Publishes LaserScan messages that can be visualized in Foxglove
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
import math
import numpy as np

class FakeLidar(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        
        # LiDAR parameters
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 3.5)
        self.declare_parameter('angle_min', 0.0)
        self.declare_parameter('angle_max', 2 * math.pi)
        self.declare_parameter('num_readings', 360)
        
        self.scan_rate = self.get_parameter('scan_rate').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.num_readings = self.get_parameter('num_readings').value
        
        # TF Listener to get robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        self.timer = self.create_timer(1.0 / self.scan_rate, self.publish_scan)
        
        # Obstacles (same as before)
        self.obstacles = [
            ('wall', 0.0, 2.5, 5.0, 0.1, 'horizontal'),
            ('wall', 0.0, -2.5, 5.0, 0.1, 'horizontal'),
            ('wall', 2.5, 0.0, 5.0, 0.1, 'vertical'),
            ('wall', -2.5, 0.0, 5.0, 0.1, 'vertical'),
            ('box', 1.0, 1.0, 0.3),
            ('box', -1.0, 1.0, 0.4),
            ('cylinder', 0.5, -1.0, 0.15),
            ('cylinder', -1.5, -1.0, 0.2),
        ]
        
        self.get_logger().info('Fake LiDAR started - Tracking robot position')

    def get_robot_pose(self):
        try:
            # Get transform from odom (world) to base_scan
            # We want to know where the scanner is in the world
            t = self.tf_buffer.lookup_transform(
                'odom',
                'base_scan',
                rclpy.time.Time())
            
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            
            # Extract yaw from quaternion
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return rx, ry, yaw
        except Exception as e:
            # self.get_logger().warn(f'Could not get transform: {e}')
            return 0.0, 0.0, 0.0

    def ray_intersects_wall(self, rx, ry, ray_angle, wall):
        _, wx, wy, length, thickness, orientation = wall
        
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        
        if orientation == 'horizontal':
            wall_x_min = wx - length / 2
            wall_x_max = wx + length / 2
            wall_y_min = wy - thickness / 2
            wall_y_max = wy + thickness / 2
            
            if abs(dy) < 1e-6: return None
            
            for wall_y in [wall_y_min, wall_y_max]:
                t = (wall_y - ry) / dy
                if t > 0:
                    ix = rx + t * dx
                    if wall_x_min <= ix <= wall_x_max:
                        distance = math.sqrt((ix - rx)**2 + (wall_y - ry)**2)
                        if self.range_min <= distance <= self.range_max:
                            return distance
        else:
            wall_x_min = wx - thickness / 2
            wall_x_max = wx + thickness / 2
            wall_y_min = wy - length / 2
            wall_y_max = wy + length / 2
            
            if abs(dx) < 1e-6: return None
            
            for wall_x in [wall_x_min, wall_x_max]:
                t = (wall_x - rx) / dx
                if t > 0:
                    iy = ry + t * dy
                    if wall_y_min <= iy <= wall_y_max:
                        distance = math.sqrt((wall_x - rx)**2 + (iy - ry)**2)
                        if self.range_min <= distance <= self.range_max:
                            return distance
        return None

    def ray_intersects_box(self, rx, ry, ray_angle, box):
        _, bx, by, size = box
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        
        box_x_min = bx - size / 2
        box_x_max = bx + size / 2
        box_y_min = by - size / 2
        box_y_max = by + size / 2
        
        min_distance = None
        
        # Vertical edges
        for box_x in [box_x_min, box_x_max]:
            if abs(dx) > 1e-6:
                t = (box_x - rx) / dx
                if t > 0:
                    iy = ry + t * dy
                    if box_y_min <= iy <= box_y_max:
                        dist = math.sqrt((box_x - rx)**2 + (iy - ry)**2)
                        if self.range_min <= dist <= self.range_max:
                            if min_distance is None or dist < min_distance:
                                min_distance = dist
                                
        # Horizontal edges
        for box_y in [box_y_min, box_y_max]:
            if abs(dy) > 1e-6:
                t = (box_y - ry) / dy
                if t > 0:
                    ix = rx + t * dx
                    if box_x_min <= ix <= box_x_max:
                        dist = math.sqrt((ix - rx)**2 + (box_y - ry)**2)
                        if self.range_min <= dist <= self.range_max:
                            if min_distance is None or dist < min_distance:
                                min_distance = dist
        return min_distance

    def ray_intersects_cylinder(self, rx, ry, ray_angle, cylinder):
        _, cx, cy, radius = cylinder
        dx = math.cos(ray_angle)
        dy = math.sin(ray_angle)
        
        fx = rx - cx
        fy = ry - cy
        
        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        
        discriminant = b * b - 4 * a * c
        if discriminant < 0: return None
        
        sqrt_discriminant = math.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2 * a)
        t2 = (-b + sqrt_discriminant) / (2 * a)
        
        t = None
        if t1 > 0: t = t1
        elif t2 > 0: t = t2
        
        if t is not None:
            dist = t * math.sqrt(dx * dx + dy * dy)
            if self.range_min <= dist <= self.range_max:
                return dist
        return None

    def cast_ray(self, rx, ry, robot_yaw, relative_angle):
        # Ray angle in world frame
        global_angle = robot_yaw + relative_angle
        
        min_distance = self.range_max
        
        for obstacle in self.obstacles:
            distance = None
            if obstacle[0] == 'wall':
                distance = self.ray_intersects_wall(rx, ry, global_angle, obstacle)
            elif obstacle[0] == 'box':
                distance = self.ray_intersects_box(rx, ry, global_angle, obstacle)
            elif obstacle[0] == 'cylinder':
                distance = self.ray_intersects_cylinder(rx, ry, global_angle, obstacle)
            
            if distance is not None and distance < min_distance:
                min_distance = distance
        
        return min_distance

    def publish_scan(self):
        # Get current robot position
        rx, ry, yaw = self.get_robot_pose()
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_scan'
        
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = (self.angle_max - self.angle_min) / self.num_readings
        scan.time_increment = (1.0 / self.scan_rate) / self.num_readings
        scan.scan_time = 1.0 / self.scan_rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        ranges = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * scan.angle_increment
            # Cast ray from robot position
            distance = self.cast_ray(rx, ry, yaw, angle)
            ranges.append(distance)
        
        scan.ranges = ranges
        self.scan_pub.publish(scan)



def main(args=None):
    rclpy.init(args=args)
    node = FakeLidar()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
