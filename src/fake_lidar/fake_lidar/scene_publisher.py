#!/usr/bin/env python3
"""
3D Scene Publisher for Foxglove Visualization
Publishes visualization markers to create a 3D environment/map
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/scene_markers', 
            10
        )
        
        # Publish the scene at 1 Hz (static scene)
        self.timer = self.create_timer(1.0, self.publish_scene)
        
        self.get_logger().info('Scene Publisher started - publishing 3D environment')
    
    def create_floor(self, marker_id):
        """Create a floor plane"""
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scene"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position (centered at origin, below the robot)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = -0.001  # Just below robot
        marker.pose.orientation.w = 1.0
        
        # Size (large flat plane)
        marker.scale.x = 5.0  # 5m x 5m floor
        marker.scale.y = 5.0
        marker.scale.z = 0.01  # Very thin
        
        # Color (gray floor)
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0  # Permanent
        
        return marker
    
    def create_wall(self, marker_id, x, y, length, orientation='vertical'):
        """Create a wall"""
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scene"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5  # Wall height center
        marker.pose.orientation.w = 1.0
        
        # Size
        if orientation == 'vertical':
            marker.scale.x = 0.1  # Wall thickness
            marker.scale.y = length
        else:  # horizontal
            marker.scale.x = length
            marker.scale.y = 0.1
        marker.scale.z = 1.0  # Wall height
        
        # Color (beige walls)
        marker.color.r = 0.9
        marker.color.g = 0.85
        marker.color.b = 0.7
        marker.color.a = 1.0
        
        marker.lifetime.sec = 0
        
        return marker
    
    def create_obstacle(self, marker_id, x, y, size, color):
        """Create a box obstacle"""
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scene"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = size / 2.0  # Half height
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        
        # Color
        marker.color = color
        
        marker.lifetime.sec = 0
        
        return marker
    
    def create_cylinder_obstacle(self, marker_id, x, y, radius, height, color):
        """Create a cylindrical obstacle (like a pillar)"""
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scene"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = height / 2.0
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = height
        
        # Color
        marker.color = color
        
        marker.lifetime.sec = 0
        
        return marker
    
    def create_grid(self, marker_id):
        """Create a grid on the floor for reference"""
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scene"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        marker.pose.orientation.w = 1.0
        
        # Grid parameters
        grid_size = 5.0  # 5m x 5m
        grid_spacing = 0.5  # 50cm spacing
        
        # Create grid lines
        for i in range(int(grid_size / grid_spacing) + 1):
            offset = -grid_size / 2 + i * grid_spacing
            
            # Vertical lines
            p1 = Point()
            p1.x = offset
            p1.y = -grid_size / 2
            p1.z = 0.001  # Just above floor
            
            p2 = Point()
            p2.x = offset
            p2.y = grid_size / 2
            p2.z = 0.001
            
            marker.points.append(p1)
            marker.points.append(p2)
            
            # Horizontal lines
            p3 = Point()
            p3.x = -grid_size / 2
            p3.y = offset
            p3.z = 0.001
            
            p4 = Point()
            p4.x = grid_size / 2
            p4.y = offset
            p4.z = 0.001
            
            marker.points.append(p3)
            marker.points.append(p4)
        
        # Line width and color
        marker.scale.x = 0.01  # Line width
        marker.color.r = 0.3
        marker.color.g = 0.3
        marker.color.b = 0.3
        marker.color.a = 0.5
        
        marker.lifetime.sec = 0
        
        return marker
    
    def publish_scene(self):
        """Publish the complete 3D scene"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Add floor
        marker_array.markers.append(self.create_floor(marker_id))
        marker_id += 1
        
        # Add grid
        marker_array.markers.append(self.create_grid(marker_id))
        marker_id += 1
        
        # Create a simple room with walls
        # North wall
        marker_array.markers.append(
            self.create_wall(marker_id, 0.0, 2.5, 5.0, 'horizontal')
        )
        marker_id += 1
        
        # South wall
        marker_array.markers.append(
            self.create_wall(marker_id, 0.0, -2.5, 5.0, 'horizontal')
        )
        marker_id += 1
        
        # East wall
        marker_array.markers.append(
            self.create_wall(marker_id, 2.5, 0.0, 5.0, 'vertical')
        )
        marker_id += 1
        
        # West wall
        marker_array.markers.append(
            self.create_wall(marker_id, -2.5, 0.0, 5.0, 'vertical')
        )
        marker_id += 1
        
        # Add some obstacles
        # Red box
        red = ColorRGBA()
        red.r = 0.8
        red.g = 0.2
        red.b = 0.2
        red.a = 1.0
        marker_array.markers.append(
            self.create_obstacle(marker_id, 1.0, 1.0, 0.3, red)
        )
        marker_id += 1
        
        # Green box
        green = ColorRGBA()
        green.r = 0.2
        green.g = 0.8
        green.b = 0.2
        green.a = 1.0
        marker_array.markers.append(
            self.create_obstacle(marker_id, -1.0, 1.0, 0.4, green)
        )
        marker_id += 1
        
        # Blue cylinder (pillar)
        blue = ColorRGBA()
        blue.r = 0.2
        blue.g = 0.4
        blue.b = 0.8
        blue.a = 1.0
        marker_array.markers.append(
            self.create_cylinder_obstacle(marker_id, 0.5, -1.0, 0.15, 1.0, blue)
        )
        marker_id += 1
        
        # Yellow cylinder
        yellow = ColorRGBA()
        yellow.r = 0.9
        yellow.g = 0.9
        yellow.b = 0.2
        yellow.a = 1.0
        marker_array.markers.append(
            self.create_cylinder_obstacle(marker_id, -1.5, -1.0, 0.2, 0.8, yellow)
        )
        marker_id += 1
        
        # Publish the scene
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
