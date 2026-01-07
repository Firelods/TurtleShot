#!/usr/bin/env python3
"""
Simple node that converts odometry messages to TF transforms.
Subscribes to /odom and publishes the transform from odom to base_footprint.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('Odometry to TF node started')

    def odom_callback(self, msg: Odometry):
        """Convert odometry message to TF transform"""
        t = TransformStamped()

        # Header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # 'odom'
        t.child_frame_id = msg.child_frame_id     # 'base_footprint'

        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
