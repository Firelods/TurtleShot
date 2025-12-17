#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ImuTfPublisher(Node):
    def __init__(self):
        super().__init__('imu_tf_publisher')
        self.subscription = self.create_subscription(
            Imu,
            '/oak/imu/data',
            self.imu_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('IMU TF Publisher Started')

    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_footprint'

        # Use the orientation from the IMU message
        # Note: This assumes the IMU frame and Base frame are aligned enough for a demo,
        # or that the user accepts the base frame rotating as the IMU does.
        # Ideally we would correct for the base->imu transform.
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0 # Keep it 1m up
        t.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
