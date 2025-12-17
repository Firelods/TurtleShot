import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge

import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
import struct


class FakeOakPublisher(Node):
    def __init__(self):
        super().__init__("fake_oak_publisher")

        # Paramètre : chemin de l'image fake
        self.declare_parameter("image_path", "test_image.jpg")
        image_path = self.get_parameter("image_path").get_parameter_value().string_value

        self.bridge = CvBridge()

        # Charger l'image
        frame = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError(f"❌ Impossible de lire l'image : {image_path}")

        self.frame = frame
        self.h, self.w = frame.shape[:2]

        self.get_logger().info(f"✅ Image chargée : {image_path} ({self.w}x{self.h})")

        # Préparer un nuage de points organisé
        self.pointcloud_msg = self._create_fake_pointcloud()

        # Publishers
        self.img_pub = self.create_publisher(Image, "/oak/rgb/image_raw", 10)
        self.pc_pub = self.create_publisher(PointCloud2, "/oak/points", 10)

        # Timer 30 FPS
        self.timer = self.create_timer(0.033, self.publish_data)


    # --------------------------------------------------------
    # Création d’un nuage de points synthétique organisé
    # --------------------------------------------------------
    def _create_fake_pointcloud(self):
        points = []
        Z = 1.0  # profondeur plate 1m

        fx = fy = max(self.w, self.h)
        cx = self.w / 2.0
        cy = self.h / 2.0

        for v in range(self.h):
            for u in range(self.w):
                x = (u - cx) / fx * Z
                y = (v - cy) / fy * Z
                z = Z
                points.append((x, y, z))

        header = Header()
        header.frame_id = "oak_frame"

        cloud = pc2.create_cloud_xyz32(header, points)
        cloud.height = self.h
        cloud.width = self.w
        cloud.is_dense = True
        cloud.row_step = cloud.point_step * cloud.width

        return cloud


    # --------------------------------------------------------
    # Publication continue
    # --------------------------------------------------------
    def publish_data(self):
        stamp = self.get_clock().now().to_msg()

        # Publier l'image
        img_msg = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = "oak_frame"
        self.img_pub.publish(img_msg)

        # Publier le nuage de points
        cloud = self.pointcloud_msg
        cloud.header.stamp = stamp
        self.pc_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOakPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
