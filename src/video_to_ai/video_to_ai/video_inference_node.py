import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
import traceback
from ultralytics import YOLO

class VideoInferenceNode(Node):
    def __init__(self):
        super().__init__("video_inference_node")

        self.bridge = CvBridge()

        model_path = os.path.join(get_package_share_directory("video_to_ai"), "models", "yolo_best.pt")
        self.get_logger().info(f"Chargement du modèle : {model_path}")
        self.model = YOLO(model_path)
        
        self.latest_cloud = None

        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            qos_profile_sensor_data
        )

        self.overlay_pub = self.create_publisher(
            Image,
            "/ia/segmented_image",
            10
        )

        self.result_pub = self.create_publisher(
            String,
            "/ia/result",
            10
        )
 
        # QoS pour PointCloud2 - doit matcher le publisher (RELIABLE)
        pointcloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Changé en RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10,  # Même depth que le publisher
            durability=DurabilityPolicy.VOLATILE
        )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            "/depth_camera/points",
            self.pointcloud_callback,
            pointcloud_qos
        )

        self.object_cloud_pub = self.create_publisher(
            PointCloud2,
            "/ia/object_cloud",
            10
        )

        self.get_logger().info("video_inference_node started")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur conversion Image -> OpenCV: {e}")
            return
        
        cloud_msg = self.latest_cloud

        try:
            result_dict, overlay = self.run_inference(frame, cloud_msg)

            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)

            result_msg = String()
            result_msg.data = json.dumps(result_dict)
            self.result_pub.publish(result_msg)

            if cloud_msg is None:
                self.get_logger().warn("Pas de nuage de points (projection 3D désactivée)", throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().error(f"Erreur pendant l'inférence IA: {e}")
            traceback.print_exc() 
            return


    def pointcloud_callback(self, msg: PointCloud2):
        self.latest_cloud = msg
        self.get_logger().info(f"Nuage reçu: {msg.width}x{msg.height} points", throttle_duration_sec=5.0)

    def run_inference(self, frame, cloud_msg):
        results = self.model(frame, verbose=False)[0]

        h, w = frame.shape[:2]
        detections = []

        color_mask = np.zeros_like(frame)

        if results.masks is not None:
            masks = results.masks.data 
            boxes = results.boxes     

            for m, box in zip(masks, boxes):
                cls_id = int(box.cls.item())
                score = float(box.conf.item())
                label = self.model.names[cls_id]

                m_np = m.cpu().numpy()

                m_resized = cv2.resize(
                    m_np,
                    (w, h),
                    interpolation=cv2.INTER_NEAREST
                )

                mask_bin = (m_resized > 0.5).astype(np.uint8)

                if label == "person":
                    color = (0, 0, 255)
                elif label == "ball":
                    color = (255, 0, 0)
                else:
                    color = (0, 255, 0)

                indices = np.where(mask_bin == 1)
                if len(indices[0]) > 0:
                    color_mask[indices[0], indices[1]] = color
                
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                pos_3d, object_points = self.projection_3D_mask(
                    mask_bin,
                    cloud_msg,
                    h,
                    w
                )

                if object_points is not None and len(object_points) > 0:
                    self.publish_object_cloud(object_points, cloud_msg.header)
                else:
                    if cloud_msg is not None:
                        self.get_logger().warn(f"Projection échouée pour {label} (pas de points valides)", throttle_duration_sec=2.0)

                detections.append({
                    "label": label,
                    "score": score,
                    "bbox": {
                        "x_min": int(x1),
                        "y_min": int(y1),
                        "x_max": int(x2),
                        "y_max": int(y2),
                    },
                    "position_3d": pos_3d
                })

        
        overlay = cv2.addWeighted(frame, 0.6, color_mask, 0.4, 0)

        result_dict = {
            "detections": detections,
        }

        return result_dict, overlay
    
    def publish_object_cloud(self, points, header):
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.object_cloud_pub.publish(cloud_msg)

    def projection_3D_mask(self, binary_mask, cloud_msg, h_img, w_img):
        """Projection COMPLÈTE : tous les points du masque sans filtrage"""
        if cloud_msg is None:
            return {"x": None, "y": None, "z": None}, None

        ys, xs = np.where(binary_mask == 1)
        
        if len(xs) == 0:
            return {"x": None, "y": None, "z": None}, None
        
        points_3d = []
        
        try:
            # Vérification dimensions
            if cloud_msg.height > 1:
                if cloud_msg.height != h_img or cloud_msg.width != w_img:
                    self.get_logger().warn(
                        f"Mismatch: Image {w_img}x{h_img} vs Cloud {cloud_msg.width}x{cloud_msg.height}", 
                        throttle_duration_sec=2.0
                    )
                    return {"x": None, "y": None, "z": None}, None
            
            # Lire TOUT le nuage
            all_points = list(pc2.read_points(
                cloud_msg, 
                field_names=("x", "y", "z"), 
                skip_nans=False
            ))
            
            # ✅ Parcourir TOUS les pixels du masque (pas de step)
            for u, v in zip(xs, ys):
                idx = int(v) * cloud_msg.width + int(u)
                
                if idx >= len(all_points):
                    continue
                
                point = all_points[idx]
                
                # Unpacking
                try:
                    x, y, z = point
                    x, y, z = float(x), float(y), float(z)
                except (ValueError, TypeError):
                    try:
                        coords = list(point)
                        x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                    except (IndexError, TypeError, ValueError):
                        continue
                
                # ✅ Garder seulement les points valides (pas NaN)
                if np.isnan(x) or np.isnan(y) or np.isnan(z):
                    continue
                
                # ✅ Pas de filtrage de distance, on garde tout
                points_3d.append([x, y, z])
        
        except Exception as e:
            self.get_logger().error(f"Erreur lecture PointCloud2: {e}")
            traceback.print_exc()
            return {"x": None, "y": None, "z": None}, None
        
        if len(points_3d) == 0:
            return {"x": None, "y": None, "z": None}, None
        
        # Position médiane pour le JSON
        points_np = np.array(points_3d)
        median = np.median(points_np, axis=0)
        
        self.get_logger().info(
            f"Projection: {len(points_3d)} points extraits du masque",
            throttle_duration_sec=2.0
        )
        
        return {
            "x": float(median[0]),
            "y": float(median[1]),
            "z": float(median[2])
        }, points_3d

    
def main(args=None):
    rclpy.init(args=args)
    node = VideoInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()