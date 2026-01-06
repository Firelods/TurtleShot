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
import struct

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
 
        pointcloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
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
        
        # Masque de segmentation
        segmentation_mask = np.zeros((h, w), dtype=np.uint8)

        if results.masks is not None:
            masks = results.masks.data 
            boxes = results.boxes     

            for m, box in zip(masks, boxes):
                cls_id = int(box.cls.item())
                score = float(box.conf.item())
                label = self.model.names[cls_id]

                m_np = m.cpu().numpy()
                m_resized = cv2.resize(m_np, (w, h), interpolation=cv2.INTER_NEAREST)
                mask_bin = (m_resized > 0.5).astype(np.uint8)

                # Définir les couleurs selon l'index de classe
                if cls_id == 1:  # Human
                    color = (0, 255, 255)  # Jaune en BGR
                    seg_id = 1
                elif cls_id == 0:  # Red Ball
                    color = (0, 0, 255)  # Rouge en BGR
                    seg_id = 2
                elif cls_id == 2:  # Trashcan
                    color = (0, 255, 0)  # Vert en BGR
                    seg_id = 3
                else:
                    color = (255, 0, 255)  # Magenta en BGR
                    seg_id = 4

                indices = np.where(mask_bin == 1)
                if len(indices[0]) > 0:
                    color_mask[indices[0], indices[1]] = color
                    segmentation_mask[indices[0], indices[1]] = seg_id
                
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                # Calculer la position 3D médiane
                pos_3d = self.compute_median_position(mask_bin, cloud_msg, h, w)

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

        if cloud_msg is not None:
            self.publish_colored_cloud(cloud_msg, segmentation_mask, h, w)
        
        overlay = cv2.addWeighted(frame, 0.6, color_mask, 0.4, 0)

        result_dict = {"detections": detections}

        return result_dict, overlay
    
    def compute_median_position(self, binary_mask, cloud_msg, h_img, w_img):
        """Calcule la position médiane d'un objet segmenté"""
        if cloud_msg is None:
            return {"x": None, "y": None, "z": None}

        ys, xs = np.where(binary_mask == 1)
        if len(xs) == 0:
            return {"x": None, "y": None, "z": None}
        
        try:
            all_points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=False))
            
            points_3d = []
            step = max(1, len(xs) // 500)
            
            for i in range(0, len(xs), step):
                idx = int(ys[i]) * w_img + int(xs[i])
                if idx >= len(all_points):
                    continue
                
                point = all_points[idx]
                try:
                    x, y, z = point
                    x, y, z = float(x), float(y), float(z)
                    if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        points_3d.append([x, y, z])
                except:
                    continue
            
            if len(points_3d) > 0:
                median = np.median(np.array(points_3d), axis=0)
                return {"x": float(median[0]), "y": float(median[1]), "z": float(median[2])}
        except:
            pass
        
        return {"x": None, "y": None, "z": None}

    def publish_colored_cloud(self, cloud_msg, segmentation_mask, h_img, w_img):
        """Publie TOUT le nuage de points avec couleurs RGB selon segmentation"""
        try:
            all_points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=False))
            
            colored_points = []
            
            # Parcourir TOUS les pixels
            for v in range(h_img):
                for u in range(w_img):
                    idx = v * w_img + u
                    
                    if idx >= len(all_points):
                        continue
                    
                    point = all_points[idx]
                    
                    try:
                        x, y, z = point
                        x, y, z = float(x), float(y), float(z)
                    except:
                        continue
                    
                    if np.isnan(x) or np.isnan(y) or np.isnan(z):
                        continue
                    
                    # Couleur selon segmentation
                    seg_value = segmentation_mask[v, u]
                    
                    if seg_value == 1:  # Person
                        r, g, b = 255, 255, 0 
                    elif seg_value == 2:  # Ball
                        r, g, b = 255, 0, 0  
                    elif seg_value == 3: # Trash can
                        r, g, b = 0, 255, 0
                    elif seg_value == 4: 
                        r, g, b = 255, 0, 255  
                    else:
                        r, g, b = 180, 180, 180  # en gris
                    
                    rgb_bytes = struct.pack('BBBB', b, g, r, 255)  # BGRA little-endian
                    rgb_float = struct.unpack('f', rgb_bytes)[0]
                    
                    colored_points.append([x, y, z, rgb_float])
        
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            cloud_colored = pc2.create_cloud(cloud_msg.header, fields, colored_points)
            self.object_cloud_pub.publish(cloud_colored)
            
            self.get_logger().info(
                f"Nuage colorisé: {len(colored_points)} points "
                f"(Human={np.sum(segmentation_mask==1)}, Red Ball={np.sum(segmentation_mask==2)}, "
                f"Trashcan={np.sum(segmentation_mask==3)}, Fond={np.sum(segmentation_mask==0)})",
                throttle_duration_sec=5.0
            )
        
        except Exception as e:
            self.get_logger().error(f"Erreur création nuage colorisé: {e}")
            traceback.print_exc()


    
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