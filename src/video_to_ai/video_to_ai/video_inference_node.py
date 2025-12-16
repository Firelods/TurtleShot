import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
import struct
import traceback
from ultralytics import YOLO

class VideoInferenceNode(Node):
    def __init__(self):
        super().__init__("video_inference_node")

        self.bridge = CvBridge()

        # Chargement du modèle
        model_path = os.path.join(get_package_share_directory("video_to_ai"), "models", "yolo_best.pt")
        self.get_logger().info(f"Chargement du modèle : {model_path}")
        self.model = YOLO(model_path)
        # self.model.fuse() # Optionnel, accélère parfois l'inférence

        self.latest_cloud = None

        # Subscriber vidéo
        self.sub = self.create_subscription(
            Image,
            "/oak/rgb/image_raw",
            self.image_callback,
            10
        )

        # Publisher IA de l'image segmentée
        self.overlay_pub = self.create_publisher(
            Image,
            "/ia/segmented_image",
            10
        )

        # Publisher IA du résultat JSON
        self.result_pub = self.create_publisher(
            String,
            "/ia/result",
            10
        )

        # Subscriber PointCloud2
        self.pc_sub = self.create_subscription(
            PointCloud2,
            "/oak/points",
            self.pointcloud_callback,
            10
        )

        self.get_logger().info("video_inference_node started")

    def image_callback(self, msg: Image):
        # Convertir en image OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur conversion Image -> OpenCV: {e}")
            return
        
        # Utilisation du dernier nuage reçu
        cloud_msg = self.latest_cloud
        if cloud_msg is None:
            self.get_logger().debug("Pas de nuage de points reçu pour l'instant.")

        # Appel du modèle IA
        try:
            result_dict, overlay = self.run_inference(frame, cloud_msg)

            # Publication de l'overlay 2D
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)

            # Publication du résultat JSON
            result_msg = String()
            result_msg.data = json.dumps(result_dict)
            self.result_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f"Erreur pendant l'inférence IA: {e}")
            traceback.print_exc() 
            return


    def pointcloud_callback(self, msg: PointCloud2):
        # On stocke le dernier nuage de points reçu
        self.latest_cloud = msg

    def run_inference(self, frame, cloud_msg):
        # verbose=False pour alléger la console
        results = self.model(frame, verbose=False)[0]

        h, w = frame.shape[:2]
        detections = []

        color_mask = np.zeros_like(frame)

        if results.masks is not None:
            masks = results.masks.data 
            boxes = results.boxes     

            for m, box in zip(masks, boxes):
                # Classe et score
                cls_id = int(box.cls.item())
                score = float(box.conf.item())
                label = self.model.names[cls_id]

                # Masque
                m_np = m.cpu().numpy()

                # Resize du masque à la taille du frame si besoin
                m_resized = cv2.resize(
                    m_np,
                    (w, h),
                    interpolation=cv2.INTER_NEAREST
                )

                mask_bin = (m_resized > 0.5).astype(np.uint8)

                if label == "person":
                    color = (0, 0, 255)      # rouge 
                elif label == "ball":
                    color = (255, 0, 0)      # bleu
                else:
                    color = (0, 255, 0)      # vert

                indices = np.where(mask_bin == 1)
                # Sécurité pour l'assignation numpy
                if len(indices[0]) > 0:
                    color_mask[indices[0], indices[1]] = color
                
                # Bbox
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                # Projection 3D en utilisant le masque de segmentation
                pos_3d = self.projection_3D_mask(
                    mask_bin,
                    cloud_msg,
                    h,
                    w
                )

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
    

    def projection_3D_mask(self, binary_mask, cloud_msg, h_img, w_img):
        """
        Calcule la position moyenne des points 3D correspondant au masque binaire.
        """
        if cloud_msg is None:
            return {"x": None, "y": None, "z": None}

        h_pc = cloud_msg.height
        w_pc = cloud_msg.width

        if h_pc != h_img or w_pc != w_img:
            # Si les résolutions diffèrent, on ne peut pas mapper directement pixel->point
            return {"x": None, "y": None, "z": None}

        # Récupération des offsets
        x_off = -1; y_off = -1; z_off = -1
        for f in cloud_msg.fields:
            if f.name == "x": x_off = f.offset
            elif f.name == "y": y_off = f.offset
            elif f.name == "z": z_off = f.offset
        
        if x_off == -1: return {"x": None, "y": None, "z": None}

        ys, xs = np.where(binary_mask == 1)

        if len(xs) == 0:
            return {"x": None, "y": None, "z": None}

        step = 1
        if len(xs) > 1000:
            step = 10
        
        valid_points = []
        
        fmt = '>' if cloud_msg.is_bigendian else '<'
        fmt += 'f'
        point_step = cloud_msg.point_step
        row_step = cloud_msg.row_step
        data = cloud_msg.data

        for i in range(0, len(xs), step):
            u, v = xs[i], ys[i]
            
            # Calcul de l'offset mémoire
            offset = v * row_step + u * point_step
            
            try:
                z = struct.unpack_from(fmt, data, offset + z_off)[0]
                
                # Filtrage basique des points invalides
                if not np.isnan(z) and z > 0.1 and z < 15.0:
                    x = struct.unpack_from(fmt, data, offset + x_off)[0]
                    y = struct.unpack_from(fmt, data, offset + y_off)[0]
                    valid_points.append((x, y, z))
            except:
                continue

        if not valid_points:
            return {"x": None, "y": None, "z": None}

        points_np = np.array(valid_points)
        median_xyz = np.median(points_np, axis=0)

        return {
            "x": float(median_xyz[0]), 
            "y": float(median_xyz[1]), 
            "z": float(median_xyz[2])
        }


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