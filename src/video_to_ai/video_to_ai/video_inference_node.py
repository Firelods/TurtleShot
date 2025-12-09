import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
import struct
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
from ultralytics import YOLO

class VideoInferenceNode(Node):
    def __init__(self):
        super().__init__("video_inference_node")

        self.bridge = CvBridge()

        # Chargement du modèle
        self.model = YOLO("yolo_best.pt")
        self.model.fuse() 

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


        # Publisher PointCloud2 coloré
        self.colored_pc_pub = self.create_publisher(
            PointCloud2,
            "/ia/colored_points",
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
        
        if self.latest_cloud is None:
            self.get_logger().warn("Aucun nuage de points reçu pour l'instant, pas de 3D.")
            cloud_msg = None
        else:
            cloud_msg = self.latest_cloud

        # Appel du modèle IA en local
        try:
            result_dict, overlay, color_mask  = self.run_inference(frame, cloud_msg)


            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)

            if cloud_msg is not None:
                colored_cloud = self.create_colored_pointcloud(cloud_msg, color_mask)
                if colored_cloud is not None:
                    self.colored_pc_pub.publish(colored_cloud)

        except Exception as e:
            self.get_logger().error(f"Erreur pendant l'inférence IA: {e}")
            return

        # Publication du résultat au format JSON
        result_msg = String()
        result_msg.data = json.dumps(result_dict)
        self.result_pub.publish(result_msg)


    def pointcloud_callback(self, msg: PointCloud2):
        # On stocke le dernier nuage de points reçu
        self.latest_cloud = msg

    def run_inference(self, frame, cloud_msg):
        results = self.model(frame)[0]

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

                color_mask[mask_bin == 1] = color
                
                # Bbox
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                pos_3d = self.projection_3D(
                    (x1, y1, x2, y2),
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

        return result_dict, overlay, color_mask
    

    def projection_3D(self, box, cloud_msg, h_img, w_img):
        if cloud_msg is None:
            return {"x": None, "y": None, "z": None}

        h_pc = cloud_msg.height
        w_pc = cloud_msg.width

        if h_pc != h_img or w_pc != w_img:
            self.get_logger().warn(
                f"Taille pointcloud ({h_pc},{w_pc}) != image ({h_img},{w_img}), pas de 3D."
            )
            return {"x": None, "y": None, "z": None}

        # centre de la bbox
        x1, y1, x2, y2 = box
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        cx = max(0, min(cx, w_pc - 1))
        cy = max(0, min(cy, h_pc - 1))

        # Un seul point dans le nuage
        point_iter = pc2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=False,
            uvs=[(cx, cy)]
        )

        try:
            x, y, z = next(point_iter)
        except StopIteration:
            return {"x": None, "y": None, "z": None}

        if np.isnan(z) or z == 0.0:
            return {"x": None, "y": None, "z": None}

        return {"x": float(x), "y": float(y), "z": float(z)}


    def create_colored_pointcloud(self, cloud_msg, color_mask):
        h_pc = cloud_msg.height
        w_pc = cloud_msg.width

        h_img, w_img, _ = color_mask.shape
        if h_pc != h_img or w_pc != w_img:
            self.get_logger().warn(
                f"color_mask ({h_img},{w_img}) != pointcloud ({h_pc},{w_pc}), skip colored cloud."
            )
            return None

        # Lire tous les points du cloud
        points = list(pc2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=False
        ))

        def pack_rgb(r, g, b):
            rgb_uint32 = (r << 16) | (g << 8) | b
            return struct.unpack('f', struct.pack('I', rgb_uint32))[0]

        new_points = []
        idx = 0
        for y in range(h_pc):
            for x in range(w_pc):
                x3d, y3d, z3d = points[idx]
                b, g, r = color_mask[y, x]
                rgb_float = pack_rgb(int(r), int(g), int(b))
                new_points.append((x3d, y3d, z3d, rgb_float))
                idx += 1

        fields = [
            PointField("x", 0,  PointField.FLOAT32, 1),
            PointField("y", 4,  PointField.FLOAT32, 1),
            PointField("z", 8,  PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]

        colored_cloud = pc2.create_cloud(
            cloud_msg.header,
            fields,
            new_points
        )

        return colored_cloud

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
