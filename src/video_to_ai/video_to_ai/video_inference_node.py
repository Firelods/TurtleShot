import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
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
        self.model = YOLO("yolov8n-seg.pt")
        self.model.fuse() 

        # Subscriber vidéo
        self.sub = self.create_subscription(
            Image,
            "/video",
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

        self.get_logger().info("video_inference_node started")

    def image_callback(self, msg: Image):
        # Convertir en image OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur conversion Image -> OpenCV: {e}")
            return

        # Appel du modèle IA en local
        try:
            result_dict = self.run_inference(frame)

            # Récupérer le masque
            mask = np.array(result_dict["mask"], dtype=np.uint8)

            # Resize du masque pour matcher la taille réelle du frame
            mask_resized = cv2.resize(
                mask,
                (frame.shape[1], frame.shape[0]),
                interpolation=cv2.INTER_NEAREST
            )

            # Coloriage
            colored_mask = np.zeros_like(frame)
            colored_mask[mask_resized == 1] = (0, 0, 255)

            # Overlay
            overlay = cv2.addWeighted(frame, 0.6, colored_mask, 0.4, 0)

            # Publier l'image overlay
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)

        except Exception as e:
            self.get_logger().error(f"Erreur pendant l'inférence IA: {e}")
            return

        # Publication du résultat au format JSON
        result_msg = String()
        result_msg.data = json.dumps(result_dict)
        self.result_pub.publish(result_msg)


    def run_inference(self, frame):
        results = self.model(frame)[0]

        h, w = frame.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        detections = []

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

                mask[m_resized > 0.5] = 1

                # Bbox
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                detections.append({
                    "label": label,
                    "score": score,
                    "bbox": {
                        "x_min": int(x1),
                        "y_min": int(y1),
                        "x_max": int(x2),
                        "y_max": int(y2),
                    }
                })

        return {
            "detections": detections,
            "mask": mask.tolist()
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
