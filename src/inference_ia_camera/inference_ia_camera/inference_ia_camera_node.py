import os
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String, Float32MultiArray
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
import traceback
import struct
from ament_index_python.packages import get_package_share_directory


class VideoInferenceNode(Node):
    def __init__(self):
        super().__init__("video_inference_node")
        self.bridge = CvBridge()

        pkg_dir = get_package_share_directory("video_to_ai")
        cfg_path = os.path.join(pkg_dir, "config", "yolo_seg.json")
        with open(cfg_path, "r") as f:
            cfg = json.load(f)

        self.input_h = int(cfg["input_shape"][-2])
        self.input_w = int(cfg["input_shape"][-1])
        self.out0_shape = tuple(cfg["output0_shape"])
        self.out1_shape = tuple(cfg["output1_shape"])
        self.class_names = cfg["class_names"]
        self.nc = int(cfg.get("nc", len(self.class_names)))
        self.nm = int(cfg.get("nm", 32))
        self.boxes_format = cfg.get("boxes_format", "cxcywh")
        self.boxes_normalized = bool(cfg.get("boxes_normalized", False))

        self.conf_th = 0.2
        self.iou_th = 0.45
        self.mask_th = 0.5

        self.latest_cloud = None
        self.latest_o0 = None
        self.latest_o1 = None

        self.create_subscription(Image, "/camera/image_raw", self.image_callback, qos_profile_sensor_data)

        pointcloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(PointCloud2, "/depth_camera/points", self.pointcloud_callback, pointcloud_qos)

        nn_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Float32MultiArray, "/oak/nn/output0", self.o0_callback, nn_qos)
        self.create_subscription(Float32MultiArray, "/oak/nn/output1", self.o1_callback, nn_qos)

        self.overlay_pub = self.create_publisher(Image, "/ia/segmented_image", 10)
        self.result_pub = self.create_publisher(String, "/ia/result", 10)
        self.object_cloud_pub = self.create_publisher(PointCloud2, "/ia/object_cloud", 10)

        self.get_logger().info("video_inference_node started")

    def o0_callback(self, msg: Float32MultiArray):
        self.latest_o0 = msg

    def o1_callback(self, msg: Float32MultiArray):
        self.latest_o1 = msg

    def pointcloud_callback(self, msg: PointCloud2):
        self.latest_cloud = msg

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erreur conversion Image: {e}")
            return

        if self.latest_o0 is None or self.latest_o1 is None:
            return

        try:
            result_dict, overlay = self.run_postprocess(frame, self.latest_o0, self.latest_o1, self.latest_cloud)

            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)

            out = String()
            out.data = json.dumps(result_dict)
            self.result_pub.publish(out)

        except Exception as e:
            self.get_logger().error(f"Erreur post-process: {e}")
            traceback.print_exc()

    def run_postprocess(self, frame, o0_msg, o1_msg, cloud_msg):
        h, w = frame.shape[:2]
        color_mask = np.zeros_like(frame)
        seg_mask = np.zeros((h, w), dtype=np.uint8)
        detections = []

        out0 = np.array(o0_msg.data, dtype=np.float32)
        out1 = np.array(o1_msg.data, dtype=np.float32)

        if out0.size != int(np.prod(self.out0_shape)) or out1.size != int(np.prod(self.out1_shape)):
            return {"detections": []}, frame

        out0 = out0.reshape(self.out0_shape)
        protos = out1.reshape(self.out1_shape)

        objs = self.decode_yolo_seg(out0, protos, (h, w), self.conf_th, self.iou_th, self.mask_th)

        for obj in objs:
            cls_id = obj["cls_id"]
            score = obj["score"]
            
            # Ne colorier que si le score est > 0.2
            if score <= 0.2:
                continue
                
            x1, y1, x2, y2 = obj["bbox"]
            mask_bin = obj["mask"]

            label = self.class_names[cls_id] if 0 <= cls_id < len(self.class_names) else str(cls_id)

            if cls_id == 1:
                color, seg_id = (0, 255, 255), 1
            elif cls_id == 0:
                color, seg_id = (0, 0, 255), 2
            elif cls_id == 2:
                color, seg_id = (0, 255, 0), 3
            else:
                color, seg_id = (255, 0, 255), 4

            ys, xs = np.where(mask_bin == 1)
            if len(xs) > 0:
                color_mask[ys, xs] = color
                seg_mask[ys, xs] = seg_id

            pos_3d = self.compute_median_position(mask_bin, cloud_msg, h, w)

            detections.append({
                "label": label,
                "score": float(score),
                "bbox": {"x_min": int(x1), "y_min": int(y1), "x_max": int(x2), "y_max": int(y2)},
                "position_3d": pos_3d
            })

        if cloud_msg is not None:
            self.publish_colored_cloud(cloud_msg, seg_mask, h, w)

        overlay = cv2.addWeighted(frame, 0.6, color_mask, 0.4, 0)
        return {"detections": detections}, overlay

    def sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-x))

    def nms_xyxy(self, boxes, scores, iou_th):
        if len(boxes) == 0:
            return []
        boxes = boxes.astype(np.float32)
        scores = scores.astype(np.float32)
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1 + 1.0) * (y2 - y1 + 1.0)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w_ = np.maximum(0.0, xx2 - xx1 + 1.0)
            h_ = np.maximum(0.0, yy2 - yy1 + 1.0)
            inter = w_ * h_
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
            inds = np.where(iou <= iou_th)[0]
            order = order[inds + 1]
        return keep

    def decode_yolo_seg(self, out0, protos, img_shape, conf_th, iou_th, mask_th):
        h, w = img_shape
        pred = out0[0]
        N = pred.shape[1]

        boxes = pred[0:4, :]
        cls_scores = pred[4:4 + self.nc, :]
        mask_coeff = pred[4 + self.nc:4 + self.nc + self.nm, :]

        # Pour chaque détection, prendre la classe avec le score maximum
        cls_id = np.argmax(cls_scores, axis=0)
        max_scores = np.max(cls_scores, axis=0)
        
        # Filtrer par seuil de confiance
        keep = max_scores > conf_th
        if not np.any(keep):
            return []

        boxes = boxes[:, keep]
        scores = max_scores[keep]
        cls_id = cls_id[keep]
        mask_coeff = mask_coeff[:, keep]

        if self.boxes_format == "cxcywh":
            cx, cy, bw, bh = boxes[0, :], boxes[1, :], boxes[2, :], boxes[3, :]
            if self.boxes_normalized:
                cx, bw = cx * w, bw * w
                cy, bh = cy * h, bh * h
            x1 = np.clip(cx - bw / 2.0, 0, w - 1)
            y1 = np.clip(cy - bh / 2.0, 0, h - 1)
            x2 = np.clip(cx + bw / 2.0, 0, w - 1)
            y2 = np.clip(cy + bh / 2.0, 0, h - 1)
        else:
            x1, y1, x2, y2 = boxes[0, :], boxes[1, :], boxes[2, :], boxes[3, :]
            if self.boxes_normalized:
                x1, x2 = x1 * w, x2 * w
                y1, y2 = y1 * h, y2 * h
            x1 = np.clip(x1, 0, w - 1)
            y1 = np.clip(y1, 0, h - 1)
            x2 = np.clip(x2, 0, w - 1)
            y2 = np.clip(y2, 0, h - 1)

        boxes_xyxy = np.stack([x1, y1, x2, y2], axis=1)
        keep_idx = self.nms_xyxy(boxes_xyxy, scores, iou_th)

        prot = protos[0]
        nm, mh, mw = prot.shape

        objs = []
        for i in keep_idx:
            b = boxes_xyxy[i].astype(int).tolist()
            c = int(cls_id[i])
            s = float(scores[i])

            coeff = mask_coeff[:, i]
            m = (coeff.reshape(1, -1) @ prot.reshape(nm, -1)).reshape(mh, mw)
            m = self.sigmoid(m)
            m_up = cv2.resize(m, (w, h), interpolation=cv2.INTER_LINEAR)
            mask_bin = (m_up > mask_th).astype(np.uint8)

            x1i, y1i, x2i, y2i = b
            crop = np.zeros_like(mask_bin)
            crop[y1i:y2i + 1, x1i:x2i + 1] = mask_bin[y1i:y2i + 1, x1i:x2i + 1]
            objs.append({"bbox": b, "cls_id": c, "score": s, "mask": crop})

        return objs

    def compute_median_position(self, binary_mask, cloud_msg, h_img, w_img):
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
                x, y, z = all_points[idx]
                x, y, z = float(x), float(y), float(z)
                if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                    points_3d.append([x, y, z])

            if points_3d:
                median = np.median(np.array(points_3d), axis=0)
                return {"x": float(median[0]), "y": float(median[1]), "z": float(median[2])}
        except:
            pass

        return {"x": None, "y": None, "z": None}

    def publish_colored_cloud(self, cloud_msg, segmentation_mask, h_img, w_img):
        try:
            all_points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=False))
            colored_points = []

            for v in range(h_img):
                for u in range(w_img):
                    idx = v * w_img + u
                    if idx >= len(all_points):
                        continue

                    x, y, z = all_points[idx]
                    x, y, z = float(x), float(y), float(z)
                    if np.isnan(x) or np.isnan(y) or np.isnan(z):
                        continue

                    seg_value = segmentation_mask[v, u]

                    if seg_value == 1:
                        r, g, b = 255, 255, 0
                    elif seg_value == 2:
                        r, g, b = 255, 0, 0
                    elif seg_value == 3:
                        r, g, b = 0, 255, 0
                    elif seg_value == 4:
                        r, g, b = 255, 0, 255
                    else:
                        r, g, b = 180, 180, 180

                    rgb_bytes = struct.pack('BBBB', b, g, r, 255)
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

        except Exception as e:
            self.get_logger().error(f"Erreur nuage colorisé: {e}")


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