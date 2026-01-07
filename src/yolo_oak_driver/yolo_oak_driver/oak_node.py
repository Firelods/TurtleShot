import os, json
import numpy as np
import cv2
import depthai as dai
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String, Float32MultiArray, Header
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from ament_index_python.packages import get_package_share_directory


class OakYoloSegDriver(Node):
    def __init__(self):
        super().__init__("yolo_oak_driver")
        self.bridge = CvBridge()

        pkg = get_package_share_directory("yolo_oak_driver")
        self.blob_path = os.path.join(pkg, "models", "model.blob")
        cfg_path = os.path.join(pkg, "helpers", "config.json")

        with open(cfg_path, "r") as f:
            cfg = json.load(f)

        self.input_h = int(cfg["input_shape"][-2])
        self.input_w = int(cfg["input_shape"][-1])
        self.output0_shape = tuple(cfg["output0_shape"])
        self.output1_shape = tuple(cfg["output1_shape"])
        
        self.class_names = cfg.get("class_names", ["Red Ball", "Human", "Trashcan"])
        self.nc = len(self.class_names)
        self.nm = 32
        
        self.colors = [
            (0, 0, 255),
            (255, 0, 0),
            (0, 255, 0),
        ]
        
        self.conf_threshold = 0.25
        self.iou_threshold = 0.45
        self.mask_threshold = 0.5

        self.pub_img = self.create_publisher(Image, "/camera/image_raw", 10)
        self.pub_seg = self.create_publisher(Image, "/camera/image_segmented", 10)
        self.pub_o0 = self.create_publisher(Float32MultiArray, "/oak/nn/output0", 10)
        self.pub_o1 = self.create_publisher(Float32MultiArray, "/oak/nn/output1", 10)
        self.pub_pointcloud = self.create_publisher(PointCloud2, "/depth_camera/points", 10)

        self.pipeline, self.cam, self.nn, self.stereo = self._build_pipeline()

        self.q_rgb = self.cam.preview.createOutputQueue(maxSize=4, blocking=False)
        self.q_nn = self.nn.out.createOutputQueue(maxSize=4, blocking=False)
        self.q_depth = self.stereo.depth.createOutputQueue(maxSize=4, blocking=False)

        self.pipeline.start()

        self.frame = None

        self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info("DepthAI pipeline started")

    def _build_pipeline(self):
        p = dai.Pipeline()

        cam = p.create(dai.node.ColorCamera)
        cam.setPreviewSize(self.input_w, self.input_h)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(30)

        mono_left = p.create(dai.node.MonoCamera)
        mono_right = p.create(dai.node.MonoCamera)
        stereo = p.create(dai.node.StereoDepth)
        
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_ACCURACY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(False)
        
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        nn = p.create(dai.node.NeuralNetwork)
        nn.setBlobPath(self.blob_path)
        cam.preview.link(nn.input)

        return p, cam, nn, stereo

    def decode_detections(self, out0, out1, img_shape):
        h, w = img_shape
        pred = out0.reshape(self.output0_shape)
        protos = out1.reshape(self.output1_shape)
        
        pred = pred[0]
        protos = protos[0]
        
        boxes = pred[0:4, :]
        cls_scores = pred[4:4+self.nc, :]
        mask_coeff = pred[4+self.nc:4+self.nc+self.nm, :]
        
        cls_id = np.argmax(cls_scores, axis=0)
        scores = cls_scores[cls_id, np.arange(cls_scores.shape[1])]
        
        keep = scores > self.conf_threshold
        if not np.any(keep):
            return []
        
        boxes = boxes[:, keep]
        scores = scores[keep]
        cls_id = cls_id[keep]
        mask_coeff = mask_coeff[:, keep]
        
        cx, cy, bw, bh = boxes[0, :], boxes[1, :], boxes[2, :], boxes[3, :]
        x1 = np.clip((cx - bw / 2.0), 0, w - 1).astype(int)
        y1 = np.clip((cy - bh / 2.0), 0, h - 1).astype(int)
        x2 = np.clip((cx + bw / 2.0), 0, w - 1).astype(int)
        y2 = np.clip((cy + bh / 2.0), 0, h - 1).astype(int)
        
        detections = []
        for i in range(len(scores)):
            coeff = mask_coeff[:, i]
            mask = (coeff.reshape(1, -1) @ protos.reshape(self.nm, -1)).reshape(160, 160)
            mask = 1 / (1 + np.exp(-mask))
            
            mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_LINEAR)
            mask_bin = (mask > self.mask_threshold).astype(np.uint8)
            
            detections.append({
                'bbox': [x1[i], y1[i], x2[i], y2[i]],
                'class': int(cls_id[i]),
                'score': float(scores[i]),
                'mask': mask_bin
            })
        
        return detections

    def draw_segmentation(self, frame, detections):
        overlay = frame.copy()
        
        for det in detections:
            cls_id = det['class']
            score = det['score']
            mask = det['mask']
            x1, y1, x2, y2 = det['bbox']
            
            color = self.colors[cls_id % len(self.colors)]
            
            mask_colored = np.zeros_like(frame)
            mask_colored[mask > 0] = color
            overlay = cv2.addWeighted(overlay, 1.0, mask_colored, 0.5, 0)
            
            label = f"{self.class_names[cls_id]}: {score:.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(overlay, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1)
            cv2.putText(overlay, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.6, (255, 255, 255), 2)
        
        return overlay

    def publish_pointcloud(self, depth_frame, rgb_frame):
        h, w = depth_frame.shape
        
        fx, fy = 400, 400
        cx, cy = w/2, h/2
        
        points = []
        for v in range(0, h, 2):
            for u in range(0, w, 2):
                z = depth_frame[v, u] / 1000.0
                if z == 0 or np.isnan(z) or z > 10.0:
                    continue
                
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                
                if v < rgb_frame.shape[0] and u < rgb_frame.shape[1]:
                    b, g, r = rgb_frame[v, u]
                else:
                    r, g, b = 128, 128, 128
                
                rgb_bytes = struct.pack('BBBB', b, g, r, 255)
                rgb_float = struct.unpack('f', rgb_bytes)[0]
                
                points.append([x, y, z, rgb_float])
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "oak_rgb_camera_optical_frame"
        
        cloud = pc2.create_cloud(header, fields, points)
        self.pub_pointcloud.publish(cloud)

    def tick(self):
        rgb = self.q_rgb.tryGet()
        if rgb is not None:
            self.frame = rgb.getCvFrame()
            
            msg = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "oak_rgb_camera_optical_frame"
            self.pub_img.publish(msg)

        nn_msg = self.q_nn.tryGet()
        if nn_msg is None:
            return

        layer_names = nn_msg.getAllLayerNames()
        
        out0 = None
        out1 = None
        
        if 'output0' in layer_names:
            tensor0 = nn_msg.getTensor('output0')
            out0 = np.array(tensor0).astype(np.float32).flatten()
        
        if 'output1' in layer_names:
            tensor1 = nn_msg.getTensor('output1')
            out1 = np.array(tensor1).astype(np.float32).flatten()

        if out0 is not None:
            m = Float32MultiArray()
            m.data = out0.tolist()
            self.pub_o0.publish(m)

        if out1 is not None:
            m = Float32MultiArray()
            m.data = out1.tolist()
            self.pub_o1.publish(m)
        
        depth = self.q_depth.tryGet()
        if depth is not None and self.frame is not None:
            self.publish_pointcloud(depth.getFrame(), self.frame)
        
        if out0 is not None and out1 is not None and self.frame is not None:
            try:
                detections = self.decode_detections(out0, out1, (self.input_h, self.input_w))
                
                if detections:
                    frame_seg = self.draw_segmentation(self.frame.copy(), detections)
                    seg_msg = self.bridge.cv2_to_imgmsg(frame_seg, encoding="bgr8")
                    seg_msg.header.stamp = self.get_clock().now().to_msg()
                    seg_msg.header.frame_id = "oak_rgb_camera_optical_frame"
                    self.pub_seg.publish(seg_msg)
                else:
                    self.pub_seg.publish(self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8"))
                    
            except Exception as e:
                self.get_logger().error(f"Segmentation error: {e}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = OakYoloSegDriver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()