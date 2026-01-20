# Inference IA Camera

This package provides an AI inference node for processing camera images using YOLO segmentation models.

## Overview

The `inference_ia_camera` package receives pre-computed neural network outputs and performs post-processing to detect and segment objects in camera images. It works in conjunction with camera drivers that provide raw inference data.

## Features

- YOLO segmentation model post-processing
- Object detection with bounding boxes
- Instance segmentation masks
- 3D pose estimation using depth data
- Configurable confidence and IoU thresholds

## Dependencies

- `rclpy` - ROS 2 Python client library
- `sensor_msgs` - Image and PointCloud2 messages
- `std_msgs` - Standard message types
- `cv_bridge` - OpenCV-ROS bridge
- `numpy`, `opencv-python` - Image processing

## Building

```bash
colcon build --packages-select inference_ia_camera
source install/setup.bash
```

## Usage

### Run the Inference Node

```bash
ros2 run inference_ia_camera inference_ia_camera_node --ros-args -p use_sim_time:=true
```

## Configuration

The node reads configuration from `config/yolo_seg.json`:

```json
{
    "input_shape": [1, 3, 640, 640],
    "output0_shape": [1, 37, 8400],
    "output1_shape": [1, 32, 160, 160],
    "class_names": ["Red Ball", "Human", "Trashcan"],
    "nc": 3,
    "nm": 32
}
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `conf_th` | `0.2` | Confidence threshold for detection |
| `iou_th` | `0.45` | IoU threshold for NMS |
| `mask_th` | `0.5` | Mask threshold for segmentation |

## Topics

### Subscribed
- `/camera/image_raw` (`sensor_msgs/Image`) - RGB camera images
- `/depth_camera/points` (`sensor_msgs/PointCloud2`) - Depth point cloud
- `/oak/nn/output0` (`std_msgs/Float32MultiArray`) - NN detection output
- `/oak/nn/output1` (`std_msgs/Float32MultiArray`) - NN segmentation output

### Published
- Detection results and segmented images (package-specific topics)

## Detected Classes

The default model detects:
- **Red Ball** - Target objects for catapult
- **Human** - Humans in the scene (safety)
- **Trashcan** - Trash bins for collection

## Models

Place YOLO model files in the `models/` directory:
- `yolo.pt` - PyTorch model for reference
- `yolo_best.pt` - Fine-tuned model

## Related Packages

- `video_to_ai` - Video-based inference pipeline
- `yolo_oak_driver` - OAK-D camera driver with on-device inference

## License

Apache License 2.0
