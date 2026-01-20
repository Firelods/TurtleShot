# YOLO OAK-D Driver

This package provides a ROS 2 driver for the OAK-D camera with on-device YOLO segmentation inference using DepthAI.

## Overview

The `yolo_oak_driver` package interfaces with OAK-D cameras (OAK-D, OAK-D Pro, OAK-D Lite) and runs YOLO segmentation models directly on the camera's VPU (Visual Processing Unit) for real-time object detection and segmentation.

## Features

- OAK-D camera integration via DepthAI
- On-device YOLO segmentation inference
- RGB image publishing
- Depth point cloud generation
- Neural network output streaming
- Real-time segmented image overlay

## Dependencies

- `rclpy` - ROS 2 Python client library
- `sensor_msgs` - Image and PointCloud2 messages
- `std_msgs` - Standard message types
- `cv_bridge` - OpenCV-ROS bridge
- `depthai` - DepthAI SDK for OAK cameras
- `numpy`, `opencv-python` - Image processing

## Hardware Requirements

- OAK-D, OAK-D Pro, or OAK-D Lite camera
- USB 3.0 connection (USB-C)

## Building

```bash
colcon build --packages-select yolo_oak_driver
source install/setup.bash
```

## Installation

Install DepthAI dependencies:

```bash
pip install depthai
```

For udev rules (Linux):
```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Usage

### Run the OAK Driver

```bash
ros2 run yolo_oak_driver oak_node
```

## Configuration

Configuration is stored in `helpers/config.json`:

```json
{
    "input_shape": [1, 3, 640, 640],
    "output0_shape": [1, 37, 8400],
    "output1_shape": [1, 32, 160, 160],
    "class_names": ["Red Ball", "Human", "Trashcan"]
}
```

## Models

Place the compiled blob model in `models/`:
- `model_V2.blob` - Compiled YOLO model for OAK VPU

To convert a PyTorch model to blob format, use the DepthAI Model Zoo tools or blobconverter.

## Topics

### Published
- `/camera/image_raw` (`sensor_msgs/Image`) - RGB camera image
- `/camera/image_segmented` (`sensor_msgs/Image`) - Segmented overlay image
- `/depth_camera/points` (`sensor_msgs/PointCloud2`) - Depth point cloud
- `/oak/nn/output0` (`std_msgs/Float32MultiArray`) - Detection output (boxes, scores)
- `/oak/nn/output1` (`std_msgs/Float32MultiArray`) - Segmentation masks

## Detection Classes

Default model classes:
| Index | Class | Color (BGR) |
|-------|-------|-------------|
| 0 | Red Ball | (0, 0, 255) |
| 1 | Human | (255, 0, 0) |
| 2 | Trashcan | (0, 255, 0) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `conf_threshold` | `0.2` | Detection confidence threshold |
| `iou_threshold` | `0.45` | NMS IoU threshold |
| `mask_threshold` | `0.5` | Segmentation mask threshold |

## Pipeline Architecture

```
OAK-D Camera
├── Color Camera (RGB)
│   └── Preview Output → /camera/image_raw
├── Neural Network (VPU)
│   ├── Output0 → /oak/nn/output0 (detections)
│   └── Output1 → /oak/nn/output1 (masks)
└── Stereo Depth
    └── Depth Output → /depth_camera/points
```

## Troubleshooting

### Camera not detected
- Check USB 3.0 connection
- Verify udev rules are installed
- Try `lsusb` to see if device is recognized

### Low FPS
- Reduce input resolution in model
- Check USB bandwidth (USB 3.0 required)
- Reduce number of output streams

### Model loading fails
- Verify blob file exists in `models/`
- Check blob is compiled for correct VPU version
- Ensure input/output shapes match config

## Related Packages

- `inference_ia_camera` - Alternative inference pipeline
- `video_to_ai` - Video-based inference for simulation

## License

Apache License 2.0
