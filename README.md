# ros2_multicam_yolo

Real-time multi-camera object detection using YOLOv8 for ROS2 systems.

## Overview

This ROS2 package provides a robust solution for performing real-time object detection across multiple camera feeds using YOLOv8. It's designed to handle synchronized processing of multiple video streams while maintaining high performance through GPU acceleration.

Key features:
- Support for multiple synchronized camera inputs
- Real-time object detection using YOLOv8
- Configurable detection confidence thresholds
- Optional visualization with annotated images
- Standardized detection output using vision_msgs
- GPU acceleration support
- Flexible camera topic configuration

## Prerequisites

- ROS2 (Humble or newer recommended)
- Python 3.8+
- CUDA-capable GPU (recommended)
- Required Python packages:
  ```bash
  pip install ultralytics opencv-python-headless
  ```

## Installation

1. Create a ROS2 workspace (if you haven't already):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/yourusername/ros2_multicam_yolo.git
```

3. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the package:
```bash
colcon build --packages-select ros2_multicam_yolo
```

5. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Basic Launch
```bash
ros2 launch ros2_multicam_yolo multicam_detector.launch.py
```

### Parameters

The node can be configured with the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| model_path | string | "yolov8n.pt" | Path to the YOLO model file |
| camera_topics | string[] | ["/camera_image/Cam_BR", ...] | List of camera topics to subscribe to |
| confidence_threshold | double | 0.5 | Minimum confidence threshold for detections |
| device | string | "cuda" | Device to run inference on ("cuda" or "cpu") |
| publish_annotated_image | bool | true | Whether to publish annotated images |
| publish_bboxes | bool | true | Whether to publish bounding box messages |
| processed_imgsz | int | 1280 | Input image size for YOLO processing |

### Topics

For each camera topic `/camera_image/Cam_X`, the node publishes:
- `/camera_image/Cam_X_annotated` (sensor_msgs/Image): Annotated images with detection visualization
- `/camera_image/Cam_X_bboxes` (vision_msgs/Detection2DArray): Detected object bounding boxes

## Configuration

### Custom Camera Topics

You can specify custom camera topics using ROS2 parameters:

```bash
ros2 run ros2_multicam_yolo multi_cam_yolo_node --ros-args -p camera_topics:="['/cam1/image_raw','/cam2/image_raw']"
```

### Custom YOLO Model

To use a custom YOLO model:

```bash
ros2 run ros2_multicam_yolo multi_cam_yolo_node --ros-args -p model_path:="/path/to/your/model.pt"
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Citation

If you use this software in your research, please cite:

```bibtex
@software{ros2_multicam_yolo,
  author = {Your Name},
  title = {ROS2 Multi-Camera YOLO Detection},
  year = {2024},
  publisher = {GitHub},
  url = {https://github.com/yourusername/ros2_multicam_yolo}
}
```