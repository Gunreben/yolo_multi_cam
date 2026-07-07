# yolo_multi_cam

Real-time multi-camera person detection using YOLO (ultralytics) for ROS 2.

Runs one YOLO model over the tractor's compressed camera streams
(5× ArkCam surround + ZED 2i front), rate-limited per camera, and
publishes `vision_msgs/Detection2DArray` per camera. Built as the camera
branch of the obstacle/people-detection demonstrator (see the
`obstacle_detection` package for lidar clustering and fusion).

## Topics

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| in | `/camera/<pos>/image_raw/compressed` | CompressedImage | ArkCams (v1 and v2 stacks) |
| in | `/zed/zed_node/left/color/raw/image/compressed` | CompressedImage | ZED, v2 stack |
| in | `/zed/zed_node/left_raw/image_raw_color/compressed` | CompressedImage | ZED, v1 bags |
| out | `/yolo/<camera_name>/detections` | Detection2DArray | **always published per processed frame, incl. empty** |
| out | `/yolo/<camera_name>/annotated/compressed` | CompressedImage | only if `publish_annotated_image` |

Output topic names are **stable** (keyed by camera name, not input
topic), so downstream consumers are independent of which recording-stack
version supplies the images. The `Detection2DArray` header carries the
image timestamp and the camera's optical frame — fusion uses that frame
for TF lookups.

Empty detection arrays are intentional: downstream fusion expires stale
detections through them. Do not filter them out.

## v1/v2 topic profiles

The ZED topics were renamed between recording-stack v1 (wrapper 4.x)
and v2 (wrapper 5.x). Cameras may configure `topic_v1` / `topic_v2`
instead of a single `topic`:

- `topic_profile: v1` / `v2` — subscribe to that variant only
- `topic_profile: auto` (default) — subscribe to both, lock onto
  whichever delivers first, drop the other subscription

ArkCam topics are identical in both stacks and use plain `topic`.

## Parameters (config/params.yaml)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `yolo11n.pt` | Relative paths resolve against the installed package share dir first |
| `camera_names` | 6 cameras | Keys for the `cameras.<name>.*` blocks |
| `cameras.<name>.topic[_v1/_v2]` | — | Input topic(s) per camera |
| `topic_profile` | `auto` | `v1`, `v2` or `auto` |
| `target_rate` | 10.0 | Max inference rate per camera (Hz); surplus frames are skipped |
| `confidence_threshold` | 0.45 | Minimum detection confidence |
| `iou_threshold` | 0.5 | NMS IoU threshold |
| `filter_classes` | `[0]` | COCO class IDs to keep (0 = person); empty = all |
| `processed_imgsz` | 640 | YOLO input size (model resizes internally) |
| `device` | `cuda:0` | Inference device |
| `publish_annotated_image` | false | Debug overlay images |
| `jpeg_quality` | 80 | JPEG quality of annotated output |

## Usage

```bash
ros2 launch yolo_multi_cam yolo_multi_cam.launch.py
```

Requires `ultralytics` (and a CUDA-enabled torch). On Jetson use the
NVIDIA torch wheels:
<https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048>

## Performance (AGX Orin, D3 bench 2026-07-07)

TensorRT FP16 engine (`/mnt/ros2_ws/models/yolo11n.engine`, built
on-device — see comment in `config/params.yaml`; engines are
device/TensorRT-version specific): 17.2 ms/frame end-to-end in-node
(GPU ~5 ms; Python pre/post dominates) vs 22.5 ms for the `.pt`.
With 4 live ArkCams + ZED and the multithreaded executor:
8.3 Hz/ArkCam + 7.5 Hz ZED (= every 3rd/2nd frame), ~70 ms
stamp→detection latency, ~1.8 cores, GPU ~50% (shared with ZED NEURAL
depth). `target_rate` quantizes to frame strides of the 25/15 fps
sources — see `config/params.yaml` for the measured 10-vs-12 trade-off.

## Changelog

### 0.2.1 (2026-07, D3)
- Multithreaded executor + reentrant callbacks: JPEG decode of
  different cameras overlaps inference (model behind a lock, per-camera
  busy flag prevents same-camera pile-up).
- Default `model_path` switched to the on-device TensorRT FP16 engine.

### 0.2.0 (2026-07)
- Output topics renamed to stable `/yolo/<camera_name>/detections`
  (were `<input_topic>_bboxes`).
- **Empty detection arrays are now published** for every processed
  frame — required by detection_fusion for stale-detection expiry
  (previously a camera that stopped seeing a person kept its last
  detection "live" forever downstream).
- v1/v2 topic profiles with auto-detection (ZED wrapper rename).
- Per-camera rate limiting (`target_rate`, default 10 Hz).
- ZED topics updated to wrapper 5.x names (v2 profile).
- `yolo11n.pt` installed with the package; relative `model_path`
  resolves against the share directory (no longer cwd-dependent).
- Removed the no-op `_processing` reentrancy flag and the
  `publish_bboxes` toggle (detections are the point of the node).

### 0.1.0
- Initial version: per-topic subscription, `<topic>_bboxes` outputs,
  publishes only non-empty detections.

## License

MIT — see [LICENSE](LICENSE).

## Citation

```bibtex
@software{multi_cam_yolo,
  author = {Lorenz Gunreben},
  title = {ROS2 Multi-Camera YOLO Detection},
  year = {2024},
  publisher = {GitHub},
  url = {https://github.com/Gunreben/multi_cam_yolo}
}
```
