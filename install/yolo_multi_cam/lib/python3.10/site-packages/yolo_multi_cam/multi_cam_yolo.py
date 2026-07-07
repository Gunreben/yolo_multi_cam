#!/usr/bin/env python3
"""Multi-camera YOLO person detection.

Subscribes to the tractor's compressed camera topics, runs YOLO on each
frame (rate-limited per camera) and publishes vision_msgs/Detection2DArray
per camera on stable topic names:

    /yolo/<camera_name>/detections

Detection arrays are published for EVERY processed frame, including empty
ones — downstream consumers rely on empty messages to expire stale
detections.

Cameras whose ROS topic changed between recording-stack v1 and v2 (the
ZED) can list both names (``topic_v1`` / ``topic_v2``); with
``topic_profile: auto`` the node subscribes to both and locks onto
whichever delivers first, so the same configuration works against v1
bags and the live v2 stack.
"""

import functools
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


def _cv2_to_imgmsg(frame: np.ndarray, header) -> Image:
    """Manual cv2 -> sensor_msgs/Image without cv_bridge (avoids numpy 2.x crash)."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = frame.shape[:2]
    if frame.ndim == 3:
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
    else:
        msg.encoding = "mono8"
        msg.step = msg.width
    msg.is_bigendian = False
    msg.data = frame.tobytes()
    return msg


def _cv2_to_compressed_imgmsg(
    frame: np.ndarray, header, jpeg_quality: int = 80
) -> CompressedImage:
    msg = CompressedImage()
    msg.header = header
    msg.format = "jpeg"
    _, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    msg.data = buf.tobytes()
    return msg


class CameraState:
    """Per-camera subscriptions, publishers and rate limiting."""

    def __init__(self, name: str):
        self.name = name
        self.topics: list[str] = []          # candidate input topics
        self.locked_topic: str | None = None  # first topic that delivered
        self.subs: dict[str, object] = {}
        self.det_pub = None
        self.annotated_pub = None
        self.annotated_compressed = True
        self.last_processed = 0.0
        # Per-camera busy flag: with the multithreaded executor a slow
        # frame must not stack up successors of the same camera.
        self.busy = threading.Lock()


class MultiCamYoloNode(Node):
    def __init__(self):
        super().__init__("multi_cam_yolo_node")

        self.declare_parameter("model_path", "yolo11n.pt")
        self.declare_parameter("camera_names", ["zed_front"])
        # v1 = topics as recorded by the v1 stack, v2 = live v2 stack,
        # auto = subscribe to both and lock onto whichever delivers first.
        self.declare_parameter("topic_profile", "auto")
        self.declare_parameter("target_rate", 10.0)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("publish_annotated_image", False)
        self.declare_parameter("processed_imgsz", 640)
        self.declare_parameter("filter_classes", [0])
        self.declare_parameter("iou_threshold", 0.5)
        self.declare_parameter("jpeg_quality", 80)

        self.model_path = self._resolve_model_path(
            self.get_parameter("model_path").value)
        self.topic_profile = self.get_parameter("topic_profile").value
        self.target_rate = self.get_parameter("target_rate").value
        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.device = self.get_parameter("device").value
        self.publish_annotated_image = self.get_parameter("publish_annotated_image").value
        self.processed_imgsz = self.get_parameter("processed_imgsz").value
        self.filter_classes = list(self.get_parameter("filter_classes").value)
        self.iou_threshold = self.get_parameter("iou_threshold").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value
        self.min_interval = 1.0 / self.target_rate if self.target_rate > 0.0 else 0.0

        self.class_names: dict[int, str] = {}
        self.model = None

        if YOLO is not None:
            self.get_logger().info(
                f"Loading YOLO model from {self.model_path} on {self.device}..."
            )
            self.model = YOLO(self.model_path)
            self.class_names = self.model.names or {}

            self.get_logger().info("Running CUDA warmup inference...")
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            self.model.predict(
                dummy, device=self.device, imgsz=self.processed_imgsz, verbose=False
            )
            self.get_logger().info(
                f"Model ready. Filtering to: "
                f"{[self.class_names.get(c, str(c)) for c in self.filter_classes]}"
            )
        else:
            self.get_logger().error(
                "ultralytics not installed. Run: pip install ultralytics"
            )

        # Decode/preprocess of different cameras run concurrently
        # (cv2/numpy/torch release the GIL); the model itself is guarded
        # by _predict_lock, so inference throughput bounds the total.
        self._predict_lock = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()

        self.cameras: dict[str, CameraState] = {}
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        for name in self.get_parameter("camera_names").value:
            cam = CameraState(name)
            cam.topics = self._camera_topics(name)
            if not cam.topics:
                self.get_logger().warn(f"Camera '{name}': no topic configured, skipping")
                continue

            cam.det_pub = self.create_publisher(
                Detection2DArray, f"/yolo/{name}/detections", 10
            )
            if self.publish_annotated_image:
                cam.annotated_compressed = cam.topics[0].endswith("/compressed")
                if cam.annotated_compressed:
                    cam.annotated_pub = self.create_publisher(
                        CompressedImage, f"/yolo/{name}/annotated/compressed", 10
                    )
                else:
                    cam.annotated_pub = self.create_publisher(
                        Image, f"/yolo/{name}/annotated", 10
                    )

            for topic in cam.topics:
                msg_type = CompressedImage if topic.endswith("/compressed") else Image
                cam.subs[topic] = self.create_subscription(
                    msg_type,
                    topic,
                    functools.partial(self._image_cb, camera_name=name, topic=topic),
                    qos_sub,
                    callback_group=self._cb_group,
                )
            self.cameras[name] = cam
            self.get_logger().info(
                f"Camera '{name}': {' | '.join(cam.topics)} -> /yolo/{name}/detections"
            )

    def _resolve_model_path(self, model_path: str) -> str:
        """Absolute paths pass through; else prefer the installed copy."""
        if os.path.isabs(model_path):
            return model_path
        try:
            share = get_package_share_directory("yolo_multi_cam")
            candidate = os.path.join(share, model_path)
            if os.path.isfile(candidate):
                return candidate
        except Exception:
            pass
        return model_path

    def _camera_topics(self, name: str) -> list[str]:
        prefix = f"cameras.{name}"
        self.declare_parameter(f"{prefix}.topic", "")
        self.declare_parameter(f"{prefix}.topic_v1", "")
        self.declare_parameter(f"{prefix}.topic_v2", "")
        topic = self.get_parameter(f"{prefix}.topic").value
        v1 = self.get_parameter(f"{prefix}.topic_v1").value
        v2 = self.get_parameter(f"{prefix}.topic_v2").value

        if topic:
            return [topic]
        if self.topic_profile == "v1":
            return [t for t in [v1] if t]
        if self.topic_profile == "v2":
            return [t for t in [v2] if t]
        return [t for t in dict.fromkeys([v2, v1]) if t]

    def _image_cb(self, msg, *, camera_name: str, topic: str):
        cam = self.cameras[camera_name]

        # Auto profile: lock onto the first topic that delivers and drop
        # the alternative subscription.
        if cam.locked_topic is None:
            cam.locked_topic = topic
            for other, sub in list(cam.subs.items()):
                if other != topic:
                    self.destroy_subscription(sub)
                    del cam.subs[other]
                    self.get_logger().info(
                        f"Camera '{camera_name}': locked to {topic} (dropped {other})"
                    )
        elif topic != cam.locked_topic:
            return

        if self.model is None:
            return

        now = time.monotonic()
        if self.min_interval > 0.0 and now - cam.last_processed < self.min_interval:
            return
        if not cam.busy.acquire(blocking=False):
            return  # previous frame of this camera still in flight
        cam.last_processed = now

        try:
            self._run_inference(msg, cam)
        except Exception as e:
            self.get_logger().warn(
                f"Inference failed on {topic}: {e}", throttle_duration_sec=2.0
            )
        finally:
            cam.busy.release()

    def _run_inference(self, msg, cam: CameraState):
        if isinstance(msg, CompressedImage):
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            h, w = msg.height, msg.width
            channels = len(msg.data) // (h * w) if h > 0 and w > 0 else 3
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, channels))
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if frame is None:
            return

        with self._predict_lock:
            results = self.model.predict(
                frame,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                device=self.device,
                imgsz=self.processed_imgsz,
                classes=self.filter_classes if self.filter_classes else None,
                verbose=False,
            )

        det_array = Detection2DArray()
        det_array.header = msg.header

        annotated = frame.copy() if cam.annotated_pub is not None else None

        boxes = results[0].boxes if results else None
        for box in boxes if boxes is not None else []:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            cls_name = self.class_names.get(cls_id, str(cls_id))

            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls_name
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            det_array.detections.append(det)

            if annotated is not None:
                color = (0, 255, 0) if cls_name == "person" else (255, 180, 0)
                cv2.rectangle(
                    annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, 2
                )
                label = f"{cls_name} {conf:.2f}"
                cv2.putText(
                    annotated,
                    label,
                    (int(x1), int(y1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1,
                )

        # Always publish, even when empty: downstream fusion expires
        # stale detections based on these messages.
        cam.det_pub.publish(det_array)

        if annotated is not None:
            if cam.annotated_compressed:
                out = _cv2_to_compressed_imgmsg(annotated, msg.header, self.jpeg_quality)
            else:
                out = _cv2_to_imgmsg(annotated, msg.header)
            cam.annotated_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MultiCamYoloNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
