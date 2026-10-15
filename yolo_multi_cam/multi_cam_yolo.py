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

Backends: ``model_path`` ending in ``.onnx`` runs through onnxruntime
(TensorRT FP16 / CUDA, no ultralytics or torch needed, see onnx_yolo.py);
anything else (``.pt``, ``.engine``) goes through ultralytics.
"""

import array
import functools
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
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
    msg.data = array.array("B", frame.tobytes())  # Humble fast path
    return msg


def _cv2_to_compressed_imgmsg(
    frame: np.ndarray, header, jpeg_quality: int = 80
) -> CompressedImage:
    msg = CompressedImage()
    msg.header = header
    msg.format = "jpeg"
    _, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    msg.data = array.array("B", buf.tobytes())
    return msg


class _UltralyticsBackend:
    def __init__(self, model_path, device, imgsz):
        self.model = YOLO(model_path)
        self.names = self.model.names or {}
        self.device, self.imgsz = device, imgsz
        self.model.predict(np.zeros((640, 640, 3), dtype=np.uint8), device=device,
                           imgsz=imgsz, verbose=False)

    def predict(self, frames, conf, iou, classes):
        out = []
        for frame in frames:
            results = self.model.predict(frame, conf=conf, iou=iou, device=self.device,
                                         imgsz=self.imgsz, classes=classes, verbose=False)
            boxes = results[0].boxes if results else None
            if boxes is None or len(boxes) == 0:
                out.append(np.zeros((0, 6), np.float32))
                continue
            out.append(np.concatenate([boxes.xyxy.cpu().numpy(),
                                       boxes.conf.cpu().numpy()[:, None],
                                       boxes.cls.cpu().numpy()[:, None]], 1))
        return out


class _OnnxBackend:
    def __init__(self, model_path, providers, imgsz, cache_dir, batch):
        from .onnx_yolo import OnnxYolo
        self.model = OnnxYolo(model_path, providers=providers, imgsz=imgsz,
                              cache_dir=cache_dir)
        self.names = self.model.names
        self.batch = batch
        self.model.predict([np.zeros((imgsz, imgsz, 3), np.uint8)], pad_to=batch)

    def prepare(self, frame):
        return self.model.prepare(frame)

    def predict(self, frames, conf, iou, classes):
        return self.model.predict(frames, conf=conf, iou=iou, classes=classes,
                                  pad_to=self.batch)

    def predict_prepared(self, items, conf, iou, classes):
        return self.model.predict_prepared(items, conf=conf, iou=iou, classes=classes,
                                           pad_to=self.batch)


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
        self.frame_id = ""                    # overrides the image header frame
        self.last_processed = 0.0
        self.pending = None                   # latest unprocessed message


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
        # onnx backend only
        self.declare_parameter("providers", ["tensorrt", "cuda", "cpu"])
        self.declare_parameter("trt_cache_dir", "~/.cache/yolo_multi_cam_trt")

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

        self.get_logger().info(f"Loading YOLO model from {self.model_path}...")
        if self.model_path.endswith(".onnx"):
            self.model = _OnnxBackend(
                self.model_path, list(self.get_parameter("providers").value),
                self.processed_imgsz,
                os.path.expanduser(self.get_parameter("trt_cache_dir").value),
                batch=len(self.get_parameter("camera_names").value))
            self.get_logger().info(f"onnxruntime providers: "
                                   f"{self.model.model.active_providers}")
        elif YOLO is not None:
            self.model = _UltralyticsBackend(self.model_path, self.device,
                                             self.processed_imgsz)
        else:
            self.get_logger().error(
                "ultralytics not installed (pip install ultralytics) — "
                "or use an exported .onnx model_path"
            )
        if self.model is not None:
            self.class_names = self.model.names
            self.get_logger().info(
                f"Model ready. Filtering to: "
                f"{[self.class_names.get(c, str(c)) for c in self.filter_classes]}"
            )

        # Subscription callbacks only stash the latest message per camera;
        # one worker thread decodes all due cameras in parallel (cv2 releases
        # the GIL) and runs them through the model as one batch. rclpy's
        # multithreaded executor costs more than the inference itself at
        # 6 cameras x 25 fps.
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._decoder = ThreadPoolExecutor(max_workers=6)

        self.cameras: dict[str, CameraState] = {}
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        for name in self.get_parameter("camera_names").value:
            cam = CameraState(name)
            cam.topics = self._camera_topics(name)
            cam.frame_id = self.get_parameter(f"cameras.{name}.frame_id").value
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
                )
            self.cameras[name] = cam
            self.get_logger().info(
                f"Camera '{name}': {' | '.join(cam.topics)} -> /yolo/{name}/detections"
            )

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def _resolve_model_path(self, model_path: str) -> str:
        """Absolute paths pass through; else prefer the installed copy."""
        model_path = os.path.expanduser(model_path)
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
        # Optional: frame for the published detections instead of the image
        # header's (e.g. bags stamped with a non-optical camera frame).
        self.declare_parameter(f"{prefix}.frame_id", "")
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
        with self._lock:
            cam.pending = msg
        self._wake.set()

    def _worker_loop(self):
        while rclpy.ok():
            self._wake.wait(0.1)
            self._wake.clear()
            now = time.monotonic()
            due = []
            with self._lock:
                for cam in self.cameras.values():
                    if cam.pending is None or now - cam.last_processed < self.min_interval:
                        continue
                    due.append((cam, cam.pending))
                    cam.pending = None
                    cam.last_processed = now
            if not due:
                continue
            try:
                self._run_inference(due)
            except Exception as e:  # noqa: BLE001 - keep the worker alive
                self.get_logger().warn(f"Inference failed: {e}", throttle_duration_sec=2.0)

    @staticmethod
    def _decode(msg):
        if isinstance(msg, CompressedImage):
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        h, w = msg.height, msg.width
        channels = len(msg.data) // (h * w) if h > 0 and w > 0 else 3
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, channels))
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    def _decode_prepare(self, msg):
        frame = self._decode(msg)
        if frame is None or not hasattr(self.model, "prepare"):
            return frame, None
        return frame, self.model.prepare(frame)

    def _run_inference(self, due):
        # decode + letterbox in the pool (cv2 releases the GIL)
        frames = list(self._decoder.map(self._decode_prepare, [msg for _, msg in due]))
        ok = [(cam, msg, f, p) for (cam, msg), (f, p) in zip(due, frames) if f is not None]
        if not ok:
            return
        classes = self.filter_classes if self.filter_classes else None
        if ok[0][3] is not None:
            results = self.model.predict_prepared(
                [p for *_, p in ok], self.confidence_threshold, self.iou_threshold, classes)
        else:
            results = self.model.predict(
                [f for _, _, f, _ in ok], self.confidence_threshold, self.iou_threshold,
                classes)
        ok = [(cam, msg, f) for cam, msg, f, _ in ok]
        for (cam, msg, frame), boxes in zip(ok, results):
            self._publish(cam, msg, frame, boxes)

    def _publish(self, cam: CameraState, msg, frame, boxes):
        header = msg.header
        if cam.frame_id:
            header.frame_id = cam.frame_id
        det_array = Detection2DArray()
        det_array.header = header

        annotated = frame.copy() if cam.annotated_pub is not None else None

        for x1, y1, x2, y2, conf, cls in boxes:
            cls_id = int(cls)
            conf = float(conf)
            x1, y1, x2, y2 = float(x1), float(y1), float(x2), float(y2)
            cls_name = self.class_names.get(cls_id, str(cls_id))

            det = Detection2D()
            det.header = header
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
                out = _cv2_to_compressed_imgmsg(annotated, header, self.jpeg_quality)
            else:
                out = _cv2_to_imgmsg(annotated, header)
            cam.annotated_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MultiCamYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
