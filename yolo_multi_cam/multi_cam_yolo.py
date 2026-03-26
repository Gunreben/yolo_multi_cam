#!/usr/bin/env python3

import functools

import cv2
import numpy as np
import rclpy
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


class MultiCamYoloNode(Node):
    def __init__(self):
        super().__init__("multi_cam_yolo_node")

        self.declare_parameter("model_path", "yolo11n.pt")
        self.declare_parameter("camera_topics", [
            "/zed/zed_node/right_raw/image_raw_color/compressed",
        ])
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("publish_bboxes", True)
        self.declare_parameter("processed_imgsz", 640)
        self.declare_parameter("filter_classes", [0])
        self.declare_parameter("iou_threshold", 0.5)
        self.declare_parameter("jpeg_quality", 80)

        self.model_path = self.get_parameter("model_path").value
        self.camera_topics = self.get_parameter("camera_topics").value
        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.device = self.get_parameter("device").value
        self.publish_annotated_image = self.get_parameter("publish_annotated_image").value
        self.publish_bboxes = self.get_parameter("publish_bboxes").value
        self.processed_imgsz = self.get_parameter("processed_imgsz").value
        self.filter_classes = list(self.get_parameter("filter_classes").value)
        self.iou_threshold = self.get_parameter("iou_threshold").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value

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

        self.subscribers = []
        self.annotated_pubs: dict[str, object] = {}
        self.bbox_pubs: dict[str, object] = {}
        self._topic_compressed: dict[str, bool] = {}
        self._processing = False

        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        for topic in self.camera_topics:
            is_compressed = topic.endswith("/compressed")
            msg_type = CompressedImage if is_compressed else Image
            self._topic_compressed[topic] = is_compressed

            sub = self.create_subscription(
                msg_type,
                topic,
                functools.partial(
                    self._image_cb, topic=topic, compressed=is_compressed
                ),
                qos_sub,
            )
            self.subscribers.append(sub)

            if self.publish_annotated_image:
                ann_type = CompressedImage if is_compressed else Image
                self.annotated_pubs[topic] = self.create_publisher(
                    ann_type, f"{topic}_annotated", 10
                )
            if self.publish_bboxes:
                self.bbox_pubs[topic] = self.create_publisher(
                    Detection2DArray, f"{topic}_bboxes", 10
                )

            self.get_logger().info(f"Subscribed to: {topic}")

    def _image_cb(self, msg, *, topic: str, compressed: bool):
        if self.model is None or self._processing:
            return

        self._processing = True
        try:
            self._run_inference(msg, topic, compressed)
        except Exception as e:
            self.get_logger().warn(
                f"Inference failed on {topic}: {e}", throttle_duration_sec=2.0
            )
        finally:
            self._processing = False

    def _run_inference(self, msg, topic: str, compressed: bool):
        if compressed:
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

        results = self.model.predict(
            frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device,
            imgsz=self.processed_imgsz,
            classes=self.filter_classes if self.filter_classes else None,
            verbose=False,
        )

        if not results:
            return

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return

        det_array = Detection2DArray()
        det_array.header = msg.header

        annotated = frame.copy() if self.publish_annotated_image else None

        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            cls_name = self.class_names.get(cls_id, str(cls_id))

            if self.publish_bboxes:
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

        if self.publish_bboxes and det_array.detections:
            self.bbox_pubs[topic].publish(det_array)

        if annotated is not None and topic in self.annotated_pubs:
            if self._topic_compressed[topic]:
                out = _cv2_to_compressed_imgmsg(
                    annotated, msg.header, self.jpeg_quality
                )
            else:
                out = _cv2_to_imgmsg(annotated, msg.header)
            self.annotated_pubs[topic].publish(out)


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
