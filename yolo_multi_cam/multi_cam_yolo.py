#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np
import functools

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # handle gracefully if not installed

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class MultiCamYoloNode(Node):
    def __init__(self):
        super().__init__('multi_cam_yolo_node')

        # QoS profile for sensor data (BEST_EFFORT for compatibility with some camera drivers)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('camera_topics', [
            '/camera/side_left/compressed',
            '/camera/rear_left/compressed',
            '/camera/rear_mid/compressed',
            '/camera/rear_right/compressed',
            '/camera/side_right/compressed'
        ])
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('publish_bboxes', True)
        self.declare_parameter('processed_imgsz', 1280)

        # Get parameter values
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.publish_annotated_image = self.get_parameter('publish_annotated_image').get_parameter_value().bool_value
        self.publish_bboxes = self.get_parameter('publish_bboxes').get_parameter_value().bool_value
        self.processed_imgsz = self.get_parameter('processed_imgsz').get_parameter_value().integer_value

        self.bridge = CvBridge()

        # Load YOLO model
        if YOLO is not None:
            self.get_logger().info(f"Loading YOLO model from {self.model_path} on {self.device}...")
            self.model = YOLO(self.model_path)
        else:
            self.get_logger().warn(" The 'ultralytics' library is not installed. Please run:\n"
                "  pip install ultralytics\n")
            self.model = None

        # Create subscriptions for multiple cameras
        self.subscribers = []
        self.annotated_image_publishers = {}
        self.bbox_publishers = {}

        for topic in self.camera_topics:
            is_compressed = topic.endswith("/compressed")
            msg_type = CompressedImage if is_compressed else Image

            sub = self.create_subscription(
                msg_type,
                topic,
                functools.partial(self.image_callback, topic=topic, compressed=is_compressed),
                qos_profile  # Apply BEST_EFFORT QoS
            )
            self.subscribers.append(sub)

            if self.publish_annotated_image:
                annotated_topic = f"{topic}_annotated"
                self.annotated_image_publishers[topic] = self.create_publisher(Image, annotated_topic, 10)
                self.get_logger().info(f"Will publish annotated images on: {annotated_topic}")

            if self.publish_bboxes:
                bboxes_topic = f"{topic}_bboxes"
                self.bbox_publishers[topic] = self.create_publisher(Detection2DArray, bboxes_topic, 10)
                self.get_logger().info(f"Will publish bounding boxes on: {bboxes_topic}")

    def image_callback(self, msg, topic=None, compressed=False):
        if self.model is None:
            return

        if compressed:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.predict(frame, conf=self.confidence_threshold, device=self.device, imgsz=self.processed_imgsz)

        if len(results) > 0:
            dets = results[0].boxes

            if self.publish_bboxes:
                detection_array_msg = Detection2DArray()
                detection_array_msg.header = msg.header

            annotated_frame = frame.copy() if self.publish_annotated_image else None

            for box in dets:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0]

                if cls in [0, 2]:
                    self.get_logger().info(
                        f"Detected class {cls} with conf {conf:.2f} at [{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]"
                    )

                if self.publish_bboxes:
                    detection_msg = Detection2D()
                    detection_msg.header = msg.header
                    detection_msg.bbox.center.position.x = float(x1 + (x2 - x1) / 2.0)
                    detection_msg.bbox.center.position.y = float(y1 + (y2 - y1) / 2.0)
                    detection_msg.bbox.size_x = float(x2 - x1)
                    detection_msg.bbox.size_y = float(y2 - y1)

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls)
                    hypothesis.hypothesis.score = float(conf)
                    detection_msg.results.append(hypothesis)
                    detection_array_msg.detections.append(detection_msg)

                if self.publish_annotated_image and annotated_frame is not None:
                    cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    label_text = f"{cls} {conf:.2f}"
                    cv2.putText(annotated_frame, label_text, (int(x1), int(y1) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if self.publish_bboxes:
                self.bbox_publishers[topic].publish(detection_array_msg)

            if self.publish_annotated_image and annotated_frame is not None:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_image_publishers[topic].publish(annotated_msg)

        else:
            self.get_logger().info("No detections in this frame.")


def main(args=None):
    rclpy.init(args=args)
    node = MultiCamYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
