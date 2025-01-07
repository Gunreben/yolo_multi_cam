#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

import functools

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None  # handle gracefully if not installed

# For publishing bounding boxes in a standard message
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class MultiCamYoloNode(Node):
    def __init__(self):
        super().__init__('multi_cam_yolo_node')

        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('camera_topics', [
            '/camera_image/Cam_BR',
            '/camera_image/Cam_BL',
            '/camera_image/Cam_ML',
            '/camera_image/Cam_FR',
            '/camera_image/Cam_FL'
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
            # If your YOLO library supports selecting device:
            # self.model.to(self.device)
        else:
            self.get_logger().warn("YOLO library not found. Detection will be disabled.")
            self.model = None

        # Create subscriptions for multiple cameras
        self.subscribers = []
        # Create publishers for each camera (depending on user settings)
        self.annotated_image_publishers = {}
        self.bbox_publishers = {}

        # Instead of self.image_callback, use functools.partial
        for topic in self.camera_topics:
            sub = self.create_subscription(
                Image,
                topic,
                functools.partial(self.image_callback, topic=topic),
                10
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

    def image_callback(self, msg: Image, topic=None):
        """
        Callback for each subscribed camera topic.
        """
        # If YOLO is not available, just return
        if self.model is None:
            return

        # Identify which camera topic triggered this callback
        current_topic = topic

        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO inference
        results = self.model.predict(frame, conf=self.confidence_threshold, device=self.device, imgsz=self.processed_imgsz)

        # If we got results, process them
        if len(results) > 0:
            dets = results[0].boxes

            # Prepare bounding boxes data for publication (vision_msgs)
            if self.publish_bboxes:
                detection_array_msg = Detection2DArray()
                # Stamp with the current message's header info (to keep consistent)
                detection_array_msg.header = msg.header

            # Draw bounding boxes if needed
            annotated_frame = frame.copy() if self.publish_annotated_image else None

            # Loop over detections
            for box in dets:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0]

                # ---------------------------------------------------------
                # Example filtering: let's log persons (0) or cars (2).
                # You can modify or remove this filtering as desired.
                if cls in [0, 2]:
                    self.get_logger().info(
                        f"Detected class {cls} with conf {conf:.2f} at "
                        f"[{x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}]"
                    )
                # ---------------------------------------------------------

                # Prepare detection2D msg if needed
                if self.publish_bboxes:
                    detection_msg = Detection2D()
                    detection_msg.header = msg.header

                    # YOLO uses pixel coordinates (top-left to bottom-right)
                    box_width = x2 - x1
                    box_height = y2 - y1

                    # Center of the bounding box
                    detection_msg.bbox.center.position.x = float(x1 + box_width / 2.0)
                    detection_msg.bbox.center.position.y = float(y1 + box_height / 2.0)
                    detection_msg.bbox.size_x = float(box_width)
                    detection_msg.bbox.size_y = float(box_height)

                    # Add hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls)
                    hypothesis.hypothesis.score = float(conf)
                    #hypothesis.pose = Pose()                    ###TODO: Is this necessary?

                    detection_msg.results.append(hypothesis)

                    # Add detection to the array
                    detection_array_msg.detections.append(detection_msg)

                # Draw bounding box if needed
                if self.publish_annotated_image and annotated_frame is not None:
                    color = (0, 255, 0)  # BGR (green box)
                    cv2.rectangle(
                        annotated_frame,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        color,
                        2
                    )
                    # Optionally, you could add class/score text
                    label_text = f"{cls} {conf:.2f}"
                    cv2.putText(
                        annotated_frame,
                        label_text,
                        (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        color,
                        1
                    )

            # Publish bounding boxes if enabled
            if self.publish_bboxes:
                self.bbox_publishers[current_topic].publish(detection_array_msg)

            # Publish annotated image if enabled
            if self.publish_annotated_image and annotated_frame is not None:
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_image_publishers[current_topic].publish(annotated_msg)
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
