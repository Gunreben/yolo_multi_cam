import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("yolo_multi_cam")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")

    return LaunchDescription([
        Node(
            package="yolo_multi_cam",
            executable="multi_cam_yolo",
            name="multi_cam_yolo_node",
            parameters=[params_file],
            output="screen",
        ),
    ])
