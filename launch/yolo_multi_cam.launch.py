import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("yolo_multi_cam")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=params_file,
            description="Extra parameter file loaded on top of config/params.yaml "
                        "(e.g. config/params_desktop.yaml)"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="yolo_multi_cam",
            executable="multi_cam_yolo",
            name="multi_cam_yolo_node",
            parameters=[params_file, LaunchConfiguration("params_file"),
                        {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output="screen",
        ),
    ])
