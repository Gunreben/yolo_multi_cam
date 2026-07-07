import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/ros2_ws/src/yolo_multi_cam/install/yolo_multi_cam'
