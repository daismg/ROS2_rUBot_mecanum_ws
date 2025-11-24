#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('my_robot_ai_identification')
    params_file = os.path.join(pkg_path, 'config', 'yolo_signals.yaml')

    yolo_node = Node(
        package='my_robot_ai_identification',
        executable='rubot_navigation_yolo_exec',
        name='object_detection',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([yolo_node])
