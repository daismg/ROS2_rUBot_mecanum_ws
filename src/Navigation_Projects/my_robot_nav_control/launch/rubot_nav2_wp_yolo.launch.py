#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('my_robot_nav_control')
    params_file = os.path.join(pkg_path, 'config', 'target_yolo.yaml')

    nav_node = Node(
        package='my_robot_nav_control',     # <-- posa el teu paquet real
        executable='rubot_nav2_wp_yolo_exec',        # <-- nom del console_script al setup.py
        name='custom_nav2',                # <-- igual que el node i el YAML
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([nav_node])
