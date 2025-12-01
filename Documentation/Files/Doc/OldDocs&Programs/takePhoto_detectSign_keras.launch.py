from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'my_robot_ai_identification'  # Reemplaza con el nombre de tu paquete
    script_name = 'takePhoto_detectSign_keras.py' # Reemplaza con el nombre de tu archivo .py

    return LaunchDescription([
        Node(
            package=package_name,
            executable=script_name,
            name='keras_detector_node',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()