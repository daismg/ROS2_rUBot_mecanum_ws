from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    video_device = LaunchConfiguration('video_device')

    declare_width = DeclareLaunchArgument(
        'image_width',
        default_value='160',
        description='Width of the camera image'
    )
    declare_height = DeclareLaunchArgument(
        'image_height',
        default_value='120',
        description='Height of the camera image'
    )
    declare_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device for USB camera'
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'image_width': image_width},
            {'image_height': image_height},
            {'video_device': video_device},
            # aquí pots afegir frame_id, framerate, etc.
            # {'camera_frame_id': 'usb_cam_link'},
            # {'framerate': 30.0},
        ]
    )

    return LaunchDescription([
        declare_width,
        declare_height,
        declare_device,
        usb_cam_node
    ])
