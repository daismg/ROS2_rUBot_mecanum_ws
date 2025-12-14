from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/aligned_depth_to_color/image_raw',
        description='Depth image topic aligned with RGB'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='CameraInfo topic for RGB camera'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description='Camera frame id'
    )

    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='odom',
        description='Target frame to transform the point to (e.g. odom, base_link)'
    )

    u_arg = DeclareLaunchArgument(
        'u',
        default_value='320',
        description='Pixel column (u)'
    )

    v_arg = DeclareLaunchArgument(
        'v',
        default_value='240',
        description='Pixel row (v)'
    )

    node = Node(
        package='my_camera_tools',          # posa aquí el teu nom de package
        executable='image_point_distance_node_exec',  # nom de l’executable instal·lat
        name='image_point_distance_node',
        output='screen',
        parameters=[
            {
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'target_frame': LaunchConfiguration('target_frame'),
                'u': LaunchConfiguration('u'),
                'v': LaunchConfiguration('v'),
            }
        ]
    )

    return LaunchDescription([
        depth_topic_arg,
        camera_info_topic_arg,
        camera_frame_arg,
        target_frame_arg,
        u_arg,
        v_arg,
        node
    ])
