import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ================================================================
    # Arguments globals (només els que realment uses)
    # ================================================================
    robot_model = LaunchConfiguration('robot_model')

    mecanum_serial_port = LaunchConfiguration('mecanum_serial_port')
    rplidar_serial_port = LaunchConfiguration('rplidar_serial_port')
    rplidar_frame_id = LaunchConfiguration('rplidar_frame_id')

    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    usb_video_device = LaunchConfiguration('usb_video_device')

    declare_robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='robot_arm/my_simple_robot.urdf',
        description='URDF/XACRO path inside my_robot_description/urdf'
    )

    declare_mecanum_serial_port = DeclareLaunchArgument(
        'mecanum_serial_port', default_value='/dev/ttyACM0',
        description='Serial port for Nano mecanum driver'
    )

    declare_rplidar_serial_port = DeclareLaunchArgument(
        'rplidar_serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )

    declare_rplidar_frame_id = DeclareLaunchArgument(
        'rplidar_frame_id', default_value='base_link',
        description='Frame ID for RPLidar data'
    )

    declare_camera_width = DeclareLaunchArgument(
        'camera_width', default_value='160',
        description='Width of the camera image'
    )

    declare_camera_height = DeclareLaunchArgument(
        'camera_height', default_value='120',
        description='Height of the camera image'
    )

    declare_usb_video_device = DeclareLaunchArgument(
        'usb_video_device', default_value='/dev/video0',
        description='Video device for USB camera'
    )

    # ================================================================
    # Robot description (URDF/XACRO) + robot_state_publisher
    # ================================================================
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            robot_model
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
        ]
    )

    # ================================================================
    # Inclusió dels sub-launchs de hardware
    # ================================================================
    robot_driver_hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_bringup'),
                'launch',
                'robot_driver_hw.launch.py'
            )
        ),
        launch_arguments={
            'mecanum_serial_port': mecanum_serial_port,
        }.items()
    )

    rplidar_hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_bringup'),
                'launch',
                'rplidar_hw.launch.py'
            )
        ),
        launch_arguments={
            'rplidar_serial_port': rplidar_serial_port,
            'rplidar_frame_id': rplidar_frame_id,
        }.items()
    )

    usb_cam_hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_bringup'),
                'launch',
                'usb_cam_hw.launch.py'
            )
        ),
        launch_arguments={
            'image_width': camera_width,
            'image_height': camera_height,
            'video_device': usb_video_device,
        }.items()
    )

    # ================================================================
    # Construcció LaunchDescription
    # ================================================================
    ld = LaunchDescription()

    ld.add_action(declare_robot_model)
    ld.add_action(declare_mecanum_serial_port)
    ld.add_action(declare_rplidar_serial_port)
    ld.add_action(declare_rplidar_frame_id)
    ld.add_action(declare_camera_width)
    ld.add_action(declare_camera_height)
    ld.add_action(declare_usb_video_device)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(robot_driver_hw_launch)
    ld.add_action(usb_cam_hw_launch)
    ld.add_action(rplidar_hw_launch)

    return ld
