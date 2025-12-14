from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments que venen del bringup
    serial_port = LaunchConfiguration('rplidar_serial_port')
    frame_id = LaunchConfiguration('rplidar_frame_id')

    # Arguments interns amb valors per defecte (pots exposar-los també al bringup si vols)
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    channel_type = LaunchConfiguration('channel_type', default='serial')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    declare_port = DeclareLaunchArgument(
        'rplidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )

    declare_frame = DeclareLaunchArgument(
        'rplidar_frame_id',
        default_value='laser',
        description='Frame ID for RPLidar data'
    )

    declare_baudrate = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baudrate for RPLidar serial connection'
    )

    declare_channel_type = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='Channel type (serial/tcp) for RPLidar'
    )

    declare_inverted = DeclareLaunchArgument(
        'inverted',
        default_value='false',
        description='Whether to invert scan data'
    )

    declare_angle_compensate = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Enable angle compensation'
    )

    declare_scan_mode = DeclareLaunchArgument(
        'scan_mode',
        default_value='Sensitivity',
        description='Scan mode of lidar'
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',  # tal com tens al rplidar_a1.launch.py
        name='rplidar_node',
        output='screen',
        respawn=True,  # perquè es reiniciï si cau
        respawn_delay=2.0,
        parameters=[{
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            # alguns forks també accepten 'scan_mode'
            'scan_mode': scan_mode,
        }]
    )

    return LaunchDescription([
        declare_port,
        declare_frame,
        declare_baudrate,
        declare_channel_type,
        declare_inverted,
        declare_angle_compensate,
        declare_scan_mode,
        rplidar_node
    ])
