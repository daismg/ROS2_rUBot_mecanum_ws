from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mecanum_serial_port = LaunchConfiguration('mecanum_serial_port')

    declare_mecanum_serial_port = DeclareLaunchArgument(
        'mecanum_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Nano mecanum driver'
    )

    driver_node = Node(
        package='my_robot_driver',
        executable='rubot_nano_driver_mecanum_exec',
        name='rubot_nano_driver_mecanum',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'serial_port': mecanum_serial_port,
            'baud_rate': 57600,
            'loop_rate': 30,
            'encoder_cpr': 1320,
            # 'use_sim_time': False  # no cal forçar-ho en HW
        }]
    )

    return LaunchDescription([
        declare_mecanum_serial_port,
        driver_node
    ])
