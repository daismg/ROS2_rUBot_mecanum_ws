from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='robot_arm/my_simple_robot.urdf',
        description='Robot model path relative to urdf/ folder'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = LaunchConfiguration('robot_model')

    # Get share path as string
    description_pkg_path = str(get_package_share_path('my_robot_description'))

    # Create substitutions
    urdf_path = PathJoinSubstitution([
        TextSubstitution(text=description_pkg_path),
        'urdf',
        robot_model
    ])

    rviz_config_path = os.path.join(description_pkg_path, 'rviz', 'urdf_config.rviz')

    # Robot description using xacro
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
