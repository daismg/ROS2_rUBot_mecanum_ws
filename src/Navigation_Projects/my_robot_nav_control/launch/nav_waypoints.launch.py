from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Generates a launch description for the navigation task node,
    loading waypoints from a YAML file.
    """
    # Troba la ruta al nostre paquet
    pkg_dir = get_package_share_directory('my_robot_nav_control')
    
    # Construeix la ruta al fitxer de paràmetres YAML
    params_file = os.path.join(pkg_dir, 'config', 'waypoints_paramsV2.yaml')

    # Create the node action, passing the waypoints as a parameter.
    navigation_node = Node(
        package='my_robot_nav_control',
        executable='nav_waypoints_exec',  # Make sure this matches your executable name
        name='nav_waypoints_node',
        parameters=[params_file], # Carrega tots els paràmetres del fitxer
        output='screen',  #  Good for seeing the output in the terminal
    )

    # Create the launch description and add the actions.
    return LaunchDescription([
        navigation_node
    ])