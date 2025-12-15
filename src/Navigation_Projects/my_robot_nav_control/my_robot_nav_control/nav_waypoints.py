#!/usr/bin/env python3
import time
import ast

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


class NavigationTask(Node):

    def __init__(self):
        super().__init__('nav_waypoints_node')

        self.navigator = BasicNavigator()

        # Declare parameters
        self.declare_parameter('initial_pose', '(0.0, 0.0, 0.0)')
        self.declare_parameter('waypoints', '[]')
        self.declare_parameter('final_pose', '(0.0, 0.0, 0.0)')

        # Read parameters
        initial_pose_str = self.get_parameter('initial_pose').value
        waypoints_str = self.get_parameter('waypoints').value
        final_pose_str = self.get_parameter('final_pose').value

        self.get_logger().info(f'Initial pose: {initial_pose_str}')
        self.get_logger().info(f'Waypoints: {waypoints_str}')
        self.get_logger().info(f'Final pose: {final_pose_str}')

        # Parse with SINGLE function
        self.initial_pose = self._parse_points(initial_pose_str)
        self.waypoints = self._parse_points(waypoints_str)
        self.final_pose = self._parse_points(final_pose_str)

        # Validate
        if not isinstance(self.initial_pose, tuple):
            raise RuntimeError("Initial pose must be a tuple (x,y,yaw).")

        if not isinstance(self.final_pose, tuple):
            raise RuntimeError("Final pose must be a tuple (x,y,yaw).")

        if not isinstance(self.waypoints, list):
            raise RuntimeError("Waypoints must be a list of tuples.")

    # ------------------------------------------------------------
    # SINGLE FUNCTION FOR PARSING
    # ------------------------------------------------------------
    def _parse_points(self, input_str):
        """
        Converts:
          '(x, y, yaw)'   -> tuple
          '[(x,y,yaw), ...]' -> list of tuples
          '[]' -> empty list
        """
        try:
            data = ast.literal_eval(input_str)
        except Exception as e:
            self.get_logger().error(f'Error parsing parameter "{input_str}": {e}')
            raise

        # CASE A: a single pose -> must be tuple of length 3
        if isinstance(data, tuple):
            if len(data) != 3:
                raise ValueError(f"Tuple must have 3 elements. Got: {data}")
            return (float(data[0]), float(data[1]), float(data[2]))

        # CASE B: list of poses
        if isinstance(data, list):
            parsed_list = []
            for item in data:
                if not (isinstance(item, tuple) and len(item) == 3):
                    raise ValueError(f"Each waypoint must be a tuple (x,y,yaw). Got: {item}")
                parsed_list.append((float(item[0]), float(item[1]), float(item[2])))
            return parsed_list

        # Otherwise invalid format
        raise ValueError(f"Invalid parameter format: {input_str}")

    # ------------------------------------------------------------
    # POSE CREATION
    # ------------------------------------------------------------
    def _create_pose_stamped(self, x, y, yaw):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w

        return pose

    # ------------------------------------------------------------
    # NAVIGATION LOGIC
    # ------------------------------------------------------------
    def set_initial_pose(self):
        x, y, yaw = self.initial_pose
        self.navigator.setInitialPose(self._create_pose_stamped(x, y, yaw))
        self.get_logger().info(f"Initial pose set: {self.initial_pose}")

    def wait_for_nav2(self):
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

    def run_navigation(self):
        final_x, final_y, final_yaw = self.final_pose

        # No waypoints → go directly to final pose
        if len(self.waypoints) == 0:
            self.get_logger().info("No waypoints. Going directly to final pose.")
            goal = self._create_pose_stamped(final_x, final_y, final_yaw)
            self.navigator.goToPose(goal)
        else:
            self.get_logger().info(f"Following {len(self.waypoints)} waypoints and then final pose.")
            pose_list = [
                self._create_pose_stamped(x, y, yaw)
                for (x, y, yaw) in self.waypoints
            ]
            # Add final pose as last waypoint
            pose_list.append(self._create_pose_stamped(final_x, final_y, final_yaw))
            self.navigator.followWaypoints(pose_list)

        # Wait for completion
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)

        result = self.navigator.getResult()
        self.get_logger().info(f"Navigation finished: {result}")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationTask()

    node.set_initial_pose()
    node.wait_for_nav2()
    node.run_navigation()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
