#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


class NavigationTask(Node):
    def __init__(self):
        super().__init__('custon_nav2')

        # --- Nav2 Simple Commander ---
        self.navigator = BasicNavigator()

        # --- Parameters ---
        # [x, y, yaw]
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('target_pose', [5.0, 3.0, 0.0])
        self.declare_parameter('waypoint', [2.0, 1.0, 0.0])
        self.declare_parameter('wait_for_traffic_wp', 5.0)  # seconds

        self.initial_pose_xyz = self.get_parameter('initial_pose').value
        self.target_pose_xyz = self.get_parameter('target_pose').value
        self.waypoint_xyz = self.get_parameter('waypoint').value
        self.wait_for_traffic_wp = float(self.get_parameter('wait_for_traffic_wp').value)

        self.get_logger().info(f"initial_pose: {self.initial_pose_xyz}")
        self.get_logger().info(f"target_pose:  {self.target_pose_xyz}")
        self.get_logger().info(f"waypoint:     {self.waypoint_xyz}")
        self.get_logger().info(f"wait_for_traffic_wp: {self.wait_for_traffic_wp:.1f} s")

        # --- Waypoint proporcionat pel YOLO (PoseStamped) ---
        self.traffic_waypoint = None

        # Subscriure al waypoint que ve del node YOLO
        self.wp_sub = self.create_subscription(
            PoseStamped,
            '/traffic_waypoint',
            self.traffic_waypoint_callback,
            10
        )

    # ------------------------------------------------------------------
    # Helpers
    def create_pose_stamped(self, position_x, position_y, orientation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(
            0.0, 0.0, orientation_z
        )
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(position_x)
        pose.pose.position.y = float(position_y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        return pose

    def set_initial_pose_from_params(self):
        x, y, yaw = self.initial_pose_xyz
        initial_pose = self.create_pose_stamped(x, y, yaw)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(f"Set initial pose to: x={x}, y={y}, yaw={yaw}")

    def wait_for_nav2(self):
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active and ready for navigation.')

    # ------------------------------------------------------------------
    # SUB: /traffic_waypoint (del node YOLO)
    def traffic_waypoint_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f"Received /traffic_waypoint from YOLO: "
            f"x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}"
        )
        self.traffic_waypoint = msg

    # ------------------------------------------------------------------
    # Selecciona waypoint (YOLO si n’hi ha, sinó YAML)
    def get_selected_waypoint_pose(self) -> PoseStamped:
        if self.traffic_waypoint is not None:
            self.get_logger().info("Using traffic waypoint from YOLO (overriding YAML waypoint).")
            return self.traffic_waypoint
        else:
            x, y, yaw = self.waypoint_xyz
            self.get_logger().info("No /traffic_waypoint received, using YAML waypoint.")
            return self.create_pose_stamped(x, y, yaw)

    # ------------------------------------------------------------------
    # goToPose amb PoseStamped
    def go_to_pose(self, pose: PoseStamped):
        self.navigator.goToPose(pose)
        self.get_logger().info(
            f"Going to pose: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
        )

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # Si vols més info, descomenta:
            # if feedback is not None and hasattr(feedback, 'distance_remaining'):
            #     self.get_logger().debug(
            #         f"Distance remaining: {feedback.distance_remaining:.2f} m"
            #     )

        result = self.navigator.getResult()
        self.get_logger().info(f"Navigation finished with result: {result}")
        return result


def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationTask()

    # --- Posició inicial ---
    navigation_node.set_initial_pose_from_params()

    # --- Esperar Nav2 ---
    navigation_node.wait_for_nav2()

    # --- Esperar opcionalment un waypoint del YOLO ---
    navigation_node.get_logger().info(
        f"Waiting up to {navigation_node.wait_for_traffic_wp:.1f} s "
        f"for /traffic_waypoint override (optional)..."
    )
    start = navigation_node.get_clock().now().nanoseconds / 1e9
    while rclpy.ok():
        now = navigation_node.get_clock().now().nanoseconds / 1e9
        if (now - start) >= navigation_node.wait_for_traffic_wp:
            break
        rclpy.spin_once(navigation_node, timeout_sec=0.1)

    # --- 1) Anar al waypoint (YOLO o YAML) ---
    wp_pose = navigation_node.get_selected_waypoint_pose()
    navigation_node.get_logger().info("Navigating to intermediate waypoint...")
    navigation_node.go_to_pose(wp_pose)

    # --- 2) Anar al destí final ---
    x_t, y_t, yaw_t = navigation_node.target_pose_xyz
    target_pose = navigation_node.create_pose_stamped(x_t, y_t, yaw_t)
    navigation_node.get_logger().info("Navigating to final target pose...")
    navigation_node.go_to_pose(target_pose)

    navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
