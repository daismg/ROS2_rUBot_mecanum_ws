#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion  # sudo apt install ros-humble-tf-transformations


class ImagePointDistanceNode(Node):
    """
    Node that:
      - Reads a depth image aligned with the RGB image.
      - Uses camera intrinsics (CameraInfo) to project a pixel (u, v) into 3D.
      - Computes distance of that 3D point in the camera frame.
      - Transforms that point to a target frame (e.g. 'odom') using TF2.
      - Computes distance in target frame and camera RPY orientation.
    """

    def __init__(self):
        super().__init__('image_point_distance_node')

        # --- Parameters ---
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('u', 320)   # column (pixel)
        self.declare_parameter('v', 240)   # row (pixel)

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.u = int(self.get_parameter('u').get_parameter_value().integer_value)
        self.v = int(self.get_parameter('v').get_parameter_value().integer_value)

        self.get_logger().info(f"Using depth_topic: {self.depth_topic}")
        self.get_logger().info(f"Using camera_info_topic: {self.camera_info_topic}")
        self.get_logger().info(f"Pixel (u, v): ({self.u}, {self.v})")
        self.get_logger().info(f"Camera frame: {self.camera_frame}")
        self.get_logger().info(f"Target frame: {self.target_frame}")

        # Camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )

    # ------------------------------------------------------------------
    # CameraInfo callback
    # ------------------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        """
        Read intrinsics from CameraInfo (pinhole model).
        """
        # K = [fx 0 cx  0 fy cy  0 0 1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        # We only need it once
        self.camera_info_sub.destroy()
        self.get_logger().info(
            f"Camera intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}"
        )

    # ------------------------------------------------------------------
    # Depth callback
    # ------------------------------------------------------------------
    def depth_callback(self, msg: Image):
        if self.fx is None:
            self.get_logger().warn("Camera intrinsics not received yet. Waiting for CameraInfo...")
            return

        # Check if pixel is inside the image
        if not (0 <= self.v < msg.height and 0 <= self.u < msg.width):
            self.get_logger().error(
                f"Pixel (u={self.u}, v={self.v}) is outside the image size "
                f"(width={msg.width}, height={msg.height})."
            )
            return

        # Convert depth data to numpy (assuming 32FC1 or 16UC1)
        depth = self.get_depth_value(msg, self.u, self.v)
        if depth is None or depth <= 0.0:
            self.get_logger().warn(f"Invalid depth at pixel (u={self.u}, v={self.v}).")
            return

        # 3D point in camera frame (pinhole projection)
        Z = depth
        X = (self.u - self.cx) * Z / self.fx
        Y = (self.v - self.cy) * Z / self.fy

        # Distance in camera frame
        dist_cam = math.sqrt(X*X + Y*Y + Z*Z)

        self.get_logger().info(
            f"Point in {self.camera_frame}: "
            f"X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m, |p|={dist_cam:.3f} m"
        )

        # Prepare PointStamped in camera frame
        point_camera = PointStamped()
        point_camera.header.stamp = msg.header.stamp
        point_camera.header.frame_id = self.camera_frame
        point_camera.point.x = X
        point_camera.point.y = Y
        point_camera.point.z = Z

        # Transform point to target frame (e.g. 'odom')
        try:
            point_target = self.tf_buffer.transform(
                point_camera, self.target_frame, timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform point to {self.target_frame}: {ex}")
            return

        Xo = point_target.point.x
        Yo = point_target.point.y
        Zo = point_target.point.z

        dist_odom = math.sqrt(Xo*Xo + Yo*Yo + Zo*Zo)

        self.get_logger().info(
            f"Point in {self.target_frame}: "
            f"X={Xo:.3f} m, Y={Yo:.3f} m, Z={Zo:.3f} m, |p|={dist_odom:.3f} m"
        )

        # Get camera pose (orientation) w.r.t target frame (RPY)
        try:
            # transform from target_frame -> camera_frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.camera_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"Could not lookup transform {self.target_frame}->{self.camera_frame}: {ex}")
            return

        q = transform.transform.rotation
        # Quaternion -> RPY
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.get_logger().info(
            f"Camera orientation w.r.t {self.target_frame} (RPY in rad): "
            f"R={roll:.3f}, P={pitch:.3f}, Y={yaw:.3f}"
        )

    # ------------------------------------------------------------------
    # Helper: read depth value
    # ------------------------------------------------------------------
    def get_depth_value(self, msg: Image, u: int, v: int):
        """
        Returns depth in meters for pixel (u, v).
        Supports encodings '32FC1' (meters) or '16UC1' (millimeters).
        """
        if msg.encoding == '32FC1':
            dtype = np.float32
            step = 4
            factor = 1.0  # already in meters
        elif msg.encoding == '16UC1':
            dtype = np.uint16
            step = 2
            factor = 0.001  # from mm to meters
        else:
            self.get_logger().error(f"Unsupported depth encoding: {msg.encoding}")
            return None

        # Index in the data buffer
        index = v * msg.step + u * step
        # Use numpy frombuffer
        depth_array = np.frombuffer(msg.data[index:index + step], dtype=dtype)
        if depth_array.size == 0:
            return None

        depth_value = float(depth_array[0]) * factor
        return depth_value


def main(args=None):
    rclpy.init(args=args)
    node = ImagePointDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
