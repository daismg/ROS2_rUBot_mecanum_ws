#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolov8Inference

from ament_index_python.packages import get_package_share_directory
import os
import math
import yaml


class YoloObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # ------------------- Parameters -------------------
        # Aquests valors són defaults; si el YAML els conté, ROS 2 els sobreescriu
        self.declare_parameter('modelYolo', 'yolov8n_custom.pt')
        self.declare_parameter('topic', '/image_raw')
        self.declare_parameter('twist_frequency', 20.0)

        self.declare_parameter('front_distance', 1.0)
        self.declare_parameter('sign_positions', '')  # ROS2 always stores it as a string
        
        # Read parameters
        model_file = self.get_parameter('modelYolo').value
        self.image_topic = self.get_parameter('topic').value
        sign_positions_str = self.get_parameter('sign_positions').value
        # Translate in dictionary
        self.sign_positions = yaml.safe_load(sign_positions_str)
        if self.sign_positions is None:
            self.sign_positions = {}

        self.twist_freq = float(self.get_parameter('twist_frequency').value)
        if self.twist_freq <= 0.0:
            self.get_logger().warn("twist_frequency <= 0, using 20.0 Hz")
            self.twist_freq = 20.0
        self.twist_period = 1.0 / self.twist_freq

        self.front_distance = float(self.get_parameter('front_distance').value)

        # ------------------- Model path (portable) -------------------
        package_path = get_package_share_directory('my_robot_ai_identification')
        self.model_path = os.path.join(package_path, 'models', model_file)

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"YOLO model not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)

        self.get_logger().info(f"Loaded YOLO model: {self.model_path}")
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info(f"front_distance: {self.front_distance} m")
        self.get_logger().info(f"twist_frequency: {self.twist_freq} Hz")
        self.get_logger().info(f"sign_positions: {self.sign_positions}")

        # ------------------- Setup -------------------
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)

        # Camera subscriber with sensor QoS
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.camera_callback,
            qos_profile_sensor_data
        )

        # Odom subscriber (for proximity gate)
        self.robot_x = None
        self.robot_y = None
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        # Direct override of Nav2 by frequency
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Action handling
        self.current_twist = None
        self.action_end_time = None
        self.timer = self.create_timer(self.twist_period, self.timer_cb)

    # ------------------------------------------------------------------
    # ODOM
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def get_sign_distance(self, sign_name: str):
        """Current approximation:
           distance from robot (odom) to known sign position (map/odom frame).
           In the future, replace this with depth-based distance."""
        if self.robot_x is None or self.robot_y is None:
            return None

        if sign_name not in self.sign_positions:
            return None

        sx, sy = self.sign_positions[sign_name]
        dx = sx - self.robot_x
        dy = sy - self.robot_y
        return math.sqrt(dx*dx + dy*dy)

    def should_react(self, sign_name: str) -> bool:
        d = self.get_sign_distance(sign_name)
        if d is None:
            return False
        return d <= self.front_distance

    # ------------------------------------------------------------------
    # CAMERA
    def camera_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model(img)

        yolov8_msg = Yolov8Inference()
        yolov8_msg.header.frame_id = "inference"
        yolov8_msg.header.stamp = self.get_clock().now().to_msg()

        detected_signs = []

        for r in results:
            for box in r.boxes:
                inf = InferenceResult()
                b = box.xyxy[0].to('cpu').numpy()
                c = int(box.cls)
                class_name = self.model.names[c]

                detected_signs.append(class_name)

                inf.class_name = class_name
                inf.left, inf.top, inf.right, inf.bottom = map(int, b)
                inf.box_width = inf.right - inf.left
                inf.box_height = inf.bottom - inf.top
                inf.x = inf.left + inf.box_width / 2.0
                inf.y = inf.top + inf.box_height / 2.0

                yolov8_msg.yolov8_inference.append(inf)

        # React to detected traffic signs (only if near)
        self.handle_signs(detected_signs)

        # Publish annotated image safely
        if results:
            annotated = results[0].plot()
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.img_pub.publish(img_msg)

        self.yolov8_pub.publish(yolov8_msg)

    # ------------------------------------------------------------------
    # SIGN LOGIC
    def handle_signs(self, detected_signs):
        now = self.get_clock().now().nanoseconds / 1e9

        def T(vx=0.0, vy=0.0, wz=0.0):
            t = Twist()
            t.linear.x = float(vx)
            t.linear.y = float(vy)
            t.angular.z = float(wz)
            return t

        # If an action is already running, don't override it
        if self.current_twist is not None:
            return

        if "Prohibido" in detected_signs and self.should_react("Prohibido"):
            self.get_logger().info("Prohibido near → PARO total")
            self.current_twist = T(0.0, 0.0, 0.0)
            self.action_end_time = now + 9999.0

        elif "STOP" in detected_signs and self.should_react("STOP"):
            self.get_logger().info("STOP near → paro 3 s")
            self.current_twist = T(0.0, 0.0, 0.0)
            self.action_end_time = now + 3.0

        elif "Ceda" in detected_signs and self.should_react("Ceda"):
            self.get_logger().info("Ceda near → avanç lent 3 s")
            self.current_twist = T(0.05, 0.0, 0.0)
            self.action_end_time = now + 3.0

        elif "Derecha" in detected_signs and self.should_react("Derecha"):
            self.get_logger().info("Derecha near → gir a la dreta 1 s")
            self.current_twist = T(0.0, 0.0, -0.5)
            self.action_end_time = now + 1.0

        elif "Izquierda" in detected_signs and self.should_react("Izquierda"):
            self.get_logger().info("Izquierda near → gir a l'esquerra 1 s")
            self.current_twist = T(0.0, 0.0, 0.5)
            self.action_end_time = now + 1.0

        if self.current_twist:
            # Publish immediately once
            self.cmd_vel_pub.publish(self.current_twist)

    # ------------------------------------------------------------------
    # TIMER (priority over Nav2 by higher freq)
    def timer_cb(self):
        if self.current_twist is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Override Nav2 while action is active
        self.cmd_vel_pub.publish(self.current_twist)

        # End action
        if now >= self.action_end_time:
            self.current_twist = None
            self.action_end_time = None


def main(args=None):
    rclpy.init(args=args)
    node = YoloObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
