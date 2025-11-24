#!/usr/bin/env python3
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolov8Inference
bridge = CvBridge()
import cv2
import time  

class YoloObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__('object_detection')

        # Load a pre-trained YOLOv8 object detection model
        self.model = YOLO('/home/agilex/ROS2_rUBot_mecanum_ws/src/AI_Projects/my_robot_ai_identification/models/yolov8n_custom.pt') 
        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        # He puesto un publisher del cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def camera_callback(self, msg: Image) -> None:
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        img_resized = cv2.resize(img, (640, 640))
        results = self.model(img_resized) 

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        detected_signs = []

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inf_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  
                c = box.cls
                class_name = self.model.names[int(c)]
                self.inf_result.class_name = class_name
                detected_signs.append(class_name)

                self.inf_result.left = int(b[0])
                self.inf_result.top = int(b[1])
                self.inf_result.right = int(b[2])
                self.inf_result.bottom = int(b[3])
                self.inf_result.box_width = (self.inf_result.right - self.inf_result.left) 
                self.inf_result.box_height = (self.inf_result.bottom - self.inf_result.top)
                self.inf_result.x = self.inf_result.left + (self.inf_result.box_width / 2.0)
                self.inf_result.y = self.inf_result.top + (self.inf_result.box_height / 2.0)
                self.yolov8_inference.yolov8_inference.append(self.inf_result)

        # La lógica de reacción a señales (Hay que mirar si genera conflicto con el NAV)
        if "Prohibido" in detected_signs:
            self.get_logger().info("Prohibido detectado: paro total")
            self.stop_robot()
            rclpy.shutdown()

        elif "STOP" in detected_signs:
            self.get_logger().info("Stop detectado: paro 3 segundos")
            self.stop_robot()
            time.sleep(3)

        elif "Ceda" in detected_signs:
            self.get_logger().info("Ceda detectado: velocidad reducida 3 segundos")
            self.publish_velocity(0.05, 0.0)
            time.sleep(3)
            self.stop_robot()

        elif "Derecha" in detected_signs:
            self.get_logger().info("Derecha detectado: giro a la derecha")
            self.publish_velocity(0.0, -0.5)
            time.sleep(1)
            self.stop_robot()

        elif "Izquierda" in detected_signs:
            self.get_logger().info("Izquierda detectado: giro a la izquierda")
            self.publish_velocity(0.0, 0.5)
            time.sleep(1)
            self.stop_robot()

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

    #Funciones auxiliares
    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

def main(args=None) -> None:
    rclpy.init(args=args)
    object_detection = YoloObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
