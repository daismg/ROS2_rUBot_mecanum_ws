#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, degrees, radians, sin, cos
from tf_transformations import euler_from_quaternion

class Rubot(Node):

    def __init__(self):
        super().__init__('rubot_control_node')
        
        # Declaració i obtenció de paràmetres
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('f', 0.0)
        self.x_goal = self.get_parameter('x').value
        self.y_goal = self.get_parameter('y').value
        self.f_goal = radians(self.get_parameter('f').value)
        
        # Variables de posició
        self.x_pose = 0.0
        self.y_pose = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # Publicadors i subscriptors
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.update_odom, 10)
        self.rate = self.create_rate(10)

    def update_odom(self, data):
        """Actualitza la posició i orientació del robot."""
        self.x_pose = data.pose.pose.position.x
        self.y_pose = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('Odometria rebuda per primer cop.')

    def stop_robot(self):
        """Envia un missatge de velocitat zero per aturar el robot."""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)

    def move2pose(self):
        """Mou el robot a la posició i orientació desitjades en 3 fases."""
        
        # Espera fins a tenir una lectura d'odometria vàlida
        while not self.odom_received and rclpy.ok():
            self.get_logger().info('Esperant odometria...')
            rclpy.spin_once(self, timeout_sec=1.0)

        goal_pose = Pose()
        goal_pose.position.x = self.x_goal
        goal_pose.position.y = self.y_goal

        distance_tolerance = 0.15  # Tolerància de distància
        angle_tolerance = 0.1    # Tolerància d'angle en radians

        vel_msg = Twist()

        # --- FASE 1: Girar per encarar l'objectiu (X, Y) ---
        self.get_logger().info('FASE 1: Girant per encarar l\'objectiu.')
        target_angle = atan2(goal_pose.position.y - self.y_pose, goal_pose.position.x - self.x_pose)
        angle_error = target_angle - self.yaw
        
        while rclpy.ok() and abs(atan2(sin(angle_error), cos(angle_error))) >= angle_tolerance:
            angle_error = target_angle - self.yaw
            normalized_error = atan2(sin(angle_error), cos(angle_error))
            
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5 * normalized_error # Guany per al gir
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        
        self.stop_robot()
        self.get_logger().info('FASE 1 completada.')

        # --- FASE 2: Avançar cap a l'objectiu ---
        self.get_logger().info('FASE 2: Avançant cap a l\'objectiu.')
        while rclpy.ok() and sqrt(pow(goal_pose.position.x - self.x_pose, 2) + pow(goal_pose.position.y - self.y_pose, 2)) >= distance_tolerance:
            # Guany (velocitat) proporcional a la distància
            vel_msg.linear.x = 0.5 * sqrt(pow(goal_pose.position.x - self.x_pose, 2) + pow(goal_pose.position.y - self.y_pose, 2))
            
            # Petita correcció de rumb mentre avança
            target_angle = atan2(goal_pose.position.y - self.y_pose, goal_pose.position.x - self.x_pose)
            angle_error = target_angle - self.yaw
            normalized_error = atan2(sin(angle_error), cos(angle_error))
            vel_msg.angular.z = 0.5 * normalized_error
            
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            
        self.stop_robot()
        self.get_logger().info('FASE 2 completada.')

        # --- FASE 3: Girar per assolir l'orientació final ---
        self.get_logger().info('FASE 3: Ajustant orientació final.')
        orientation_error = self.f_goal - self.yaw
        
        while rclpy.ok() and abs(atan2(sin(orientation_error), cos(orientation_error))) >= angle_tolerance:
            orientation_error = self.f_goal - self.yaw
            normalized_error = atan2(sin(orientation_error), cos(orientation_error))

            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5 * normalized_error # Guany més suau per al gir final
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        self.stop_robot()
        self.get_logger().info("Objectiu (POSE) assolit!")

def main(args=None):
    rclpy.init(args=args)
    rubot = Rubot()
    try:
        rubot.move2pose()
    except KeyboardInterrupt:
        rubot.get_logger().info("Aturant el node per l'usuari.")
    finally:
        rubot.stop_robot()
        rubot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()