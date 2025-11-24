#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, degrees, radians, sin, cos # Importem sin i cos
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class Rubot(Node):

    def __init__(self):
        super().__init__('rubot_control_node')

        # Define goal odometry from parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('f', 0.0)

        self.x_goal = self.get_parameter('x').value
        self.y_goal = self.get_parameter('y').value
        self.f_goal = radians(self.get_parameter('f').value)
        self.q_goal = quaternion_from_euler(0, 0, self.f_goal)

        # Define initial values for actual odometry (read in callback function)
        self.x_pose = 0.0
        self.y_pose = 0.0
        self.yaw = 0.0

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Odometry is received.
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.update_odom, 10)

        self.odom = Odometry()
        self.rate = self.create_rate(10)

    def update_odom(self, data):
        """Callback function which is called when a new message of type Odometry is received by the subscriber."""
        self.odom = data
        self.x_pose = round(self.odom.pose.pose.position.x, 2)
        self.y_pose = round(self.odom.pose.pose.position.y, 2)
        self.orientation_q = self.odom.pose.pose.orientation
        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.position.x - self.x_pose), 2) +
                    pow((goal_pose.position.y - self.y_pose), 2))

    def linear_vel(self, goal_pose, constant=0.5):
        """Compute the linear velocity towards the goal."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """Compute the steering angle towards the goal."""
        return atan2(goal_pose.position.y - self.y_pose, goal_pose.position.x - self.x_pose)

    def angular_vel(self, goal_pose, constant=5.0):
        """Compute the angular velocity towards the goal."""
        angle_error = self.steering_angle(goal_pose) - self.yaw
        # --- CORRECCIÓ CLAU ---
        # Normalitzem l'error per agafar sempre el camí més curt
        angle_error_normalized = atan2(sin(angle_error), cos(angle_error))
        return constant * angle_error_normalized

    def move2pose(self):
        """Moves the robot to the goal."""
        goal_pose = Pose()
        goal_pose.position.x = self.x_goal
        goal_pose.position.y = self.y_goal

        distance_tolerance = 0.1
        angle_tolerance = 0.1 # en radians (~5.7 graus)

        vel_msg = Twist()

        # Bucle principal: va cap a la posició
        while rclpy.ok() and self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info(f"Distance to target: {self.euclidean_distance(goal_pose):.2f}")

            self.rate.sleep()
        
        # Atura el moviment lineal un cop arribem a la posició
        vel_msg.linear.x = 0.0
        self.velocity_publisher.publish(vel_msg)

        # Bucle secundari: corregeix l'orientació final
        orientation_error = self.f_goal - self.yaw
        while rclpy.ok() and abs(atan2(sin(orientation_error), cos(orientation_error))) >= angle_tolerance:
            orientation_error = self.f_goal - self.yaw
            # Normalitzem també l'error d'orientació final
            normalized_error = atan2(sin(orientation_error), cos(orientation_error))
            
            vel_msg.angular.z = normalized_error * 2.0 # Guany més suau per a la correcció final

            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info(f"Orientation error: {degrees(normalized_error):.2f} degrees")
            
            self.rate.sleep()

        # Atura completament el robot
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        self.get_logger().info("Goal POSE reached!")


def main(args=None):
    rclpy.init(args=args)
    rubot = Rubot()
    try:
        # Esperem un segon per asegurar que la subscripció a /odom s'ha establert
        rclpy.spin_once(rubot, timeout_sec=1.0)
        rubot.move2pose()
    except KeyboardInterrupt:
        rubot.get_logger().info("Shutting down")
    finally:
        # Atura el robot abans de sortir
        stop_msg = Twist()
        rubot.velocity_publisher.publish(stop_msg)
        rubot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()