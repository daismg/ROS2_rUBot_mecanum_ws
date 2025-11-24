#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, degrees, radians
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
        self.z_pose = round(self.odom.pose.pose.position.z, 2)
        self.orientation_q = self.odom.pose.pose.orientation
        self.orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        self.rpw = euler_from_quaternion(self.orientation_list)
        self.yaw = self.rpw[2]

    def euclidean_distance(self, goal_odom):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_odom.pose.pose.position.x - self.x_pose), 2) +
                    pow((goal_odom.pose.pose.position.y - self.y_pose), 2))

    def linear_vel(self, goal_odom, constant=0.5):
        """Compute the linear velocity towards the goal."""
        return constant * self.euclidean_distance(goal_odom)

    def steering_angle(self, goal_odom):
        """Compute the steering angle towards the goal."""
        return atan2(goal_odom.pose.pose.position.y - self.y_pose, goal_odom.pose.pose.position.x - self.x_pose)

    def angular_vel(self, goal_odom, constant=5):
        """Compute the angular velocity towards the goal."""
        return constant * (self.steering_angle(goal_odom) - self.yaw)

    def move2pose(self):
        """Moves the robot to the goal."""
        goal_odom = Odometry()
        goal_odom.pose.pose.position.x = self.x_goal
        goal_odom.pose.pose.position.y = self.y_goal

        distance_tolerance = 0.1
        angle_tolerance = 0.1

        vel_msg = Twist()

        while rclpy.ok() and self.euclidean_distance(goal_odom) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_odom)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(goal_odom)

            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info("Distance to target: " + str(round(self.euclidean_distance(goal_odom), 2)))

            self.rate.sleep()

        while rclpy.ok() and abs(self.f_goal - self.yaw) >= angle_tolerance:
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = (self.f_goal - self.yaw) * 0.5

            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info("Orientation error: " + str(round(degrees(self.f_goal - self.yaw), 2)))

            self.rate.sleep()

        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        self.get_logger().info("Goal POSE reached!")

def main(args=None):
    rclpy.init(args=args)
    rubot = Rubot()
    try:
        rubot.move2pose()
    except KeyboardInterrupt:
        rubot.get_logger().info("Shutting down")
    finally:
        rubot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
