#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('distance_laser', 0.5)
        self.base_distance = self.get_parameter('distance_laser').value
        self.tolerance = 0.05

        # Subscriptions and publications
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10  # Default QoS depth
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.log_info)

        self.closest = (float('inf'), 'unknown')

    def laser_callback(self, scan):
        angle_min_deg = scan.angle_min * 180.0 / 3.14159
        angle_increment_deg = scan.angle_increment * 180.0 / 3.14159

        front_distances = []
        right_distances = []

        for i, distance in enumerate(scan.ranges):
            angle_deg = angle_min_deg + i * angle_increment_deg
            if distance == float('inf') or distance == 0.0:
                continue
            if -15 <= angle_deg <= 15:
                front_distances.append(distance)
            elif -110 <= angle_deg < -15:
                right_distances.append(distance)

        min_front = min(front_distances) if front_distances else float('inf')
        min_right = min(right_distances) if right_distances else float('inf')

        self.closest = (min(min_front, min_right), 'front' if min_front < min_right else 'right')

        twist = Twist()

        if min_front < self.base_distance:
            twist.angular.z = 0.3
            action = "Turning left: obstacle ahead"
        elif min_right < self.base_distance - self.tolerance:
            twist.angular.z = 0.3
            action = "Too close to wall: turning left"
        elif min_right > self.base_distance + self.tolerance:
            twist.angular.z = -0.3
            action = "Too far from wall: turning right"
        else:
            twist.linear.x = 0.2
            action = "Following wall"

        self.get_logger().info(action)
        self.publisher.publish(twist)

    def log_info(self):
        dist, region = self.closest
        self.get_logger().info(f"Closest object at {dist:.2f} m in region: {region.upper()}")


def main(args=None):
    rclpy.init(args=args)
    rubot = WallFollower()
    rclpy.spin(rubot)
    rubot.destroy_node()
    rclpy.shutdown()