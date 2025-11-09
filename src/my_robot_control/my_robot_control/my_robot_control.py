#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # Initialize robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_f = 0.0
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Subscribe to odometry topic
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Create a publisher for velocity commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare and get parameters
        self.declare_parameter('vx', 0.3)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('w', 0.0)
        self.declare_parameter('td', 3.0)
        
        self.vx = self.get_parameter('vx').value
        self.vy = self.get_parameter('vy').value
        self.w = self.get_parameter('w').value
        self.td = self.get_parameter('td').value
        
        # Start timer to control robot motion
        self.timer = self.create_timer(0.1, self.move_robot)

    def odom_callback(self, msg):
        """ Updates robot position from odometry """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_f = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def move_robot(self):
        """ Publishes velocity commands until time limit is reached """
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        
        if elapsed_time < self.td:
            self.get_logger().info(f'Robot moving... Time elapsed: {elapsed_time:.2f} sec')
            vel = Twist()
            vel.linear.x = self.vx
            vel.linear.y = self.vy
            vel.angular.z = self.w
            self.publisher.publish(vel)
        else:
            self.get_logger().warn('Stopping robot')
            self.publisher.publish(Twist())  # Stop the robot
            self.timer.cancel()
            self.get_logger().info(f"Robot stopped")
            #time.sleep(0.1) #add a small delay if needed to give time to finish the get_logger.
            rclpy.try_shutdown() #shutdown the node better than rclpy.shutdown
            #No more get_loggers are permitted

def main():
    rclpy.init()
    rubot1_controller = RobotController()
    try:
        rclpy.spin(rubot1_controller)
    except KeyboardInterrupt:
        # ROS2 is already stopped and I can not execute any more functions
        # if elapsed time is not reached, the robot will not stop
        pass
    finally:
        rubot1_controller.destroy_node()

if __name__ == '__main__':
    main()
