#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import sign
import time #add time module

class RobotSelfControl(Node):

    def __init__(self):
        super().__init__('robot_selfcontrol_node')
        self.declare_parameter('distance_laser', 0.3)
        self.declare_parameter('speed_factor', 1.0)
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('backward_speed', -0.2)
        self.declare_parameter('rotation_speed', 0.3)
        self.declare_parameter('time_to_stop', 5.0)
        
        self._distanceLaser = self.get_parameter('distance_laser').value
        self._speedFactor = self.get_parameter('speed_factor').value
        self._forwardSpeed = self.get_parameter('forward_speed').value
        self._backwardSpeed = self.get_parameter('backward_speed').value
        self._rotationSpeed = self.get_parameter('rotation_speed').value
        self._time_to_stop = self.get_parameter('time_to_stop').value

        self._msg = Twist()
        self._msg.linear.x = self._forwardSpeed * self._speedFactor
        self._msg.angular.z = 0.0
        
        self._cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.Laser_callback, 10)
        
        self.__isScanRangesLengthCorrectionFactorCalculated = False
        self.__scanRangesLengthCorrectionFactor = 2

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self._shutting_down = False #flag to ensure a proper shutdown

    def timer_callback(self):
        if self._shutting_down: #check flag
            return
        elapsed_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        self._cmdVel.publish(self._msg)
        self.get_logger().info(f"Vx: {self._msg.linear.x:.2f} and w: {self._msg.angular.z:.2f} in time: {elapsed_time:.1f}")
        
        if elapsed_time >= self._time_to_stop:
            self.stop()
            self.timer.cancel()
            self.get_logger().info(f"Robot stopped")
            #time.sleep(0.1) #add a small delay if needed to give time to finish the get_logger.
            rclpy.try_shutdown() #shutdown the node better than rclpy.shutdown
            #No more get_loggers are permitted

    def Laser_callback(self, scan):
        if self._shutting_down: #check flag
            return
        if not self.__isScanRangesLengthCorrectionFactorCalculated:
            self.__scanRangesLengthCorrectionFactor = int(len(scan.ranges) / 360)
            self.__isScanRangesLengthCorrectionFactorCalculated = True
            self.get_logger().info(f"Lidar beams/degre factor: {self.__scanRangesLengthCorrectionFactor:.0f}")

        closestDistance, elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges)
            if scan.range_min < val < scan.range_max
        )
        angleClosestDistance = elementIndex / self.__scanRangesLengthCorrectionFactor
        #rubot.get_logger().info(f"Angle Closest Distance (raw): {angleClosestDistance:.0f} degrees")
        # To take into account the Lidar Zero angle is on the back
        angleClosestDistance = angleClosestDistance - 180
        # To wrapp the angle to the ranege [+180, -180]
        angleClosestDistance = (angleClosestDistance + 180) % 360 - 180
        #rubot.get_logger().info(f"Angle Closest Distance (wrapped): {angleClosestDistance:.2f} degrees")
        #rubot.get_logger().info(f"Closest distance: {closestDistance:.2f} meters and Angle: {angleClosestDistance:.1f} degrees")

        if closestDistance < self._distanceLaser and -80 < angleClosestDistance < 80:
            self._msg.linear.x = self._backwardSpeed * self._speedFactor
            self._msg.angular.z = -sign(angleClosestDistance) * self._rotationSpeed * self._speedFactor
        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.angular.z = 0.0

    def stop(self):
        self._shutting_down = True
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self._cmdVel.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    rubot = RobotSelfControl()
    try:
        rclpy.spin(rubot)
    except KeyboardInterrupt:
        # ROS2 is already stopped and I can not execute any more functions
        # if elapsed time is not reached, the robot will not stop
        pass
    finally:
        rubot.destroy_node()

if __name__ == '__main__':
    main()