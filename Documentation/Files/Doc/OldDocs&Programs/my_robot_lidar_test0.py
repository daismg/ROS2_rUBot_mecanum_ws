#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanValues(Node):

    def __init__(self):
        super().__init__('lidar_test_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10)  # QoS history depth
        self.subscription  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info(f"Number of scan points: {len(msg.ranges)}")
        # values at 0 degrees (aproximat)
        if len(msg.ranges) > 0:
            self.get_logger().info(f"Distance at 0deg (approx): {msg.ranges[0]}")
        # values at 90 degrees (aproximat)
        if len(msg.ranges) > 180:
            self.get_logger().info(f"Distance at 90deg (approx): {msg.ranges[180]}")
        # values at 180 degrees (aproximat)
        if len(msg.ranges) > 360:
            self.get_logger().info(f"Distance at 180deg (approx): {msg.ranges[360]}")
        # values at 270 degrees (aproximat)
        if len(msg.ranges) > 540:
            self.get_logger().info(f"Distance at 270deg (approx): {msg.ranges[540]}")
        # values at 360 degrees (aproximat)
        if len(msg.ranges) > 719:
            self.get_logger().info(f"Distance at 360deg (approx): {msg.ranges[719]}")

def main(args=None):
    rclpy.init(args=args)

    scan_values = ScanValues()

    rclpy.spin(scan_values)

    # Destroy the node explicitly
    # (optional - done automatically upon garbage collection)
    scan_values.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()