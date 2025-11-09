import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarTest(Node):

    def __init__(self):
        super().__init__('lidar_test_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.scan_msg_shown = False
        self.last_print_time = self.get_clock().now().seconds_nanoseconds()[0]

    def listener_callback(self, scan):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_print_time < 1:
            return  # Skip printing if less than 1 second has passed

        angle_min_deg = scan.angle_min * 180.0 / 3.14159
        angle_max_deg = scan.angle_max * 180.0 / 3.14159
        range_deg = angle_max_deg - angle_min_deg
        angle_increment_deg = scan.angle_increment * 180.0 / 3.14159
        total_points = len(scan.ranges)

        if not self.scan_msg_shown:
            self.get_logger().info(f"Min angle: {angle_min_deg:.2f} deg")
            self.get_logger().info(f"Max angle: {angle_max_deg:.2f} deg")
            self.get_logger().info(f"Total lidar range angle: {range_deg:.2f} deg")
            self.get_logger().info(f"Delta angle per index: {angle_increment_deg:.2f} deg")
            self.get_logger().info(f"Total number of Laser Beams: {total_points}")
            self.scan_msg_shown = True

        # Indices for specific angles
        index_0_deg = int((0.0 - angle_min_deg) / angle_increment_deg)
        index_neg90_deg = int((-90.0 - angle_min_deg) / angle_increment_deg)
        index_pos90_deg = int((90.0 - angle_min_deg) / angle_increment_deg)

        custom_range = []

        for i, distance in enumerate(scan.ranges):
            angle_deg = angle_min_deg + i * angle_increment_deg
            if distance == float('inf') or distance == 0.0:
                continue
            if -150 <= angle_deg <= 150:
                custom_range.append((distance, i))

        if custom_range:
            closest_distance, element_index = min(custom_range)
            angle_closest_distance = angle_min_deg + element_index * angle_increment_deg

            dist_0_deg = scan.ranges[index_0_deg]
            dist_neg90_deg = scan.ranges[index_neg90_deg]
            dist_pos90_deg = scan.ranges[index_pos90_deg]

            self.get_logger().info("---- Current LIDAR readings ----")
            self.get_logger().info(f"Distance at 0°: {dist_0_deg:.2f} m" if dist_0_deg else "No valid reading at 0°")
            self.get_logger().info(f"Distance at -90°: {dist_neg90_deg:.2f} m" if dist_neg90_deg else "No valid reading at -90°")
            self.get_logger().info(f"Distance at +90°: {dist_pos90_deg:.2f} m" if dist_pos90_deg else "No valid reading at +90°")
            self.get_logger().info(f"Minimum distance: {closest_distance:.2f} m at angle {angle_closest_distance:.2f}°")
        else:
            self.get_logger().info("No valid readings in range [-150°, 150°]")

        self.last_print_time = current_time


def main(args=None):
    rclpy.init(args=args)
    lidar1_test = LidarTest()
    rclpy.spin(lidar1_test)
    lidar1_test.destroy_node()
    rclpy.shutdown()