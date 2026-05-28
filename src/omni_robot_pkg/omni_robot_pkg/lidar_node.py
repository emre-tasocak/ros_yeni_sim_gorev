"""
LiDAR Driver Node (Real Robot)

Reads from YDLidar X2 over serial port and publishes as sensor_msgs/LaserScan.
In simulation this node is not used; the Gazebo ray sensor plugin publishes /scan directly.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from omni_robot_pkg.lidar_lib.lidar_lib import YDLidarX2


class LidarNode(Node):
    """YDLidar X2 driver ROS2 node."""

    def __init__(self):
        super().__init__('lidar_node')

        # --- Parameters ---
        self.declare_parameter('lidar_port', '/dev/ttyUSB0')
        self.declare_parameter('lidar_min_range', 0.10)   # Internal library filter (10 mm → 10 cm)
        self.declare_parameter('lidar_max_range', 8.0)
        self.declare_parameter('lidar_frame_id', 'laser')
        self.declare_parameter('control_frequency', 20.0)

        port = self.get_parameter('lidar_port').value
        self.min_range = self.get_parameter('lidar_min_range').value
        self.max_range = self.get_parameter('lidar_max_range').value
        self.frame_id = self.get_parameter('lidar_frame_id').value
        freq = self.get_parameter('control_frequency').value

        # --- LaserScan publisher ---
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # --- YDLidar initialization ---
        self.lidar = YDLidarX2(port)
        if not self.lidar.connect():
            self.get_logger().error(f'Could not connect to LiDAR on port {port}!')
            return

        self.lidar.start_scan()
        self.get_logger().info(f'LiDAR connected on {port} and scan started.')

        # Publish timer
        self.timer = self.create_timer(1.0 / freq, self._publish_scan)

    def _publish_scan(self):
        """Reads LiDAR data and publishes as LaserScan."""
        if not self.lidar.available:
            return

        # 360-element distance array (in mm, 0 = no measurement)
        raw = self.lidar.get_data()  # numpy int32 array, shape=(360,)

        # mm → meters, filter invalid values
        ranges = []
        for r_mm in raw:
            r_m = r_mm / 1000.0
            if r_m < self.min_range or r_m > self.max_range or r_mm == self.lidar.out_of_range:
                ranges.append(float('nan'))
            else:
                ranges.append(r_m)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        # 360 degrees, 1 degree resolution
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / 360.0
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / 7.0   # YDLidar X2 ~ 7 Hz
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = ranges

        self.scan_pub.publish(scan)

    def destroy_node(self):
        if self.lidar.is_scanning:
            self.lidar.stop_scan()
        if self.lidar.is_connected:
            self.lidar.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
