"""
LiDAR Processing Node

- Filters raw /scan data (below 30 cm = wheel protection)
- Publishes nearest obstacle distance → /nearest_obstacle
- Publishes obstacle detection flag → /obstacle_detected
- Publishes filtered scan as /scan/filtered

LiDAR is used ONLY for obstacle detection.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool


class LidarProcessorNode(Node):
    """LiDAR data processing ROS2 node."""

    def __init__(self):
        super().__init__('lidar_processor')

        # --- Parameters ---
        self.declare_parameter('lidar_min_range', 0.30)   # 30 cm: wheel filter
        self.declare_parameter('lidar_max_range', 8.0)
        self.declare_parameter('obstacle_safety_radius', 0.50)
        self.declare_parameter('lidar_frame_id', 'laser')

        self.min_range = self.get_parameter('lidar_min_range').value
        self.max_range = self.get_parameter('lidar_max_range').value
        self.safety_radius = self.get_parameter('obstacle_safety_radius').value
        self.frame_id = self.get_parameter('lidar_frame_id').value

        # --- Publishers ---
        self.filtered_pub = self.create_publisher(LaserScan, '/scan/filtered', 10)
        self.nearest_pub = self.create_publisher(Float32, '/nearest_obstacle', 10)
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        # --- Subscriber ---
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10)

        self.get_logger().info('LiDAR processing node started.')
        self.get_logger().info(
            f'Filter range: [{self.min_range:.2f}m, {self.max_range:.2f}m]'
        )

    def _scan_callback(self, msg: LaserScan):
        """Processes raw LiDAR data."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        angles = np.array([
            msg.angle_min + i * msg.angle_increment
            for i in range(n)
        ], dtype=np.float32)

        # --- Filtering ---
        # Set readings below 30 cm (wheels) and above max to NaN
        filtered = np.where(
            (ranges >= self.min_range) & (ranges <= self.max_range),
            ranges,
            float('nan')
        )

        # --- Publish filtered scan ---
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = self.min_range
        filtered_scan.range_max = self.max_range
        filtered_scan.ranges = filtered.tolist()
        self.filtered_pub.publish(filtered_scan)

        # --- Valid (non-NaN) indices ---
        valid_mask = np.isfinite(filtered)
        if not np.any(valid_mask):
            return

        valid_ranges = filtered[valid_mask]
        valid_angles = angles[valid_mask]



        # --- Nearest obstacle ---
        nearest_dist = float(np.min(valid_ranges))
        near_msg = Float32()
        near_msg.data = nearest_dist
        self.nearest_pub.publish(near_msg)

        # Is there an obstacle within safety radius?
        danger = Bool()
        danger.data = nearest_dist < self.safety_radius
        self.obstacle_detected_pub.publish(danger)

        if danger.data:
            self.get_logger().warn(
                f'OBSTACLE! Nearest object: {nearest_dist:.2f} m '
                f'(safety limit: {self.safety_radius:.2f} m)',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
