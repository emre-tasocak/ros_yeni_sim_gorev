"""
LiDAR İşleme Düğümü

- Ham /scan verisini filtreler (30 cm altı = tekerlek koruması)
- En uzak geçerli noktayı bulur → /farthest_point yayınlar
- En yakın engel mesafesini yayınlar → /nearest_obstacle yayınlar
- Filtrelenmiş taramayı /scan/filtered olarak yayınlar
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Bool


class LidarProcessorNode(Node):
    """LiDAR veri işleme ROS2 düğümü."""

    def __init__(self):
        super().__init__('lidar_processor')

        # --- Parametreler ---
        self.declare_parameter('lidar_min_range', 0.30)   # 30 cm: teker filtresi
        self.declare_parameter('lidar_max_range', 8.0)
        self.declare_parameter('obstacle_safety_radius', 0.50)
        self.declare_parameter('lidar_frame_id', 'laser')

        self.min_range = self.get_parameter('lidar_min_range').value
        self.max_range = self.get_parameter('lidar_max_range').value
        self.safety_radius = self.get_parameter('obstacle_safety_radius').value
        self.frame_id = self.get_parameter('lidar_frame_id').value

        # --- Yayıncılar ---
        self.filtered_pub = self.create_publisher(LaserScan, '/scan/filtered', 10)
        self.farthest_pub = self.create_publisher(PointStamped, '/farthest_point', 10)
        self.nearest_pub = self.create_publisher(Float32, '/nearest_obstacle', 10)
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        # --- Abone ---
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10)

        self.get_logger().info('LiDAR işleme düğümü başlatıldı.')
        self.get_logger().info(
            f'Filtre aralığı: [{self.min_range:.2f}m, {self.max_range:.2f}m]'
        )

    def _scan_callback(self, msg: LaserScan):
        """Ham LiDAR verisini işler."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        angles = np.array([
            msg.angle_min + i * msg.angle_increment
            for i in range(n)
        ], dtype=np.float32)

        # --- Filtreleme ---
        # 30 cm altı (tekerlek) ve max üstü değerleri NaN yap
        filtered = np.where(
            (ranges >= self.min_range) & (ranges <= self.max_range),
            ranges,
            float('nan')
        )

        # --- Filtrelenmiş tarama yayınla ---
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

        # --- Geçerli (NaN olmayan) indeksler ---
        valid_mask = np.isfinite(filtered)
        if not np.any(valid_mask):
            return

        valid_ranges = filtered[valid_mask]
        valid_angles = angles[valid_mask]

        # --- En uzak nokta ---
        farthest_idx = np.argmax(valid_ranges)
        farthest_dist = valid_ranges[farthest_idx]
        farthest_angle = valid_angles[farthest_idx]

        fp = PointStamped()
        fp.header.stamp = msg.header.stamp
        fp.header.frame_id = self.frame_id
        # Robot (laser) çerçevesinde kartezyen koordinat
        fp.point.x = float(farthest_dist * math.cos(farthest_angle))
        fp.point.y = float(farthest_dist * math.sin(farthest_angle))
        fp.point.z = 0.0
        self.farthest_pub.publish(fp)

        # --- En yakın engel ---
        nearest_dist = float(np.min(valid_ranges))
        near_msg = Float32()
        near_msg.data = nearest_dist
        self.nearest_pub.publish(near_msg)

        # Güvenlik yarıçapı içinde engel var mı?
        danger = Bool()
        danger.data = nearest_dist < self.safety_radius
        self.obstacle_detected_pub.publish(danger)

        if danger.data:
            self.get_logger().warn(
                f'ENGEL! En yakın nesne: {nearest_dist:.2f} m '
                f'(güvenlik sınırı: {self.safety_radius:.2f} m)',
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
