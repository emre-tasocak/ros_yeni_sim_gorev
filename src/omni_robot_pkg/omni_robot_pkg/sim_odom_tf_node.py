"""
Simülasyon Odometri → TF Yayıncısı

Gazebo OdometryPublisher'dan gelen /odom mesajını alır,
odom → base_footprint TF dönüşümünü yayınlar.

Neden gerekli:
  OdometryPublisher'ın kendi TF yayını 'odom → base_link' üretir.
  Ama URDF'de base_footprint → base_link sabit eklemi var.
  İki farklı düğüm 'base_link'e ata atamaz (TF ağacı döngüsü olur).
  Bu düğüm 'odom → base_footprint' yayınlar; robot_state_publisher
  'base_footprint → base_link → laser/kamera/...' zincirini tamamlar.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class SimOdomTFNode(Node):
    """Odometri mesajını odom→base_footprint TF'ye dönüştürür."""

    def __init__(self):
        super().__init__('sim_odom_tf')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.get_logger().info('Sim odom→base_footprint TF yayıncısı başlatıldı.')

    def _odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0   # 2D robot: zemin düzlemi
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimOdomTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
