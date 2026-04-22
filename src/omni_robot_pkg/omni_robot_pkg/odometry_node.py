"""
Odometri Düğümü

/wheel_ticks verisinden robot pozunu hesaplar ve yayınlar.
Aynı zamanda odom → base_link TF dönüşümünü yayınlar.

Simülasyonda bu düğüm kullanılmaz — Gazebo'nun kendi odometrisi kullanılır.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

from omni_robot_pkg.omni_kinematics import OmniKinematics


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Euler açılarını quaternion'a çevirir."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class OdometryNode(Node):
    """Enkoder verilerinden odometri hesaplayan ROS2 düğümü."""

    def __init__(self):
        super().__init__('odometry_node')

        # --- Parametreler ---
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.25)
        self.declare_parameter('ticks_per_revolution', 750)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')

        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_base').value
        tpr = self.get_parameter('ticks_per_revolution').value
        gr = self.get_parameter('gear_ratio').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.odom_frame = self.get_parameter('odom_frame_id').value

        self.kinematics = OmniKinematics(r, L, tpr, gr)

        # Robot pozu (odom çerçevesinde)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Hız tahmini (publish için)
        self.vx = 0.0
        self.vy = 0.0
        self.vomega = 0.0
        self._last_time = self.get_clock().now()

        # TF yayıncısı
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Odometri yayıncısı
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Enkoder abonesi
        self.ticks_sub = self.create_subscription(
            Int32MultiArray, '/wheel_ticks', self._ticks_callback, 10)

        self.get_logger().info('Odometri düğümü başlatıldı.')

    def _ticks_callback(self, msg: Int32MultiArray):
        """Enkoder tick değişimlerinden robot pozunu günceller."""
        if len(msg.data) < 3:
            return

        delta_t1 = msg.data[0]
        delta_t2 = msg.data[1]
        delta_t3 = msg.data[2]

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        if dt <= 0.0:
            return

        # Yeni pozu hesapla
        new_x, new_y, new_yaw, dx, dy, dyaw = self.kinematics.compute_odometry(
            delta_t1, delta_t2, delta_t3,
            self.x, self.y, self.yaw
        )

        # Anlık hız (m/s ve rad/s)
        self.vx = dx / dt
        self.vy = dy / dt
        self.vomega = dyaw / dt

        self.x = new_x
        self.y = new_y
        self.yaw = new_yaw

        # TF ve odometri yayınla
        self._publish_odom(now)
        self._publish_tf(now)

    def _publish_odom(self, stamp):
        """nav_msgs/Odometry mesajı yayınlar."""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, self.yaw)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vomega

        self.odom_pub.publish(odom)

    def _publish_tf(self, stamp):
        """odom → base_link TF dönüşümünü yayınlar."""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = euler_to_quaternion(0.0, 0.0, self.yaw)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
