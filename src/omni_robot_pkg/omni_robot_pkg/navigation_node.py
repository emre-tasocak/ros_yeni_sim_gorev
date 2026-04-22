"""
Navigasyon Düğümü

Robotun hedef bir noktaya gitmesini sağlar.
Basit P-kontrolcü kullanılır; engel kaçınma ayrı bir düğümde yapılır.

Servisler:
  /set_goal (SetGoal)  → Yeni hedef belirle
  /cancel_goal (Trigger) → Hedefi iptal et

Yayınlar:
  /cmd_vel_nav → Navigasyon hız komutu (obstacle_avoidance düğümü bu komutu alır)
  /goal_reached → Hedefe ulaşıldı mı (Bool)

Abonelikler:
  /odom → Mevcut robot pozu
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor


class NavigationNode(Node):
    """Hedefe navigasyon ROS2 düğümü (P-kontrolcü)."""

    def __init__(self):
        super().__init__('navigation_node')

        # --- Parametreler ---
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('heading_gain', 2.0)
        self.declare_parameter('control_frequency', 20.0)

        self.max_v = self.get_parameter('max_linear_velocity').value
        self.tol = self.get_parameter('goal_tolerance').value
        self.Kp = self.get_parameter('heading_gain').value
        freq = self.get_parameter('control_frequency').value

        # Robot mevcut pozu
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Hedef pozu
        self.goal_x = None
        self.goal_y = None
        self.goal_active = False
        self.goal_reached = False

        # --- Yayıncılar / Aboneler ---
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        # Hedef dinleme (mission_node bu topic'e yayınlar)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)

        # Hedef iptali servisi
        self.cancel_srv = self.create_service(
            Trigger, '/cancel_goal', self._cancel_goal_srv)

        # Kontrol döngüsü zamanlayıcısı
        self.control_timer = self.create_timer(1.0 / freq, self._control_loop)

        self.get_logger().info('Navigasyon düğümü başlatıldı.')

    def _odom_callback(self, msg: Odometry):
        """Robot pozunu günceller."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Quaternion → yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _goal_callback(self, msg: PoseStamped):
        """Yeni hedef alındığında ayarlar."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_active = True
        self.goal_reached = False
        self.get_logger().info(
            f'Yeni hedef: ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    def _cancel_goal_srv(self, request, response):
        """Aktif hedefi iptal eder."""
        self.goal_active = False
        self._publish_stop()
        response.success = True
        response.message = 'Hedef iptal edildi.'
        self.get_logger().info('Hedef iptal edildi.')
        return response

    def _control_loop(self):
        """P-kontrolcü ile hedefe yönelik hız komutu üretir."""
        if not self.goal_active or self.goal_x is None:
            return

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist = math.sqrt(dx**2 + dy**2)

        if dist <= self.tol:
            # Hedefe ulaşıldı
            if not self.goal_reached:
                self.goal_reached = True
                self.goal_active = False
                self._publish_stop()
                goal_msg = Bool()
                goal_msg.data = True
                self.goal_reached_pub.publish(goal_msg)
                self.get_logger().info(
                    f'Hedefe ulaşıldı: ({self.robot_x:.2f}, {self.robot_y:.2f})'
                )
            return

        # Hedef yönünde hız hesapla (P-kontrolcü)
        # Hız, mesafe ile orantılı ancak max_v ile sınırlı
        scale = min(self.Kp * dist, self.max_v) / dist
        vx = dx * scale
        vy = dy * scale

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = 0.0   # Rotasyon şimdilik sabit (görev gerektirmiyor)
        self.cmd_vel_nav_pub.publish(cmd)

        # Her 2 sn'de bir mesafeyi logla
        self.get_logger().info(
            f'Hedefe mesafe: {dist:.2f} m  |  vx={vx:.2f}  vy={vy:.2f}',
            throttle_duration_sec=2.0
        )

    def _publish_stop(self):
        """Dur komutu yayınlar."""
        stop = Twist()
        self.cmd_vel_nav_pub.publish(stop)

    def set_goal(self, goal_x: float, goal_y: float):
        """Programa dahili hedef ayarlamak için yardımcı metod."""
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_active = True
        self.goal_reached = False
        self.get_logger().info(f'Hedef ayarlandı: ({goal_x:.2f}, {goal_y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
