"""
Görev Kontrol Düğümü (Ana Durum Makinesi)

Görev:
  1. Başlat → LiDAR taraması yap
  2. En uzak noktayı bul (başlangıç anında)
  3. O noktaya git (50 cm öncesinde dur)
  4. Başlangıç noktasına geri dön (0, 0)
  5. Eve ulaşınca motorları durdur

Durum Makinesi:
  IDLE → SCANNING → NAVIGATING_TO_TARGET → AT_TARGET → RETURNING_HOME → DONE

Servisler:
  /start_mission (Trigger) → Görevi başlat

Yayınlar:
  /mission_state (String) → Mevcut durum
  /goal_pose     (PoseStamped) → Navigasyon düğümüne hedef
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


# Durum sabitleri
class State:
    IDLE                = 'IDLE'
    SCANNING            = 'SCANNING'
    NAVIGATING_TO_TARGET = 'NAVIGATING_TO_TARGET'
    AT_TARGET           = 'AT_TARGET'
    RETURNING_HOME      = 'RETURNING_HOME'
    DONE                = 'DONE'
    ERROR               = 'ERROR'


class MissionNode(Node):
    """Ana görev kontrol ROS2 düğümü."""

    def __init__(self):
        super().__init__('mission_node')

        # --- Parametreler ---
        self.declare_parameter('mission_stop_distance', 0.50)
        self.declare_parameter('home_tolerance', 0.15)
        self.declare_parameter('scan_settle_time', 2.0)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('control_frequency', 20.0)

        self.stop_dist = self.get_parameter('mission_stop_distance').value
        self.home_tol = self.get_parameter('home_tolerance').value
        self.scan_settle = self.get_parameter('scan_settle_time').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        freq = self.get_parameter('control_frequency').value

        # Durum makinesi
        self.state = State.IDLE
        self._scan_start_time = None
        self._farthest_point = None   # (x, y) odom çerçevesinde
        self._target_pose = None      # (x, y) → hedefe 50 cm öncesi

        # Robot başlangıç pozu (odom çerçevesi, 0,0 alınır)
        self.home_x = 0.0
        self.home_y = 0.0

        # Mevcut robot pozu
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Hedefe ulaşıldı mı?
        self.goal_reached_flag = False

        # --- Yayıncılar ---
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        # Dur komutu /cmd_vel_nav üzerinden gönderilir (navigation_node'u bypass eder)
        self.stop_cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # --- Aboneler ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        # En uzak nokta (laser çerçevesinde)
        self.farthest_sub = self.create_subscription(
            PointStamped, '/farthest_point', self._farthest_callback, 10)

        # Hedefe ulaşıldı bildirimi
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self._goal_reached_callback, 10)

        # --- Servisler ---
        self.start_srv = self.create_service(
            Trigger, '/start_mission', self._start_mission_srv)

        self.stop_srv = self.create_service(
            Trigger, '/stop_mission', self._stop_mission_srv)

        # Durum makinesi zamanlayıcısı
        self.mission_timer = self.create_timer(
            1.0 / freq, self._state_machine_step)

        self.get_logger().info('='*50)
        self.get_logger().info('Görev düğümü başlatıldı.')
        self.get_logger().info('Görevi başlatmak için: ros2 service call /start_mission std_srvs/srv/Trigger')
        self.get_logger().info('='*50)

    # ------------------------------------------------------------------
    # Abonelik geri çağırımları
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry):
        """Robot pozunu günceller."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _farthest_callback(self, msg: PointStamped):
        """
        Laser çerçevesindeki en uzak noktayı odom çerçevesine dönüştürür.
        Tarama aşamasında kaydedilir.
        """
        if self.state != State.SCANNING:
            return

        # Laser çerçevesindeki koordinat
        lx = msg.point.x
        ly = msg.point.y

        # Odom çerçevesine dönüştür (robot başlangıçta 0,0,0 kabul edilir)
        # Genel dönüşüm: odom = robot_pose * laser_offset * laser_point
        # Basitleştirme: laser çerçevesi ≈ base_link → odom dönüşümü kullanılır
        cos_y = math.cos(self.robot_yaw)
        sin_y = math.sin(self.robot_yaw)
        odom_x = self.robot_x + lx * cos_y - ly * sin_y
        odom_y = self.robot_y + lx * sin_y + ly * cos_y

        self._farthest_point = (odom_x, odom_y)

    def _goal_reached_callback(self, msg: Bool):
        """Navigasyon düğümü hedefe ulaşıldığını bildirdi."""
        if msg.data:
            self.goal_reached_flag = True

    # ------------------------------------------------------------------
    # Servis geri çağırımları
    # ------------------------------------------------------------------

    def _start_mission_srv(self, request, response):
        """Görevi başlat."""
        if self.state not in [State.IDLE, State.DONE, State.ERROR]:
            response.success = False
            response.message = f'Görev zaten çalışıyor: {self.state}'
            return response

        self._transition_to(State.SCANNING)
        response.success = True
        response.message = 'Görev başlatıldı.'
        self.get_logger().info('Görev başlatıldı!')
        return response

    def _stop_mission_srv(self, request, response):
        """Görevi acil durdur."""
        self._transition_to(State.DONE)
        self._publish_stop()
        response.success = True
        response.message = 'Görev durduruldu.'
        self.get_logger().warn('Görev acil durduruldu!')
        return response

    # ------------------------------------------------------------------
    # Durum makinesi
    # ------------------------------------------------------------------

    def _state_machine_step(self):
        """Her kontrol döngüsünde çağrılır."""
        if self.state == State.IDLE:
            pass  # Servis bekleniyor

        elif self.state == State.SCANNING:
            self._handle_scanning()

        elif self.state == State.NAVIGATING_TO_TARGET:
            self._handle_navigating_to_target()

        elif self.state == State.AT_TARGET:
            self._handle_at_target()

        elif self.state == State.RETURNING_HOME:
            self._handle_returning_home()

        elif self.state == State.DONE:
            pass  # Tamamlandı

        # Durum mesajı yayınla
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

    def _handle_scanning(self):
        """LiDAR'ın oturmasını bekler, sonra hedefi hesaplar."""
        if self._scan_start_time is None:
            self._scan_start_time = self.get_clock().now()
            self.get_logger().info(
                f'LiDAR taraması bekleniyor ({self.scan_settle:.1f} sn)...'
            )
            return

        elapsed = (self.get_clock().now() - self._scan_start_time).nanoseconds / 1e9
        if elapsed < self.scan_settle:
            return

        # Tarama tamamlandı — en uzak nokta alındı mı?
        if self._farthest_point is None:
            self.get_logger().error('En uzak nokta bulunamadı! LiDAR çalışıyor mu?')
            self._transition_to(State.ERROR)
            return

        # Hedef: en uzak noktanın 50 cm öncesi (robot yönünde)
        fx, fy = self._farthest_point
        dist_to_far = math.sqrt(fx**2 + fy**2)  # Başlangıçta robot (0,0)'da

        if dist_to_far <= self.stop_dist:
            self.get_logger().warn(
                f'En uzak nokta zaten çok yakın: {dist_to_far:.2f} m'
            )
            self._transition_to(State.DONE)
            return

        # 50 cm öncesinde dur
        scale = (dist_to_far - self.stop_dist) / dist_to_far
        self._target_pose = (fx * scale, fy * scale)

        self.get_logger().info(
            f'En uzak nokta: ({fx:.2f}, {fy:.2f}) — mesafe: {dist_to_far:.2f} m'
        )
        self.get_logger().info(
            f'Hedef (50 cm öncesi): ({self._target_pose[0]:.2f}, {self._target_pose[1]:.2f})'
        )

        # Navigasyon düğümüne hedef gönder
        self._send_goal(*self._target_pose)
        self.goal_reached_flag = False
        self._transition_to(State.NAVIGATING_TO_TARGET)

    def _handle_navigating_to_target(self):
        """Hedefe ulaşılana kadar bekler."""
        if self.goal_reached_flag:
            self.get_logger().info('Hedefe ulaşıldı!')
            self._transition_to(State.AT_TARGET)

    def _handle_at_target(self):
        """
        Hedefte duruldu. Hemen eve dönüşe geçilir.
        İstersen burada bekleme süresi eklenebilir.
        """
        self.get_logger().info('Başlangıç noktasına dönülüyor...')
        self._send_goal(self.home_x, self.home_y)
        self.goal_reached_flag = False
        self._transition_to(State.RETURNING_HOME)

    def _handle_returning_home(self):
        """Eve dönüş tamamlandı mı kontrol eder."""
        if self.goal_reached_flag:
            # Çift kontrol: odometri toleransı
            home_dist = math.sqrt(
                (self.robot_x - self.home_x)**2 +
                (self.robot_y - self.home_y)**2
            )
            if home_dist <= self.home_tol:
                self.get_logger().info(
                    f'Eve ulaşıldı! Mesafe: {home_dist:.2f} m. Motorlar durduruluyor.'
                )
                self._publish_stop()
                self._transition_to(State.DONE)
            else:
                # Henüz ulaşılmadı, tekrar hedef gönder
                self.goal_reached_flag = False
                self._send_goal(self.home_x, self.home_y)

    # ------------------------------------------------------------------
    # Yardımcı metodlar
    # ------------------------------------------------------------------

    def _transition_to(self, new_state: str):
        """Durumu değiştirir ve loglar."""
        self.get_logger().info(f'Durum: {self.state} → {new_state}')
        self.state = new_state

    def _send_goal(self, goal_x: float, goal_y: float):
        """Navigasyon düğümüne hedef yayınlar."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Hedef gönderildi: ({goal_x:.2f}, {goal_y:.2f})')

    def _publish_stop(self):
        """Tüm hız komutlarını sıfırlar (motoru durdurur)."""
        stop = Twist()
        self.stop_cmd_pub.publish(stop)
        self.get_logger().info('DUR komutu gönderildi.')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
