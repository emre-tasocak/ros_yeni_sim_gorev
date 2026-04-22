"""
DWA Dinamik Engel Kaçınma Düğümü

Dinamik Pencere Yaklaşımı (Dynamic Window Approach) kullanarak
navigasyon düğümünden gelen ham hız komutunu güvenli bir hız komutuna çevirir.

Mimari:
  /cmd_vel_nav  (navigasyon istediği hız)
  /scan/filtered (LiDAR verisi)
         ↓
    [DWA Algoritması]
         ↓
  /cmd_vel (engelden kaçınılmış güvenli hız)

DWA Mantığı:
1. Hedef yön ile hizalanmış hız örnekleri üret (dinamik pencere)
2. Her örnek için yörünge simüle et
3. Güvensiz yörüngeleri (engele <safety_radius) ele
4. En iyi puanlı yörüngeyi seç:
   puan = α*hedef_hizalama + β*engel_mesafesi + γ*hız_büyüklüğü
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(Node):
    """DWA tabanlı dinamik engel kaçınma düğümü."""

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # --- Parametreler ---
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('lidar_min_range', 0.30)
        self.declare_parameter('lidar_max_range', 8.0)
        self.declare_parameter('obstacle_safety_radius', 0.50)
        self.declare_parameter('dwa_predict_time', 2.0)
        self.declare_parameter('dwa_dt', 0.1)
        self.declare_parameter('dwa_velocity_samples', 11)
        self.declare_parameter('dwa_alpha', 1.5)
        self.declare_parameter('dwa_beta', 3.0)
        self.declare_parameter('dwa_gamma', 0.3)
        self.declare_parameter('dwa_min_clearance', 0.55)
        self.declare_parameter('control_frequency', 20.0)

        self.max_v = self.get_parameter('max_linear_velocity').value
        self.max_acc = self.get_parameter('max_acceleration').value
        self.lidar_min = self.get_parameter('lidar_min_range').value
        self.lidar_max = self.get_parameter('lidar_max_range').value
        self.safety_r = self.get_parameter('obstacle_safety_radius').value
        self.predict_t = self.get_parameter('dwa_predict_time').value
        self.dt = self.get_parameter('dwa_dt').value
        self.n_samples = self.get_parameter('dwa_velocity_samples').value
        self.alpha = self.get_parameter('dwa_alpha').value
        self.beta = self.get_parameter('dwa_beta').value
        self.gamma = self.get_parameter('dwa_gamma').value
        self.min_clearance = self.get_parameter('dwa_min_clearance').value

        # Mevcut hız (dinamik pencere hesabı için)
        self.current_vx = 0.0
        self.current_vy = 0.0

        # LiDAR verisi (son alınan)
        self.scan_ranges: list = []
        self.scan_angles: list = []
        self.has_scan = False

        # --- Yayıncılar / Aboneler ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cmd_vel_nav_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self._nav_cmd_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan/filtered', self._scan_callback, 10)

        self.get_logger().info('DWA engel kaçınma düğümü başlatıldı.')

    def _scan_callback(self, msg: LaserScan):
        """LiDAR verisini günceller."""
        ranges = []
        angles = []
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and self.lidar_min <= r <= self.lidar_max:
                angle = msg.angle_min + i * msg.angle_increment
                ranges.append(r)
                angles.append(angle)
        self.scan_ranges = ranges
        self.scan_angles = angles
        self.has_scan = True

    def _nav_cmd_callback(self, msg: Twist):
        """
        Navigasyon hız komutunu alır, DWA ile güvenli hız hesaplayıp yayınlar.
        """
        desired_vx = msg.linear.x
        desired_vy = msg.linear.y
        desired_omega = msg.angular.z

        # Eğer henüz LiDAR verisi yoksa doğrudan ilet
        if not self.has_scan or len(self.scan_ranges) == 0:
            self.cmd_vel_pub.publish(msg)
            return

        # En yakın engel mesafesi
        nearest = min(self.scan_ranges) if self.scan_ranges else self.lidar_max

        # Engel güvenli bölgede değilse DWA çalıştır
        if nearest > self.safety_r * 2.5:
            # Engel yeterince uzakta — istenen hızı doğrudan uygula
            safe_vx, safe_vy = desired_vx, desired_vy
        else:
            safe_vx, safe_vy = self._dwa_step(desired_vx, desired_vy)

            # DWA hiç yol bulamazsa (robot sınırda sıkıştı):
            # İstenen hız engelden UZAKLAŞIYORSA izin ver (yavaş)
            if safe_vx == 0.0 and safe_vy == 0.0 and (desired_vx != 0.0 or desired_vy != 0.0):
                safe_vx, safe_vy = self._escape_if_moving_away(
                    desired_vx, desired_vy, nearest)

        # Çıkış komutu
        out = Twist()
        out.linear.x = safe_vx
        out.linear.y = safe_vy
        out.angular.z = desired_omega
        self.cmd_vel_pub.publish(out)

        # Mevcut hızı güncelle
        self.current_vx = safe_vx
        self.current_vy = safe_vy

    def _dwa_step(self, desired_vx: float, desired_vy: float):
        """
        DWA adımı: istenen hız etrafında örnek üretir,
        her örnek için clearance hesaplar, en iyi olanı döndürür.
        """
        # Dinamik pencere (mevcut hız ve ivme sınırından hesaplanır)
        dt_ctrl = 1.0 / 20.0  # kontrol dönemi
        dv = self.max_acc * dt_ctrl

        vx_min = max(-self.max_v, self.current_vx - dv)
        vx_max = min(self.max_v, self.current_vx + dv)
        vy_min = max(-self.max_v, self.current_vy - dv)
        vy_max = min(self.max_v, self.current_vy + dv)

        # Hız örnekleri
        vx_samples = np.linspace(vx_min, vx_max, self.n_samples)
        vy_samples = np.linspace(vy_min, vy_max, self.n_samples)

        best_score = -1e9
        best_vx, best_vy = 0.0, 0.0

        for vx in vx_samples:
            for vy in vy_samples:
                speed = math.sqrt(vx**2 + vy**2)
                if speed > self.max_v:
                    continue

                # Yörünge boyunca minimum clearance
                clearance = self._compute_clearance(vx, vy)

                # Güvensiz yörünge — atla
                if clearance < self.min_clearance:
                    continue

                # Puan hesabı
                # 1) Hedef hız ile ne kadar uyumlu?
                goal_dist = math.sqrt((vx - desired_vx)**2 + (vy - desired_vy)**2)
                heading_score = 1.0 / (1.0 + goal_dist)

                # 2) Engele ne kadar uzak? (normalize)
                clearance_score = min(clearance / self.lidar_max, 1.0)

                # 3) Hız büyüklüğü (ilerleme teşviki)
                speed_score = speed / self.max_v

                score = (self.alpha * heading_score +
                         self.beta * clearance_score +
                         self.gamma * speed_score)

                if score > best_score:
                    best_score = score
                    best_vx, best_vy = vx, vy

        if best_score == -1e9:
            # Hiç güvenli yörünge bulunamadı — dur
            self.get_logger().warn('DWA: Güvenli yörünge bulunamadı, duruluyor!',
                                   throttle_duration_sec=1.0)
            return 0.0, 0.0

        return best_vx, best_vy

    def _compute_clearance(self, vx: float, vy: float) -> float:
        """
        Verilen hız komutu ile oluşacak yörünge boyunca
        en yakın engele olan mesafeyi hesaplar.

        Robot sabit vx, vy ile predict_t kadar hareket eder.
        Her dt adımında robot pozisyonu kontrol edilir.
        """
        if not self.scan_ranges:
            return self.lidar_max

        min_clearance = self.lidar_max

        # Zaman adımlarında robot pozisyonu
        steps = int(self.predict_t / self.dt)
        for step in range(1, steps + 1):
            t = step * self.dt
            # Robot çerçevesinde tahmin pozisyonu
            px = vx * t
            py = vy * t

            # Bu pozisyonun engellere olan mesafesi
            for r, angle in zip(self.scan_ranges, self.scan_angles):
                ox = r * math.cos(angle)
                oy = r * math.sin(angle)
                dist = math.sqrt((px - ox)**2 + (py - oy)**2)
                if dist < min_clearance:
                    min_clearance = dist

        return min_clearance

    def _escape_if_moving_away(self, desired_vx: float, desired_vy: float,
                                nearest: float):
        """
        DWA yol bulamadığında son çare: robot zaten sınırda sıkışmış.
        İstenen hız engelden UZAKLAŞIYORSA, düşük hızla geçişe izin ver.

        Nasıl çalışır: en yakın engelin yönünü bul, istenen hız ona karşı mı bak.
        Karşıysa (yani uzaklaşıyorsa) max_v'nin yarısıyla ilerlemeye izin ver.
        """
        if not self.scan_ranges:
            return 0.0, 0.0

        # En yakın engelin yönünü bul (laser çerçevesinde)
        min_r = nearest
        min_angle = 0.0
        for r, angle in zip(self.scan_ranges, self.scan_angles):
            if r <= min_r + 0.01:
                min_angle = angle
                break

        # Engele doğru birim vektör
        obs_dx = math.cos(min_angle)
        obs_dy = math.sin(min_angle)

        # İstenen hızın engel yönüyle iç çarpımı
        # Negatifse → istenen hız engelden uzaklaşıyor → güvenli
        dot = desired_vx * obs_dx + desired_vy * obs_dy
        if dot < 0.0:
            # Uzaklaşıyoruz, normalleştirip max_v/2 ile git
            speed = math.sqrt(desired_vx**2 + desired_vy**2)
            if speed < 1e-6:
                return 0.0, 0.0
            scale = min(self.max_v * 0.5 / speed, 1.0)
            self.get_logger().warn(
                'DWA kurtarma: engelden uzaklaşılıyor, yavaş geçiş.',
                throttle_duration_sec=2.0)
            return desired_vx * scale, desired_vy * scale

        # Engele doğru gidiyoruz — dur
        return 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
