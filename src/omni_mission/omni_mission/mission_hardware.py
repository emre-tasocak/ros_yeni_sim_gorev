"""
mission_hardware.py  –  GERÇEK ROBOT GÖREVİ
============================================
GÖREV AKIŞI:
  1. WAITING   – 10 saniye bekle
  2. SCANNING  – Robot dur, 2 saniye sadece LiDAR tara
                 Ön ±90° içinde en yakın cismi seç
  3. NAVIGATING– Hedefe düz git (P kontrolcü, dönme yok)
  4. AT_GOAL   – 3 saniye bekle
  5. RETURNING – Başlangıç noktasına düz geri dön
  6. DONE      – Dur

ÇALIŞTIRMAK İÇİN:
  ros2 launch omni_mission mission_hardware.launch.py \
    roboclaw_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ══════════════════════════════════════════════════════════════════════════════
# Durum makinesi
# ══════════════════════════════════════════════════════════════════════════════

class State(Enum):
    WAITING    = auto()
    SCANNING   = auto()
    NAVIGATING = auto()
    AT_GOAL    = auto()
    RETURNING  = auto()
    DONE       = auto()


# ══════════════════════════════════════════════════════════════════════════════
# Görev düğümü
# ══════════════════════════════════════════════════════════════════════════════

class OmniMissionHW(Node):

    # ── Parametreler ──────────────────────────────────────────────────────────
    WAIT_SEC      = 10.0   # Başlangıç bekleme [s]
    SCAN_WAIT     = 2.0    # LiDAR tarama süresi (robot durur) [s]
    SCAN_HALF_DEG = 90.0   # Sadece ön ±90° taranır [°]
    STOP_DIST     = 0.50   # Cismin önünde dur mesafesi [m]
    GOAL_TOL      = 0.12   # Hedefe varış toleransı [m]
    AT_GOAL_WAIT  = 3.0    # Hedefe varınca bekleme [s]

    MAX_VEL       = 0.18   # Maksimum öteleme hızı [m/s]  (hw daha yavaş)
    APPROACH_DIST = 0.80   # Bu mesafeden itibaren yavaşla [m]
    APPROACH_VEL  = 0.05   # Yaklaşma minimum hızı [m/s]
    KP            = 1.0    # Konum P katsayısı

    def __init__(self):
        super().__init__('omni_mission_hw')

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.home_x   = 0.0
        self.home_y   = 0.0
        self.home_set = False
        self.goal_x   = None
        self.goal_y   = None

        self.scan: LaserScan = None

        self.state    = State.WAITING
        self.state_t0 = self.get_clock().now()

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self._dt = 0.05
        self.create_timer(self._dt, self._loop)

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════╗\n'
            '║   OMNI ROBOT – GERÇEK ROBOT GÖREVİ      ║\n'
            f'║   {self.WAIT_SEC:.0f} saniye sonra LiDAR taraması      ║\n'
            '║   roboclaw_hw_node çalışıyor olmalı!    ║\n'
            '╚══════════════════════════════════════════╝'
        )

    # ── Callback'ler ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        if not self.home_set:
            self.home_x   = self.x
            self.home_y   = self.y
            self.home_set = True
            self.get_logger().info(
                f'Ev konumu kaydedildi: ({self.home_x:.2f}, {self.home_y:.2f})'
            )

    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    # ── Ana kontrol döngüsü ───────────────────────────────────────────────────

    def _loop(self):
        if   self.state == State.WAITING:    self._st_waiting()
        elif self.state == State.SCANNING:   self._st_scanning()
        elif self.state == State.NAVIGATING: self._st_nav(self.goal_x, self.goal_y,
                                                           State.AT_GOAL)
        elif self.state == State.AT_GOAL:    self._st_at_goal()
        elif self.state == State.RETURNING:  self._st_nav(self.home_x, self.home_y,
                                                           State.DONE)
        elif self.state == State.DONE:
            self._stop()
            self.get_logger().info('Gorev tamamlandi!', throttle_duration_sec=5.0)

    # ══════════════════════════════════════════════════════════════════════════
    # WAITING
    # ══════════════════════════════════════════════════════════════════════════

    def _st_waiting(self):
        kalan = self.WAIT_SEC - self._elapsed()
        if kalan > 0:
            self.get_logger().info(
                f'Baslamaya {kalan:.1f}s kaldi...',
                throttle_duration_sec=2.0
            )
            return
        self._stop()
        self._set_state(State.SCANNING)

    # ══════════════════════════════════════════════════════════════════════════
    # SCANNING – Robot durur, sadece LiDAR çalışır
    # ══════════════════════════════════════════════════════════════════════════

    def _st_scanning(self):
        self._stop()

        if self.scan is None:
            self.get_logger().warn(
                'LiDAR bekleniyor – surucusu calisiyor mu?',
                throttle_duration_sec=1.0
            )
            return

        kalan = self.SCAN_WAIT - self._elapsed()
        if kalan > 0:
            self.get_logger().info(
                f'LiDAR taranıyor... {kalan:.1f}s',
                throttle_duration_sec=0.5
            )
            return

        # Ön ±SCAN_HALF_DEG içinde en yakın geçerli noktayı bul
        half_rad   = math.radians(self.SCAN_HALF_DEG)
        best_r     = float('inf')
        best_angle = 0.0

        angle = self.scan.angle_min
        for r in self.scan.ranges:
            a = (angle + math.pi) % (2 * math.pi) - math.pi
            if (abs(a) <= half_rad
                    and not math.isinf(r)
                    and not math.isnan(r)
                    and self.scan.range_min < r < self.scan.range_max):
                if r < best_r:
                    best_r     = r
                    best_angle = angle
            angle += self.scan.angle_increment

        if math.isinf(best_r):
            self.get_logger().warn(
                'On yarimkürede cisim bulunamadi, yeniden taranıyor...',
                throttle_duration_sec=1.0
            )
            self.state_t0 = self.get_clock().now()
            return

        world_angle = self.yaw + best_angle
        obs_x = self.x + best_r * math.cos(world_angle)
        obs_y = self.y + best_r * math.sin(world_angle)

        dist  = math.hypot(obs_x - self.x, obs_y - self.y)
        reach = max(dist - self.STOP_DIST, 0.15)
        dx = (obs_x - self.x) / dist
        dy = (obs_y - self.y) / dist

        self.goal_x = self.x + reach * dx
        self.goal_y = self.y + reach * dy

        self.get_logger().info(
            f'\n-- LiDAR TARAMASI TAMAMLANDI --\n'
            f'   En yakin cisim : {best_r:.2f}m, aci = {math.degrees(best_angle):.1f} derece\n'
            f'   Cisim (dunya)  : ({obs_x:.2f}, {obs_y:.2f})\n'
            f'   Hedef          : ({self.goal_x:.2f}, {self.goal_y:.2f})\n'
            f'   Kat edilecek   : {reach:.2f}m'
        )
        self._set_state(State.NAVIGATING)

    # ══════════════════════════════════════════════════════════════════════════
    # NAVIGATING / RETURNING – P kontrolcü, dönme yok
    # ══════════════════════════════════════════════════════════════════════════

    def _st_nav(self, gx: float, gy: float, next_state: State):
        ex   = gx - self.x
        ey   = gy - self.y
        dist = math.hypot(ex, ey)

        if dist < self.GOAL_TOL:
            self._stop()
            self.get_logger().info(
                f'Hedefe ulasildi: ({gx:.2f}, {gy:.2f}) | '
                f'hata = {dist*100:.1f}cm'
            )
            self._set_state(next_state)
            return

        vx_w = self.KP * ex
        vy_w = self.KP * ey

        spd   = math.hypot(vx_w, vy_w)
        v_max = self.MAX_VEL

        if dist < self.APPROACH_DIST:
            ratio = dist / self.APPROACH_DIST
            v_max = self.APPROACH_VEL + ratio * (self.MAX_VEL - self.APPROACH_VEL)

        if spd > v_max:
            scale = v_max / spd
            vx_w *= scale
            vy_w *= scale

        c    = math.cos(self.yaw)
        s    = math.sin(self.yaw)
        vx_b =  c * vx_w + s * vy_w
        vy_b = -s * vx_w + c * vy_w

        self._publish(vx_b, vy_b, w=0.0)

        self.get_logger().info(
            f'-> hedef ({gx:.2f},{gy:.2f}) | '
            f'dist={dist:.2f}m | '
            f'v=({vx_b:.2f},{vy_b:.2f})',
            throttle_duration_sec=0.5
        )

    # ══════════════════════════════════════════════════════════════════════════
    # AT_GOAL
    # ══════════════════════════════════════════════════════════════════════════

    def _st_at_goal(self):
        self._stop()
        kalan = self.AT_GOAL_WAIT - self._elapsed()
        if kalan > 0:
            self.get_logger().info(
                f'Hedede bekleniyor. Eve donuse {kalan:.1f}s kaldi...',
                throttle_duration_sec=1.0
            )
        else:
            self.get_logger().info(
                f'Eve donus basliyor -> ({self.home_x:.2f}, {self.home_y:.2f})'
            )
            self._set_state(State.RETURNING)

    # ══════════════════════════════════════════════════════════════════════════
    # Yardımcılar
    # ══════════════════════════════════════════════════════════════════════════

    def _publish(self, vx_b: float, vy_b: float, w: float):
        msg = Twist()
        msg.linear.x  = vx_b
        msg.linear.y  = vy_b
        msg.angular.z = w
        self._pub_cmd.publish(msg)

    def _stop(self):
        try:
            self._pub_cmd.publish(Twist())
        except Exception:
            pass

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.state_t0).nanoseconds / 1e9

    def _set_state(self, s: State):
        self.state    = s
        self.state_t0 = self.get_clock().now()
        self.get_logger().info(f'=== Durum: {s.name} ===')


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = OmniMissionHW()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()
