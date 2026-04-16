"""
mission_hardware.py  –  GERÇEK ROBOT GÖREVİ
============================================
3-tekerlekli omni robot – RoboClaw donanım sürücüsü üzerinden çalışan görev düğümü.

GÖREV AKIŞI:
  1. Sistem başladıktan 10 saniye sonra LiDAR'dan veri al
  2. En yakın tespit edilen noktayı bul (LiDAR min range)
  3. O noktaya 1 m uzaklıkta dur (quintic polinom yörüngesi)
  4. Yolda dinamik engel varsa → quintic polinom kaçınma manevrası yap
  5. Hedefe varınca 3 saniye bekle → başlangıç noktasına geri dön (aynı algoritma)

FARKLAR (sim vs hw):
  • use_sim_time = False  (gerçek sistem saati)
  • Düğüm adı: omni_mission_hw
  • Bu düğüm yalnızca /cmd_vel, /odom ve /scan topic'lerini kullanır.
  • Donanım katmanı (RoboClaw) ayrı bir düğüm tarafından yönetilir:
      → ros2 run omni_robot_sim roboclaw_hw  (veya launch dosyasından)
  • RPLiDAR gibi gerçek sensörler için sllidar_ros2 veya rplidar_ros
    paketi /scan topic'ini yayımlamalıdır.

TOPICS (gerçek robot):
  Subscribe:  /odom         (nav_msgs/Odometry)    – roboclaw_hw_node tarafından
              /scan         (sensor_msgs/LaserScan)  – RPLiDAR sürücüsü tarafından
  Publish:    /cmd_vel      (geometry_msgs/Twist)   – roboclaw_hw_node tarafından okunur

ÇALIŞTIRMAK İÇİN:
  Terminal 1:  ros2 run omni_robot_sim roboclaw_hw
  Terminal 2:  ros2 run sllidar_ros2 sllidar_node  (veya uygun LiDAR sürücüsü)
  Terminal 3:  ros2 run omni_mission mission_hw
  Ya da:       ros2 launch omni_mission mission_hardware.launch.py
"""

import math
import numpy as np
from enum import Enum, auto

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ══════════════════════════════════════════════════════════════════════════════
# Quintic polinom yardımcı fonksiyonları  (mission_sim.py ile aynı)
# ══════════════════════════════════════════════════════════════════════════════

def quintic_coeffs(p0: float, v0: float, a0: float,
                   pf: float, vf: float, af: float,
                   T: float) -> np.ndarray:
    """
    Genel sınır koşullu 5. dereceden polinom katsayıları.

    p(t)   = c0 + c1*t + c2*t² + c3*t³ + c4*t⁴ + c5*t⁵
    Koşullar:
        p(0)  = p0,  p(T)  = pf
        p'(0) = v0,  p'(T) = vf
        p''(0)= a0,  p''(T)= af
    """
    A = np.array([
        [1,  0,   0,    0,       0,        0       ],
        [0,  1,   0,    0,       0,        0       ],
        [0,  0,   2,    0,       0,        0       ],
        [1,  T,   T**2, T**3,    T**4,     T**5    ],
        [0,  1,   2*T,  3*T**2,  4*T**3,   5*T**4  ],
        [0,  0,   2,    6*T,     12*T**2,  20*T**3 ],
    ], dtype=float)
    b = np.array([p0, v0, a0, pf, vf, af], dtype=float)
    return np.linalg.solve(A, b)


def quintic_eval(c: np.ndarray, t: float):
    """Quintic polinom konum, hız ve ivmesini t anında hesapla."""
    pos = c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3 + c[4]*t**4 + c[5]*t**5
    vel = c[1] + 2*c[2]*t + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4
    acc = 2*c[2] + 6*c[3]*t + 12*c[4]*t**2 + 20*c[5]*t**3
    return pos, vel, acc


# ══════════════════════════════════════════════════════════════════════════════
# Durum makinesi
# ══════════════════════════════════════════════════════════════════════════════

class State(Enum):
    WAITING    = auto()   # 10 sn bekleme
    SCANNING   = auto()   # LiDAR ile hedef tespiti
    NAVIGATING = auto()   # Hedefe quintic yörünge
    AT_GOAL    = auto()   # Hedefe ulaşıldı, 3 sn bekle
    RETURNING  = auto()   # Eve dön
    DONE       = auto()   # Görev bitti


# ══════════════════════════════════════════════════════════════════════════════
# Ana görev sınıfı  –  GERÇEK DONANIM
# ══════════════════════════════════════════════════════════════════════════════

class OmniMissionHW(Node):

    # ── Görev parametreleri ────────────────────────────────────────────────
    WAIT_SEC        = 10.0   # Başlangıç bekleme süresi [s]
    STOP_DIST       = 1.2    # Hedefe dur mesafesi [m]
    GOAL_TOL        = 0.20   # Hedefe varış toleransı [m]
    APPROACH_DIST   = 1.0    # Bu mesafeden sonra yavaşla [m]
    APPROACH_VEL    = 0.08   # Yaklaşma hızı [m/s]
    AT_GOAL_WAIT    = 3.0    # Hedefe varınca bekleme [s]
    MAX_VEL         = 0.18   # Maks doğrusal hız [m/s]
    MAX_ANG         = 0.9    # Maks açısal hız [rad/s]
    KP_POS          = 1.5    # Konum PD katsayısı
    KP_HDG          = 1.2    # Yön PD katsayısı

    # ── Engel kaçınma parametreleri ────────────────────────────────────────
    DANGER_DIST      = 0.65   # Tehlike bölgesi mesafesi [m]
    DANGER_HALF_DEG  = 30.0   # Tehlike konisi yarı açısı [°]
    AVOID_OFFSET     = 0.55   # Yana kayma miktarı [m]
    AVOID_MIN_T      = 0.8    # Kaçınma quintic minimum süresi [s]

    def __init__(self):
        super().__init__('omni_mission_hw')

        # ── Robot durumu ───────────────────────────────────────────────────
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.vx_body = 0.0; self.vy_body = 0.0
        self.ax_body = 0.0; self.ay_body = 0.0

        self.home_x = 0.0; self.home_y = 0.0
        self.home_set = False

        # ── LiDAR ─────────────────────────────────────────────────────────
        self.scan: LaserScan = None

        # ── Hedef ─────────────────────────────────────────────────────────
        self.goal_x: float = None
        self.goal_y: float = None

        # ── Durum makinesi ─────────────────────────────────────────────────
        self.state = State.WAITING
        self.state_t0 = self.get_clock().now()

        # ── Yörünge (quintic) ──────────────────────────────────────────────
        self.cx: np.ndarray = None
        self.cy: np.ndarray = None
        self.traj_T = 0.0
        self.traj_t = 0.0

        # ── Kaçınma quintic'i ──────────────────────────────────────────────
        self.avoiding    = False
        self.avoid_cx:   np.ndarray = None
        self.avoid_cy:   np.ndarray = None
        self.avoid_T     = 0.0
        self.avoid_t     = 0.0
        self.saved_goal_x: float = None
        self.saved_goal_y: float = None

        # ── ROS2 arayüzleri ────────────────────────────────────────────────
        self._cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry,  '/odom', self._odom_cb,  10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb,  10)

        # Kontrol döngüsü: 20 Hz
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

    # ══════════════════════════════════════════════════════════════════════
    # Callback'ler
    # ══════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        if not self.home_set:
            self.home_x = self.x
            self.home_y = self.y
            self.home_set = True
            self.get_logger().info(f'Ev konumu kaydedildi: ({self.x:.2f}, {self.y:.2f})')

    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    # ══════════════════════════════════════════════════════════════════════
    # Ana kontrol döngüsü
    # ══════════════════════════════════════════════════════════════════════

    def _loop(self):
        if   self.state == State.WAITING:    self._st_waiting()
        elif self.state == State.SCANNING:   self._st_scanning()
        elif self.state == State.NAVIGATING: self._st_nav(self.goal_x, self.goal_y, State.AT_GOAL)
        elif self.state == State.AT_GOAL:    self._st_at_goal()
        elif self.state == State.RETURNING:  self._st_nav(self.home_x, self.home_y, State.DONE)
        elif self.state == State.DONE:
            self._stop()
            self.get_logger().info('Gorev tamamlandi!', throttle_duration_sec=5.0)

    # ══════════════════════════════════════════════════════════════════════
    # Durum: WAITING
    # ══════════════════════════════════════════════════════════════════════

    def _st_waiting(self):
        kalan = self.WAIT_SEC - self._elapsed()
        if kalan > 0:
            self.get_logger().info(
                f'LiDAR taramasina {kalan:.1f}s kaldi...',
                throttle_duration_sec=2.0
            )
        else:
            self._set_state(State.SCANNING)

    # ══════════════════════════════════════════════════════════════════════
    # Durum: SCANNING
    # ══════════════════════════════════════════════════════════════════════

    def _st_scanning(self):
        if self.scan is None:
            self.get_logger().warn(
                'LiDAR verisi yok – LiDAR surucusu calisiyor mu?',
                throttle_duration_sec=1.0
            )
            return

        best_r     = float('inf')
        best_angle = 0.0
        max_r      = -float('inf')
        max_angle  = 0.0

        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if not (math.isinf(r) or math.isnan(r)):
                if self.scan.range_min < r < self.scan.range_max:
                    if r > max_r:
                        max_r = r
                        max_angle = angle
                    if r > self.STOP_DIST + 0.2 and r < best_r:
                        best_r = r
                        best_angle = angle
            angle += self.scan.angle_increment

        if max_r < 0:
            self.get_logger().warn('Hic engel algilanamadi, yeniden taranıyor...', throttle_duration_sec=1.0)
            return

        if math.isinf(best_r):
            self.get_logger().warn(
                f'Tum engeller {self.STOP_DIST}m icinde! En acik yon hedefleniyor...',
                throttle_duration_sec=1.0
            )
            best_r     = max_r
            best_angle = max_angle

        world_angle = self.yaw + best_angle
        obs_x = self.x + best_r * math.cos(world_angle)
        obs_y = self.y + best_r * math.sin(world_angle)
        dist_to_obs = math.hypot(obs_x - self.x, obs_y - self.y)

        dx = (obs_x - self.x) / dist_to_obs
        dy = (obs_y - self.y) / dist_to_obs
        reach = max(dist_to_obs - self.STOP_DIST, 0.15)
        self.goal_x = self.x + reach * dx
        self.goal_y = self.y + reach * dy

        self.get_logger().info(
            f'\n-- LiDAR TARAMASI TAMAMLANDI --\n'
            f'   Secilen engel  : {best_r:.2f}m, aci={math.degrees(best_angle):.1f} derece\n'
            f'   Engel konum    : ({obs_x:.2f}, {obs_y:.2f})\n'
            f'   Hedef (dur nok): ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

        self._plan_traj(self.goal_x, self.goal_y, 0., 0., 0., 0.)
        self._set_state(State.NAVIGATING)

    # ══════════════════════════════════════════════════════════════════════
    # Durum: NAVIGATING / RETURNING
    # ══════════════════════════════════════════════════════════════════════

    def _st_nav(self, gx: float, gy: float, next_state: State):
        dist = math.hypot(gx - self.x, gy - self.y)

        if dist < self.GOAL_TOL:
            self._stop()
            self._set_state(next_state)
            return

        if self.avoiding:
            self._exec_avoidance(gx, gy)
            return

        obs = self._check_obstacle()
        if obs is not None:
            self._start_avoidance(obs, gx, gy)
            return

        if self.cx is None:
            self._plan_traj(gx, gy, 0., 0., 0., 0.)

        t = min(self.traj_t, self.traj_T)
        px, vx_w, _ = quintic_eval(self.cx, t)
        py, vy_w, _ = quintic_eval(self.cy, t)

        vx_cmd = vx_w + self.KP_POS * (px - self.x)
        vy_cmd = vy_w + self.KP_POS * (py - self.y)

        vx_b, vy_b = self._w2b(vx_cmd, vy_cmd)

        # ── Yaklaşma yavaşlaması ──────────────────────────────────────────
        if dist < self.APPROACH_DIST:
            ratio = max(dist / self.APPROACH_DIST, 0.0)
            v_limit = self.APPROACH_VEL + ratio * (self.MAX_VEL - self.APPROACH_VEL)
            spd = math.hypot(vx_b, vy_b)
            if spd > v_limit and spd > 1e-6:
                vx_b *= v_limit / spd
                vy_b *= v_limit / spd

        if dist > 0.3:
            desired_yaw = math.atan2(gy - self.y, gx - self.x)
            w = self.KP_HDG * self._adiff(desired_yaw, self.yaw)
        else:
            w = 0.0

        self._pub(vx_b, vy_b, w)
        self.traj_t += self._dt

        if self.traj_t > self.traj_T + 0.5:
            self.get_logger().info('Yorunge bitti, yeniden planlanıyor...')
            self._plan_traj(gx, gy, self.vx_body, self.vy_body, 0., 0.)

    # ══════════════════════════════════════════════════════════════════════
    # Engel algılama ve kaçınma
    # ══════════════════════════════════════════════════════════════════════

    def _check_obstacle(self):
        """±35° önde DANGER_DIST m içinde engel varsa (mesafe, açı) döndür."""
        if self.scan is None:
            return None
        half_rad = math.radians(self.DANGER_HALF_DEG)
        min_r = float('inf')
        min_a = 0.0
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if not (math.isinf(r) or math.isnan(r)):
                if self.scan.range_min < r < self.DANGER_DIST:
                    a = (angle + math.pi) % (2 * math.pi) - math.pi
                    if abs(a) < half_rad and r < min_r:
                        min_r = r
                        min_a = a
            angle += self.scan.angle_increment
        return (min_r, min_a) if min_r < self.DANGER_DIST else None

    def _start_avoidance(self, obs, gx: float, gy: float):
        obs_r, obs_a = obs

        side = -1.0 if obs_a >= 0 else 1.0

        perp_bx, perp_by = 0.0, side * 1.0
        perp_wx = math.cos(self.yaw) * perp_bx - math.sin(self.yaw) * perp_by
        perp_wy = math.sin(self.yaw) * perp_bx + math.cos(self.yaw) * perp_by

        avoid_x = self.x + self.AVOID_OFFSET * perp_wx
        avoid_y = self.y + self.AVOID_OFFSET * perp_wy

        T = max(self.AVOID_OFFSET / self.MAX_VEL, self.AVOID_MIN_T)

        vx0_w = math.cos(self.yaw) * self.vx_body - math.sin(self.yaw) * self.vy_body
        vy0_w = math.sin(self.yaw) * self.vx_body + math.cos(self.yaw) * self.vy_body

        self.avoid_cx = quintic_coeffs(self.x,  vx0_w, 0., avoid_x, 0., 0., T)
        self.avoid_cy = quintic_coeffs(self.y,  vy0_w, 0., avoid_y, 0., 0., T)
        self.avoid_T  = T
        self.avoid_t  = 0.0
        self.saved_goal_x = gx
        self.saved_goal_y = gy
        self.avoiding = True

        self.get_logger().warn(
            f'ENGEL! {obs_r:.2f}m, aci={math.degrees(obs_a):.0f} derece -> '
            f'quintic kacınma: {"saga" if side < 0 else "sola"} {self.AVOID_OFFSET}m'
        )

    def _exec_avoidance(self, gx: float, gy: float):
        t = min(self.avoid_t, self.avoid_T)
        px, vx_w, _ = quintic_eval(self.avoid_cx, t)
        py, vy_w, _ = quintic_eval(self.avoid_cy, t)

        vx_cmd = vx_w + self.KP_POS * (px - self.x)
        vy_cmd = vy_w + self.KP_POS * (py - self.y)
        vx_b, vy_b = self._w2b(vx_cmd, vy_cmd)

        desired_yaw = math.atan2(vy_w, vx_w) if math.hypot(vx_w, vy_w) > 0.02 else self.yaw
        w = self.KP_HDG * self._adiff(desired_yaw, self.yaw)

        self._pub(vx_b, vy_b, w)
        self.avoid_t += self._dt

        if self.avoid_t >= self.avoid_T:
            self.get_logger().info(
                f'Kacınma tamamlandi -> hedefe ({gx:.2f},{gy:.2f}) yeniden planlanıyor'
            )
            vx0_w = math.cos(self.yaw) * self.vx_body - math.sin(self.yaw) * self.vy_body
            vy0_w = math.sin(self.yaw) * self.vx_body + math.cos(self.yaw) * self.vy_body
            self._plan_traj(gx, gy, vx0_w, vy0_w, 0., 0.)
            self.avoiding = False

    # ══════════════════════════════════════════════════════════════════════
    # Durum: AT_GOAL
    # ══════════════════════════════════════════════════════════════════════

    def _st_at_goal(self):
        self._stop()
        kalan = self.AT_GOAL_WAIT - self._elapsed()
        if kalan > 0:
            self.get_logger().info(
                f'Hedefe ulasildi! {kalan:.1f}s sonra eve donus...',
                throttle_duration_sec=1.0
            )
        else:
            self.get_logger().info('Eve donus basliyor!')
            self._plan_traj(self.home_x, self.home_y, 0., 0., 0., 0.)
            self._set_state(State.RETURNING)

    # ══════════════════════════════════════════════════════════════════════
    # Yardımcı fonksiyonlar
    # ══════════════════════════════════════════════════════════════════════

    def _plan_traj(self, gx, gy, v0x, v0y, a0x, a0y):
        dist = math.hypot(gx - self.x, gy - self.y)
        v_ort = self.MAX_VEL * 0.65
        T = max(dist / v_ort, 1.5)
        self.cx = quintic_coeffs(self.x, v0x, a0x, gx, 0., 0., T)
        self.cy = quintic_coeffs(self.y, v0y, a0y, gy, 0., 0., T)
        self.traj_T = T
        self.traj_t = 0.0
        self.avoiding = False

    def _w2b(self, wx: float, wy: float):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * wx + s * wy, -s * wx + c * wy

    def _adiff(self, a: float, b: float) -> float:
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def _pub(self, vx_b: float, vy_b: float, w: float):
        speed = math.hypot(vx_b, vy_b)
        if speed > self.MAX_VEL:
            vx_b *= self.MAX_VEL / speed
            vy_b *= self.MAX_VEL / speed
        w = max(-self.MAX_ANG, min(self.MAX_ANG, w))

        self.vx_body = vx_b
        self.vy_body = vy_b

        msg = Twist()
        msg.linear.x  = vx_b
        msg.linear.y  = vy_b
        msg.angular.z = w
        self._cmd.publish(msg)

    def _stop(self):
        self._cmd.publish(Twist())
        self.vx_body = 0.0
        self.vy_body = 0.0

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.state_t0).nanoseconds / 1e9

    def _set_state(self, s: State):
        self.state = s
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
