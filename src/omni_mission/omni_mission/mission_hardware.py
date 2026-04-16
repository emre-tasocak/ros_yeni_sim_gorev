"""
mission_hardware.py  –  GERÇEK ROBOT GÖREVİ
============================================
GÖREV AKIŞI:
  1. 10 saniye bekle (sistem oturulsun)
  2. Robot dur, 2 saniye sadece LiDAR tara
  3. En yakın cismi bul (LiDAR range_min≈0.15m sayesinde kendi gövdesini görmez)
  4. Cismin önünde STOP_DIST m uzakta hedefe düz git
  5. 3 saniye bekle → başlangıç noktasına düz geri dön
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
# Quintic polinom yardımcı fonksiyonları
# ══════════════════════════════════════════════════════════════════════════════

def quintic_coeffs(p0, v0, a0, pf, vf, af, T):
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


def quintic_eval(c, t):
    pos = c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3 + c[4]*t**4 + c[5]*t**5
    vel = c[1] + 2*c[2]*t + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4
    return pos, vel


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
# Ana görev sınıfı  –  GERÇEK DONANIM
# ══════════════════════════════════════════════════════════════════════════════

class OmniMissionHW(Node):

    WAIT_SEC      = 10.0   # başlangıç bekleme [s]
    SCAN_WAIT     = 2.0    # tarama süresi – robot durur [s]
    STOP_DIST     = 1.0    # cismin önünde dur mesafesi [m]
    GOAL_TOL      = 0.20   # hedefe varış toleransı [m]
    AT_GOAL_WAIT  = 3.0    # hedefe varınca bekleme [s]
    MAX_VEL       = 0.18   # maksimum hız [m/s]
    MAX_ANG       = 0.9    # maksimum açısal hız [rad/s]
    KP_POS        = 1.5    # konum P katsayısı
    KP_HDG        = 1.2    # yön P katsayısı
    APPROACH_DIST = 1.0    # yaklaşma yavaşlama mesafesi [m]
    APPROACH_VEL  = 0.08   # yaklaşma minimum hızı [m/s]

    def __init__(self):
        super().__init__('omni_mission_hw')

        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.vx_body = 0.0; self.vy_body = 0.0

        self.home_x = 0.0; self.home_y = 0.0
        self.home_set = False

        self.scan: LaserScan = None
        self.goal_x: float = None
        self.goal_y: float = None

        self.state = State.WAITING
        self.state_t0 = self.get_clock().now()

        self.cx: np.ndarray = None
        self.cy: np.ndarray = None
        self.traj_T = 0.0
        self.traj_t = 0.0

        self._cmd = self.create_publisher(Twist, '/cmd_vel', 10)
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
            self.home_x = self.x
            self.home_y = self.y
            self.home_set = True
            self.get_logger().info(f'Ev konumu: ({self.x:.2f}, {self.y:.2f})')

    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    # ── Ana döngü ─────────────────────────────────────────────────────────────

    def _loop(self):
        if   self.state == State.WAITING:    self._st_waiting()
        elif self.state == State.SCANNING:   self._st_scanning()
        elif self.state == State.NAVIGATING: self._st_nav(self.goal_x, self.goal_y, State.AT_GOAL)
        elif self.state == State.AT_GOAL:    self._st_at_goal()
        elif self.state == State.RETURNING:  self._st_nav(self.home_x, self.home_y, State.DONE)
        elif self.state == State.DONE:
            self._stop()
            self.get_logger().info('Gorev tamamlandi!', throttle_duration_sec=5.0)

    # ── WAITING ───────────────────────────────────────────────────────────────

    def _st_waiting(self):
        kalan = self.WAIT_SEC - self._elapsed()
        if kalan > 0:
            self.get_logger().info(
                f'Baslamaya {kalan:.1f}s kaldi...',
                throttle_duration_sec=2.0
            )
        else:
            self._stop()
            self._set_state(State.SCANNING)

    # ── SCANNING – robot durur, sadece LiDAR okur ─────────────────────────────

    def _st_scanning(self):
        self._stop()

        if self.scan is None:
            self.get_logger().warn(
                'LiDAR verisi yok – LiDAR surucusu calisiyor mu?',
                throttle_duration_sec=1.0
            )
            return

        gecen = self._elapsed()
        kalan = self.SCAN_WAIT - gecen

        if kalan > 0:
            self.get_logger().info(
                f'LiDAR taranıyor... {kalan:.1f}s',
                throttle_duration_sec=0.5
            )
            return

        # En yakın geçerli noktayı bul
        best_r = float('inf')
        best_angle = 0.0

        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if (not math.isinf(r) and not math.isnan(r)
                    and self.scan.range_min < r < self.scan.range_max):
                if r < best_r:
                    best_r = r
                    best_angle = angle
            angle += self.scan.angle_increment

        if math.isinf(best_r):
            self.get_logger().warn(
                'LiDAR gecerli okuma bulamadi, yeniden taranıyor...',
                throttle_duration_sec=1.0
            )
            self.state_t0 = self.get_clock().now()
            return

        world_angle = self.yaw + best_angle
        obs_x = self.x + best_r * math.cos(world_angle)
        obs_y = self.y + best_r * math.sin(world_angle)

        dist_to_obs = math.hypot(obs_x - self.x, obs_y - self.y)
        reach = max(dist_to_obs - self.STOP_DIST, 0.10)
        dx = (obs_x - self.x) / dist_to_obs
        dy = (obs_y - self.y) / dist_to_obs

        self.goal_x = self.x + reach * dx
        self.goal_y = self.y + reach * dy

        self.get_logger().info(
            f'\n-- LiDAR TARAMASI TAMAMLANDI --\n'
            f'   En yakin cisim : {best_r:.2f}m, aci={math.degrees(best_angle):.1f} derece\n'
            f'   Cisim koordinat: ({obs_x:.2f}, {obs_y:.2f})\n'
            f'   Hedef (dur nok.): ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

        self._plan_traj(self.goal_x, self.goal_y)
        self._set_state(State.NAVIGATING)

    # ── NAVIGATING / RETURNING – düz git, kaçınma yok ────────────────────────

    def _st_nav(self, gx: float, gy: float, next_state: State):
        dist = math.hypot(gx - self.x, gy - self.y)

        if dist < self.GOAL_TOL:
            self._stop()
            self._set_state(next_state)
            return

        if self.cx is None:
            self._plan_traj(gx, gy)

        t = min(self.traj_t, self.traj_T)
        px, vx_w = quintic_eval(self.cx, t)
        py, vy_w = quintic_eval(self.cy, t)

        vx_cmd = vx_w + self.KP_POS * (px - self.x)
        vy_cmd = vy_w + self.KP_POS * (py - self.y)

        vx_b, vy_b = self._w2b(vx_cmd, vy_cmd)

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
            self._plan_traj(gx, gy)

    # ── AT_GOAL ───────────────────────────────────────────────────────────────

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
            self._plan_traj(self.home_x, self.home_y)
            self._set_state(State.RETURNING)

    # ── Yardımcılar ───────────────────────────────────────────────────────────

    def _plan_traj(self, gx, gy):
        dist = math.hypot(gx - self.x, gy - self.y)
        T = max(dist / (self.MAX_VEL * 0.65), 1.5)
        self.cx = quintic_coeffs(self.x, 0., 0., gx, 0., 0., T)
        self.cy = quintic_coeffs(self.y, 0., 0., gy, 0., 0., T)
        self.traj_T = T
        self.traj_t = 0.0

    def _w2b(self, wx, wy):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * wx + s * wy, -s * wx + c * wy

    def _adiff(self, a, b):
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def _pub(self, vx_b, vy_b, w):
        spd = math.hypot(vx_b, vy_b)
        if spd > self.MAX_VEL:
            vx_b *= self.MAX_VEL / spd
            vy_b *= self.MAX_VEL / spd
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

    def _elapsed(self):
        return (self.get_clock().now() - self.state_t0).nanoseconds / 1e9

    def _set_state(self, s: State):
        self.state = s
        self.state_t0 = self.get_clock().now()
        self.cx = None
        self.cy = None
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
