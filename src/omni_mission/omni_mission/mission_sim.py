"""
mission_sim.py  –  GAZEBO SİMÜLASYON GÖREVİ
=============================================
3-tekerlekli omni robot – Gazebo Harmonic simülasyonu için görev düğümü.

GÖREV AKIŞI:
  1. Sistem başladıktan 10 saniye sonra LiDAR'dan veri al
  2. En yakın tespit edilen noktayı bul (LiDAR min range)
  3. O noktaya 1 m uzaklıkta dur (quintic polinom yörüngesi)
  4. Yolda dinamik engel varsa → quintic polinom kaçınma manevrası yap
  5. Hedefe varınca 3 saniye bekle → başlangıç noktasına geri dön (aynı algoritma)

KULLANILAN ALGORİTMALAR:
  • 5. dereceden (quintic) polinom yörüngesi → konum ve hız planlaması
  • Genel sınır koşullu quintic: p(0)=p0, p'(0)=v0, p''(0)=a0,
                                  p(T)=pf, p'(T)=vf, p''(T)=af
  • Dinamik Engel Kaçınma: LiDAR'dan ±35° önde cisim tespit →
    mevcut hız/ivmeden başlayan quintic ile yana kayma manevrası planlanır →
    engel geçince orijinal hedefe yeniden quintic planlanır

TOPICS (Gazebo simülasyonu):
  Subscribe:  /odom         (nav_msgs/Odometry)   – Gazebo odometri köprüsü
              /scan         (sensor_msgs/LaserScan) – Gazebo LiDAR köprüsü
  Publish:    /cmd_vel      (geometry_msgs/Twist)  – sim_kinematics_node üzerinden Gazebo'ya
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

def quintic_coeffs(p0: float, v0: float, a0: float,
                   pf: float, vf: float, af: float,
                   T: float) -> np.ndarray:
    """
    Genel sınır koşullu 5. dereceden polinom katsayıları.

    p(t)   = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
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
# Ana görev sınıfı
# ══════════════════════════════════════════════════════════════════════════════

class OmniMissionSim(Node):

    # ── Görev parametreleri ────────────────────────────────────────────────
    WAIT_SEC        = 10.0   # Başlangıç bekleme süresi [s]
    STOP_DIST       = 1.2    # Hedefe dur mesafesi [m]  (1.0 → 1.2: daha fazla marj)
    GOAL_TOL        = 0.20   # Hedefe varış toleransı [m] (0.12 → 0.20: erken dur)
    APPROACH_DIST   = 1.0    # Bu mesafeden sonra yavaşla [m]
    APPROACH_VEL    = 0.10   # Yaklaşma hızı [m/s]
    AT_GOAL_WAIT    = 3.0    # Hedefe varınca bekleme [s]
    MAX_VEL         = 0.22   # Maksimum doğrusal hız [m/s]
    MAX_ANG         = 1.0    # Maksimum açısal hız [rad/s]
    KP_POS          = 1.5    # Konum PD katsayısı
    KP_HDG          = 1.2    # Yön PD katsayısı

    # ── Engel kaçınma parametreleri ────────────────────────────────────────
    DANGER_DIST      = 0.50   # Tehlike bölgesi mesafesi [m]
    DANGER_HALF_DEG  = 25.0   # Tehlike konisi yarı açısı [°]
    AVOID_OFFSET     = 0.65   # Yana kayma miktarı [m]  (daha geniş → engeli geç)
    AVOID_MIN_T      = 1.0    # Kaçınma quintic minimum süresi [s]
    AVOID_COOLDOWN   = 3.0    # Kaçınma sonrası bekleme süresi [s] (tekrar tetiklenmeyi önler)

    def __init__(self):
        super().__init__('omni_mission_sim')

        # ── Robot durumu ───────────────────────────────────────────────────
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.vx_body = 0.0; self.vy_body = 0.0   # son yayımlanan hız
        self.ax_body = 0.0; self.ay_body = 0.0   # ivme tahmini

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
        self.cx: np.ndarray = None   # x(t) katsayıları
        self.cy: np.ndarray = None   # y(t) katsayıları
        self.traj_T = 0.0
        self.traj_t = 0.0

        # ── Kaçınma quintic'i ──────────────────────────────────────────────
        self.avoiding    = False
        self.avoid_cx:   np.ndarray = None
        self.avoid_cy:   np.ndarray = None
        self.avoid_T     = 0.0
        self.avoid_t     = 0.0
        self.saved_goal_x: float = None   # kaçınma sırasında asıl hedef
        self.saved_goal_y: float = None
        self.avoid_cooldown = 0.0         # kaçınma sonrası engel kontrol bekleme [s]

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
            '║   OMNI ROBOT – SİMÜLASYON GÖREVİ       ║\n'
            f'║   {self.WAIT_SEC:.0f} saniye sonra LiDAR taraması      ║\n'
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
            self.get_logger().info('✓ Görev tamamlandı!', throttle_duration_sec=5.0)

    # ══════════════════════════════════════════════════════════════════════
    # Durum: WAITING
    # ══════════════════════════════════════════════════════════════════════

    def _st_waiting(self):
        kalan = self.WAIT_SEC - self._elapsed()
        if kalan > 0:
            self.get_logger().info(
                f'LiDAR taramasına {kalan:.1f}s kaldı...',
                throttle_duration_sec=2.0
            )
        else:
            self._set_state(State.SCANNING)

    # ══════════════════════════════════════════════════════════════════════
    # Durum: SCANNING – En yakın LiDAR noktasını bul, hedef hesapla
    # ══════════════════════════════════════════════════════════════════════

    def _st_scanning(self):
        if self.scan is None:
            self.get_logger().warn('LiDAR verisi yok, bekleniyor...', throttle_duration_sec=1.0)
            return

        result = self._cluster_scan()
        if result is None:
            return   # _cluster_scan zaten uyarı loguyor

        avg_r, avg_angle, n_pts, n_total = result

        world_angle = self.yaw + avg_angle
        obs_x = self.x + avg_r * math.cos(world_angle)
        obs_y = self.y + avg_r * math.sin(world_angle)
        dist_to_obs = math.hypot(obs_x - self.x, obs_y - self.y)

        dx = (obs_x - self.x) / dist_to_obs
        dy = (obs_y - self.y) / dist_to_obs
        reach = max(dist_to_obs - self.STOP_DIST, 0.15)
        self.goal_x = self.x + reach * dx
        self.goal_y = self.y + reach * dy

        self.get_logger().info(
            f'\n── LiDAR TARAMASI – KÜMELEME SONUCU ──\n'
            f'   Toplam küme      : {n_total}\n'
            f'   Seçilen obje     : {n_pts} LiDAR noktası\n'
            f'   Mesafe / açı     : {avg_r:.2f}m, {math.degrees(avg_angle):.1f}°\n'
            f'   Engel koordinat  : ({obs_x:.2f}, {obs_y:.2f})\n'
            f'   Hedef (dur nok.) : ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

        self._plan_traj(self.goal_x, self.goal_y, 0., 0., 0., 0.)
        self._set_state(State.NAVIGATING)

    def _cluster_scan(self):
        """
        LiDAR taramasını Kartezyen mesafe eşiğiyle kümelere ayır.

        Duvarlar → çok sayıda ardışık nokta (büyük küme)
        Objeler  → az sayıda ardışık nokta  (küçük küme)

        En az noktalı küme seçilir → en küçük obje.

        Dönüş: (ort_mesafe, ort_açı, nokta_sayısı, toplam_küme) veya None
        """
        GAP_M    = 0.35   # İki ardışık nokta arası maks Kartezyen mesafe [m]
        MIN_PTS  = 2      # Geçerli küme için minimum nokta sayısı
        MAX_PTS  = 50     # Bu kadar veya daha fazla nokta = duvar (filtrele)

        ranges    = self.scan.ranges
        a_min     = self.scan.angle_min
        a_inc     = self.scan.angle_increment
        r_min_lim = self.scan.range_min
        r_max_lim = self.scan.range_max

        clusters: list = []
        cur:      list = []

        for i, r in enumerate(ranges):
            angle = a_min + i * a_inc
            valid = (
                not math.isinf(r) and not math.isnan(r)
                and r_min_lim < r < r_max_lim
            )

            if not valid:
                if len(cur) >= MIN_PTS:
                    clusters.append(cur)
                cur = []
                continue

            if cur:
                pr, pa = cur[-1]
                dx = r * math.cos(angle) - pr * math.cos(pa)
                dy = r * math.sin(angle) - pr * math.sin(pa)
                if math.hypot(dx, dy) > GAP_M:
                    if len(cur) >= MIN_PTS:
                        clusters.append(cur)
                    cur = []

            cur.append((r, angle))

        if len(cur) >= MIN_PTS:
            clusters.append(cur)

        # Duvarları çıkar: MAX_PTS'ten fazla noktalı kümeler duvar sayılır
        obj_clusters = [c for c in clusters if len(c) <= MAX_PTS]

        if not obj_clusters:
            self.get_logger().warn(
                f'Obje kümesi bulunamadı '
                f'({len(clusters)} küme var, hepsi duvar eşiğini ({MAX_PTS} nokta) aşıyor).',
                throttle_duration_sec=1.0
            )
            return None

        # En az noktalı küme = en küçük obje
        best = min(obj_clusters, key=len)
        avg_r     = sum(p[0] for p in best) / len(best)
        avg_angle = sum(p[1] for p in best) / len(best)

        # STOP_DIST içindeyse bu küme zaten çok yakın, atla
        if avg_r <= self.STOP_DIST:
            self.get_logger().warn(
                f'En küçük obje çok yakın ({avg_r:.2f}m ≤ {self.STOP_DIST}m), atlanıyor.',
                throttle_duration_sec=1.0
            )
            # Bir sonraki en küçük objeyi dene
            remaining = sorted(obj_clusters, key=len)
            for cand in remaining[1:]:
                cr = sum(p[0] for p in cand) / len(cand)
                ca = sum(p[1] for p in cand) / len(cand)
                if cr > self.STOP_DIST:
                    avg_r, avg_angle = cr, ca
                    best = cand
                    break
            else:
                return None

        return avg_r, avg_angle, len(best), len(clusters)

    # ══════════════════════════════════════════════════════════════════════
    # Durum: NAVIGATING / RETURNING – quintic + dinamik kaçınma
    # ══════════════════════════════════════════════════════════════════════

    def _st_nav(self, gx: float, gy: float, next_state: State):
        dist = math.hypot(gx - self.x, gy - self.y)

        # Hedefe ulaşıldı mı?
        if dist < self.GOAL_TOL:
            self._stop()
            self._set_state(next_state)
            return

        # ── Kaçınma manevrası aktifse devam et ────────────────────────────
        if self.avoiding:
            self._exec_avoidance(gx, gy)
            return

        # ── Kaçınma cooldown: yeni kaçınmayı engelle ──────────────────────
        if self.avoid_cooldown > 0.0:
            self.avoid_cooldown -= self._dt
            # Cooldown sırasında normal yörünge takibi yap (kontrol aşağıda devam eder)
        else:
            # ── Engel kontrolü ────────────────────────────────────────────
            obs = self._check_obstacle()
            if obs is not None:
                self._start_avoidance(obs, gx, gy)
                return

        # ── Normal quintic yörünge takibi ─────────────────────────────────
        if self.cx is None:
            self._plan_traj(gx, gy, 0., 0., 0., 0.)

        t = min(self.traj_t, self.traj_T)
        px, vx_w, ax_w = quintic_eval(self.cx, t)
        py, vy_w, ay_w = quintic_eval(self.cy, t)

        # Konum hatası düzeltmesi (PD)
        vx_cmd = vx_w + self.KP_POS * (px - self.x)
        vy_cmd = vy_w + self.KP_POS * (py - self.y)

        # Dünya → gövde çerçevesi
        vx_b, vy_b = self._w2b(vx_cmd, vy_cmd)

        # ── Yaklaşma yavaşlaması ───────────────────────────────────────────
        # Hedefe APPROACH_DIST m kala hızı doğrusal olarak APPROACH_VEL'e indir
        if dist < self.APPROACH_DIST:
            ratio = max(dist / self.APPROACH_DIST, 0.0)
            v_limit = self.APPROACH_VEL + ratio * (self.MAX_VEL - self.APPROACH_VEL)
            spd = math.hypot(vx_b, vy_b)
            if spd > v_limit and spd > 1e-6:
                vx_b *= v_limit / spd
                vy_b *= v_limit / spd

        # Yön kontrolü – yalnızca hedefe 0.3m'den uzakta hizala
        if dist > 0.3:
            desired_yaw = math.atan2(gy - self.y, gx - self.x)
            w = self.KP_HDG * self._adiff(desired_yaw, self.yaw)
        else:
            w = 0.0   # çok yakında dönme, sadece konuma git

        self._pub(vx_b, vy_b, w)
        self.traj_t += self._dt

        # Yörünge bitti ama hedefe ulaşılmadıysa yeniden planla
        if self.traj_t > self.traj_T + 0.5:
            self.get_logger().info('Yörünge bitti, yeniden planlanıyor...')
            self._plan_traj(gx, gy, self.vx_body, self.vy_body, 0., 0.)

    # ══════════════════════════════════════════════════════════════════════
    # Engel algılama ve quintic kaçınma
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
                    # Açıyı [-π, π] aralığına normalize et
                    a = (angle + math.pi) % (2 * math.pi) - math.pi
                    if abs(a) < half_rad and r < min_r:
                        min_r = r
                        min_a = a
            angle += self.scan.angle_increment
        return (min_r, min_a) if min_r < self.DANGER_DIST else None

    def _start_avoidance(self, obs, gx: float, gy: float):
        """
        Mevcut hız/ivmeden başlayan quintic kaçınma yörüngesi hesapla.

        Engel solda ise sağa, sağda ise sola kayılır.
        Kaçınma noktası: mevcut konumdan AVOID_OFFSET m yana.
        """
        obs_r, obs_a = obs

        # Yana kayma yönü (gövde çerçevesinde, sonra dünyaya çevrilir)
        side = -1.0 if obs_a >= 0 else 1.0   # sol engel → sağa kaç

        # Gövde çerçevesinde yana birim vektör → dünya çerçevesine çevir
        perp_bx, perp_by = 0.0, side * 1.0              # gövde: (0, ±1)
        perp_wx = math.cos(self.yaw) * perp_bx - math.sin(self.yaw) * perp_by
        perp_wy = math.sin(self.yaw) * perp_bx + math.cos(self.yaw) * perp_by

        avoid_x = self.x + self.AVOID_OFFSET * perp_wx
        avoid_y = self.y + self.AVOID_OFFSET * perp_wy

        # Kaçınma süresi: quintic başlangıç hız/ivmesi mevcut durumdan alınır
        T = max(self.AVOID_OFFSET / self.MAX_VEL, self.AVOID_MIN_T)

        # Mevcut gövde hızını dünya çerçevesine çevir
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
            f'⚠ ENGEL! {obs_r:.2f}m, açı={math.degrees(obs_a):.0f}° → '
            f'quintic kaçınma: {"sağa" if side < 0 else "sola"} {self.AVOID_OFFSET}m'
        )

    def _exec_avoidance(self, gx: float, gy: float):
        """Kaçınma quintic'ini çalıştır; bitince orijinal hedefe yeniden planla."""
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
                f'✓ Kaçınma tamamlandı → hedefe ({gx:.2f},{gy:.2f}) yeniden planlanıyor'
            )
            # Kaçınma son hızını başlangıç koşulu olarak kullan
            vx0_w = math.cos(self.yaw) * self.vx_body - math.sin(self.yaw) * self.vy_body
            vy0_w = math.sin(self.yaw) * self.vx_body + math.cos(self.yaw) * self.vy_body
            self._plan_traj(gx, gy, vx0_w, vy0_w, 0., 0.)
            self.avoiding = False
            self.avoid_cooldown = self.AVOID_COOLDOWN   # yeni kaçınmayı 3 s erteле

    # ══════════════════════════════════════════════════════════════════════
    # Durum: AT_GOAL
    # ══════════════════════════════════════════════════════════════════════

    def _st_at_goal(self):
        self._stop()
        kalan = self.AT_GOAL_WAIT - self._elapsed()
        if kalan > 0:
            self.get_logger().info(
                f'✓ Hedefe ulaşıldı! {kalan:.1f}s sonra eve dönüş...',
                throttle_duration_sec=1.0
            )
        else:
            self.get_logger().info('Eve dönüş başlıyor!')
            self._plan_traj(self.home_x, self.home_y, 0., 0., 0., 0.)
            self._set_state(State.RETURNING)

    # ══════════════════════════════════════════════════════════════════════
    # Yardımcı fonksiyonlar
    # ══════════════════════════════════════════════════════════════════════

    def _plan_traj(self, gx, gy, v0x, v0y, a0x, a0y):
        """Mevcut konumdan (gx,gy)'e quintic yörüngesi planla."""
        dist = math.hypot(gx - self.x, gy - self.y)
        v_ort = self.MAX_VEL * 0.65
        T = max(dist / v_ort, 1.5)
        self.cx = quintic_coeffs(self.x, v0x, a0x, gx, 0., 0., T)
        self.cy = quintic_coeffs(self.y, v0y, a0y, gy, 0., 0., T)
        self.traj_T = T
        self.traj_t = 0.0
        self.avoiding = False

    def _w2b(self, wx: float, wy: float):
        """Dünya çerçevesi vektörü → gövde çerçevesi."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * wx + s * wy, -s * wx + c * wy

    def _adiff(self, a: float, b: float) -> float:
        """İki açı arasındaki fark, [-π, π] aralığında."""
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def _pub(self, vx_b: float, vy_b: float, w: float):
        """Gövde çerçevesinde hız komutunu sınırla ve yayımla."""
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
        self.get_logger().info(f'═══ Durum: {s.name} ═══')


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = OmniMissionSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()
