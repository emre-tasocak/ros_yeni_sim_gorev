"""
3-Tekerli Omni Robot Kinematik Hesaplamaları

Fiziksel tekerlek konfigürasyonu (robota üstten bakıldığında, x=ileri, y=sol):
  w1: Sağ-ön  (front-right), α₁ = -60°
  w2: Sol-ön  (front-left),  α₂ = +60°
  w3: Arka    (back),        α₃ = 180°

Omni teker hız formülü: v_i = -vx·sin(αᵢ) + vy·cos(αᵢ) + L·ω

Ters kinematik matris (J_inv): [w1, w2, w3] = J_inv @ [vx, vy, omega]
  J_inv = [[ √3/2,  1/2, L ],   # w1 (α=-60°)
           [-√3/2,  1/2, L ],   # w2 (α=+60°)
           [  0,   -1,   L ]]   # w3 (α=180°)

İleri kinematik matris (J): [vx, vy, omega] = J @ [w1_dist, w2_dist, w3_dist]
  J = inv(J_inv) — numpy ile hesaplanır
"""

import numpy as np
import math


class OmniKinematics:
    """3-tekerli omni robot kinematik sınıfı."""

    def __init__(self, wheel_radius: float, wheel_base: float,
                 ticks_per_rev: int, gear_ratio: float = 1.0):
        """
        Args:
            wheel_radius: Teker yarıçapı (metre)
            wheel_base:   Merkez ile teker arasındaki mesafe (metre)
            ticks_per_rev: Bir teker turunda enkoder tick sayısı
            gear_ratio:   Dişli oranı
        """
        self.r = wheel_radius
        self.L = wheel_base
        self.ticks_per_rev = ticks_per_rev
        self.gear_ratio = gear_ratio

        # Bir tick'in karşılık geldiği mesafe (metre)
        self.dist_per_tick = (2.0 * math.pi * self.r) / self.ticks_per_rev

        # Ters kinematik matris: robot gövde hızı → teker doğrusal hızı
        # v_i = -vx·sin(αᵢ) + vy·cos(αᵢ) + L·ω
        L = self.L
        s = math.sqrt(3) / 2.0  # sin(60°) = √3/2
        self.J_inv = np.array([
            [ s,   0.5,  L],   # w1: sağ-ön,  α₁ = -60°
            [-s,   0.5,  L],   # w2: sol-ön,  α₂ = +60°
            [ 0.0, -1.0, L],   # w3: arka,    α₃ = 180°
        ])

        # İleri kinematik matris: teker deplasmanları → robot deplasmanı
        self.J = np.linalg.inv(self.J_inv)

    # ------------------------------------------------------------------
    # Hız dönüşümleri
    # ------------------------------------------------------------------

    def robot_vel_to_wheel_vel(self, vx: float, vy: float, omega: float):
        """
        Robot gövde hızlarını teker doğrusal hızlarına çevirir (m/s).

        Returns:
            (w1, w2, w3): Her teker için m/s cinsinden hız
        """
        robot_vel = np.array([vx, vy, omega])
        wheel_vels = self.J_inv @ robot_vel
        return float(wheel_vels[0]), float(wheel_vels[1]), float(wheel_vels[2])

    def wheel_vel_to_robot_vel(self, w1: float, w2: float, w3: float):
        """
        Teker hızlarını robot gövde hızlarına çevirir.

        Returns:
            (vx, vy, omega)
        """
        wheel_vels = np.array([w1, w2, w3])
        robot_vel = self.J @ wheel_vels
        return float(robot_vel[0]), float(robot_vel[1]), float(robot_vel[2])

    # ------------------------------------------------------------------
    # Enkoder dönüşümleri
    # ------------------------------------------------------------------

    def velocity_to_ticks_per_sec(self, velocity_ms: float) -> float:
        """Teker doğrusal hızını (m/s) enkoder tick/sn'ye çevirir."""
        return velocity_ms / self.dist_per_tick

    def ticks_per_sec_to_velocity(self, ticks_per_sec: float) -> float:
        """Enkoder tick/sn'yi teker doğrusal hızına (m/s) çevirir."""
        return ticks_per_sec * self.dist_per_tick

    def ticks_to_distance(self, ticks: int) -> float:
        """Enkoder tick sayısını tekerin kat ettiği mesafeye (metre) çevirir."""
        return ticks * self.dist_per_tick

    # ------------------------------------------------------------------
    # Odometri hesabı
    # ------------------------------------------------------------------

    def compute_odometry(self,
                         delta_ticks_1: int,
                         delta_ticks_2: int,
                         delta_ticks_3: int,
                         current_x: float,
                         current_y: float,
                         current_yaw: float):
        """
        Enkoder değişimlerinden yeni robot pozunu hesaplar.

        Args:
            delta_ticks_*: Her teker için enkoder değişimi (tick)
            current_*:     Şu anki robot pozu

        Returns:
            (new_x, new_y, new_yaw, dx_local, dy_local, dyaw)
            dx_local, dy_local: Robot çerçevesinde yerel hareket
        """
        # Tick → mesafe
        dp1 = self.ticks_to_distance(delta_ticks_1)
        dp2 = self.ticks_to_distance(delta_ticks_2)
        dp3 = self.ticks_to_distance(delta_ticks_3)

        # İleri kinematik: yerel deplasman
        displacement = self.J @ np.array([dp1, dp2, dp3])
        dx_local = float(displacement[0])
        dy_local = float(displacement[1])
        dyaw = float(displacement[2])

        # Yerel deplasmandan dünya çerçevesine dönüşüm (orta yaw açısı kullanılır)
        mid_yaw = current_yaw + dyaw / 2.0
        cos_y = math.cos(mid_yaw)
        sin_y = math.sin(mid_yaw)

        dx_world = dx_local * cos_y - dy_local * sin_y
        dy_world = dx_local * sin_y + dy_local * cos_y

        new_x = current_x + dx_world
        new_y = current_y + dy_world
        new_yaw = current_yaw + dyaw

        # Açıyı [-pi, pi] aralığına normalize et
        new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))

        return new_x, new_y, new_yaw, dx_local, dy_local, dyaw

    # ------------------------------------------------------------------
    # Yardımcı fonksiyonlar
    # ------------------------------------------------------------------

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Açıyı [-pi, pi] aralığına normalize eder."""
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def distance_to_goal(x1: float, y1: float, x2: float, y2: float) -> float:
        """İki nokta arasındaki Öklid mesafesi."""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
