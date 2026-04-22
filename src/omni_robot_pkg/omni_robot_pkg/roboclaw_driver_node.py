"""
RoboClaw Motor Sürücü Düğümü

Görevler:
  - /cmd_vel (Twist) mesajını dinler
  - Ters kinematiği kullanarak robot hızını teker hızlarına çevirir
  - RoboClaw sürücülerine hız komutları gönderir
  - Enkoderleri okur ve /wheel_ticks yayınlar (odometri için)

Motor bağlantısı (orijinal koddan):
  RoboClaw 1 (0x80):
    M1 → Teker 3 (ters yön, -v3)
    M2 → Teker 1 (ters yön, -v1)
  RoboClaw 2 (0x81):
    M2 → Teker 2 (düz yön, v2)

Enkoder okuma:
  Teker 1 ← RC1 M2
  Teker 2 ← RC1 M1  (dikkat: orijinal kodda bu şekilde)
  Teker 3 ← RC2 M1
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String
import time

from omni_robot_pkg.omni_kinematics import OmniKinematics
from omni_robot_pkg.roboclaw_3 import Roboclaw


class RoboclawDriverNode(Node):
    """RoboClaw motor sürücü ROS2 düğümü."""

    def __init__(self):
        super().__init__('roboclaw_driver')

        # --- Parametreler ---
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.25)
        self.declare_parameter('ticks_per_revolution', 750)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('roboclaw_port_1', '/dev/ttyAMA0')
        self.declare_parameter('roboclaw_port_2', '/dev/ttyAMA1')
        self.declare_parameter('roboclaw_baudrate', 38400)
        self.declare_parameter('roboclaw_address_1', 128)
        self.declare_parameter('roboclaw_address_2', 129)
        self.declare_parameter('roboclaw_pid_p', 50.0)
        self.declare_parameter('roboclaw_pid_i', 2.0)
        self.declare_parameter('roboclaw_pid_d', 0.0)
        self.declare_parameter('roboclaw_pid_qpps', 7900)
        self.declare_parameter('control_frequency', 20.0)

        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_base').value
        tpr = self.get_parameter('ticks_per_revolution').value
        gr = self.get_parameter('gear_ratio').value
        port1 = self.get_parameter('roboclaw_port_1').value
        port2 = self.get_parameter('roboclaw_port_2').value
        baud = self.get_parameter('roboclaw_baudrate').value
        self.addr1 = self.get_parameter('roboclaw_address_1').value
        self.addr2 = self.get_parameter('roboclaw_address_2').value
        pid_p = self.get_parameter('roboclaw_pid_p').value
        pid_i = self.get_parameter('roboclaw_pid_i').value
        pid_d = self.get_parameter('roboclaw_pid_d').value
        qpps = self.get_parameter('roboclaw_pid_qpps').value
        freq = self.get_parameter('control_frequency').value

        # Kinematik sınıfı
        self.kinematics = OmniKinematics(r, L, tpr, gr)

        # --- RoboClaw bağlantısı ---
        self.rc1 = Roboclaw(port1, baud)
        self.rc2 = Roboclaw(port2, baud)
        self._init_roboclaws(pid_p, pid_i, pid_d, int(qpps))

        # Enkoder sıfırlama
        self._reset_encoders()
        self._prev_ticks = [0, 0, 0]

        # --- Yayıncılar / Aboneler ---
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self.wheel_ticks_pub = self.create_publisher(
            Int32MultiArray, '/wheel_ticks', 10)

        self.status_pub = self.create_publisher(String, '/driver_status', 10)

        # Enkoder okuma zamanlayıcısı (kontrol frekansında)
        self.encoder_timer = self.create_timer(
            1.0 / freq, self._read_encoders)

        self.get_logger().info('RoboClaw sürücü düğümü başlatıldı.')

    def _init_roboclaws(self, p, i, d, qpps):
        """RoboClaw bağlantısını açar ve PID ayarlar."""
        for name, rc, port in [('RC1', self.rc1, 'port1'), ('RC2', self.rc2, 'port2')]:
            attempts = 0
            while rc.Open() == 0:
                attempts += 1
                if attempts > 5:
                    self.get_logger().error(f'{name} bağlanamadı! Port: {port}')
                    return
                time.sleep(0.5)
            self.get_logger().info(f'{name} bağlandı.')

        # PID ayarları — tüm 3 motor için
        self.rc1.SetM1VelocityPID(self.addr1, p, i, d, qpps)  # Teker 3
        self.rc1.SetM2VelocityPID(self.addr1, p, i, d, qpps)  # Teker 1
        self.rc2.SetM2VelocityPID(self.addr2, p, i, d, qpps)  # Teker 2

    def _reset_encoders(self):
        """Tüm enkoderleri sıfırlar."""
        self.rc1.SetEncM1(self.addr1, 0)
        self.rc1.SetEncM2(self.addr1, 0)
        self.rc2.SetEncM1(self.addr2, 0)
        self.rc2.SetEncM2(self.addr2, 0)

    def _cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel mesajını alır ve tekerlere hız komutu gönderir.

        Twist.linear.x  → vx (m/s)
        Twist.linear.y  → vy (m/s)
        Twist.angular.z → omega (rad/s)
        """
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        # Robot hızından teker hızlarını hesapla (m/s)
        w1, w2, w3 = self.kinematics.robot_vel_to_wheel_vel(vx, vy, omega)

        # m/s → tick/sn
        t1 = int(self.kinematics.velocity_to_ticks_per_sec(w1))
        t2 = int(self.kinematics.velocity_to_ticks_per_sec(w2))
        t3 = int(self.kinematics.velocity_to_ticks_per_sec(w3))

        # RoboClaw'a gönder (orijinal kodun motor eşlemesi korunmuştur)
        # RC1 M2 = -teker1, RC1 M1 = -teker3, RC2 M2 = teker2
        self.rc1.SpeedM2(self.addr1, -t1)
        self.rc1.SpeedM1(self.addr1, -t3)
        self.rc2.SpeedM2(self.addr2,  t2)

    def _read_encoders(self):
        """Enkoderleri okur ve /wheel_ticks yayınlar."""
        try:
            # Enkoder değerleri (kümülatif tick)
            raw1 = self.rc1.ReadEncM2(self.addr1)
            raw2 = self.rc1.ReadEncM1(self.addr1)
            raw3 = self.rc2.ReadEncM1(self.addr2)

            if raw1[0] and raw2[0] and raw3[0]:
                ticks = [int(raw1[1]), int(raw2[1]), int(raw3[1])]

                msg = Int32MultiArray()
                msg.data = ticks
                self.wheel_ticks_pub.publish(msg)

                self._reset_encoders()
        except Exception as e:
            self.get_logger().warn(f'Enkoder okuma hatası: {e}')

    def stop_all(self):
        """Tüm motorları durdurur."""
        self.rc1.SpeedM1(self.addr1, 0)
        self.rc1.SpeedM2(self.addr1, 0)
        self.rc2.SpeedM2(self.addr2, 0)

    def destroy_node(self):
        self.stop_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboclawDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
