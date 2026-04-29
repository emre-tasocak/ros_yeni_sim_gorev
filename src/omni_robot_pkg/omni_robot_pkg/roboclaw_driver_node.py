"""
RoboClaw Motor Sürücü Düğümü

Her iki RoboClaw aynı UART hattında (GPIO 14 TX / GPIO 15 RX, /dev/ttyAMA0).
Multi-drop bağlantı: adres 0x80 (RC1) ve 0x81 (RC2) ile ayrışır.
Tek Roboclaw nesnesi, iki farklı adrese komut gönderir.

Motor eşlemesi:
  RC1 (0x80) M1 → Teker 3 (ters)
  RC1 (0x80) M2 → Teker 1 (ters)
  RC2 (0x81) M2 → Teker 2 (düz)

Port izinleri için:
  sudo chmod 666 /dev/ttyAMA0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import time

from omni_robot_pkg.omni_kinematics import OmniKinematics
from omni_robot_pkg.roboclaw_3 import Roboclaw


class RoboclawDriverNode(Node):

    def __init__(self):
        super().__init__('roboclaw_driver')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.25)
        self.declare_parameter('ticks_per_revolution', 750)
        self.declare_parameter('gear_ratio', 1.0)
        self.declare_parameter('roboclaw_port_1', '/dev/ttyAMA0')
        self.declare_parameter('roboclaw_baudrate', 38400)
        self.declare_parameter('roboclaw_address_1', 128)
        self.declare_parameter('roboclaw_address_2', 129)
        self.declare_parameter('roboclaw_pid_p', 50.0)
        self.declare_parameter('roboclaw_pid_i', 2.0)
        self.declare_parameter('roboclaw_pid_d', 0.0)
        self.declare_parameter('roboclaw_pid_qpps', 7900)
        self.declare_parameter('control_frequency', 20.0)

        r     = self.get_parameter('wheel_radius').value
        L     = self.get_parameter('wheel_base').value
        tpr   = self.get_parameter('ticks_per_revolution').value
        gr    = self.get_parameter('gear_ratio').value
        port  = self.get_parameter('roboclaw_port_1').value
        baud  = self.get_parameter('roboclaw_baudrate').value
        self.addr1 = self.get_parameter('roboclaw_address_1').value
        self.addr2 = self.get_parameter('roboclaw_address_2').value
        pid_p = self.get_parameter('roboclaw_pid_p').value
        pid_i = self.get_parameter('roboclaw_pid_i').value
        pid_d = self.get_parameter('roboclaw_pid_d').value
        qpps  = self.get_parameter('roboclaw_pid_qpps').value
        freq  = self.get_parameter('control_frequency').value

        self.kinematics = OmniKinematics(r, L, tpr, gr)

        # Tek RC nesnesi — her iki adrese de komut gönderir
        self.hardware_ok = False
        self.rc = Roboclaw(port, baud)
        self._init_roboclaw(port, pid_p, pid_i, pid_d, int(qpps))

        if self.hardware_ok:
            self._reset_encoders()
        self._prev_ticks = [0, 0, 0]

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self.wheel_ticks_pub = self.create_publisher(
            Int32MultiArray, '/wheel_ticks', 10)

        self.encoder_timer = self.create_timer(1.0 / freq, self._read_encoders)

        if self.hardware_ok:
            self.get_logger().info(f'RoboClaw bağlandı ({port}). Adresler: {self.addr1}, {self.addr2}')
        else:
            self.get_logger().error(
                f'RoboClaw bağlanamadı ({port}). '
                'sudo chmod 666 /dev/ttyAMA0 ve kablo bağlantısını kontrol et.')

    def _init_roboclaw(self, port, p, i, d, qpps):
        for attempt in range(6):
            if self.rc.Open() != 0:
                break
            self.get_logger().warn(f'RoboClaw bağlantı denemesi {attempt+1}/6...')
            time.sleep(0.5)
        else:
            self.get_logger().error(f'RoboClaw açılamadı: {port}')
            return

        try:
            self.rc.SetM1VelocityPID(self.addr1, p, i, d, qpps)  # Teker 3
            self.rc.SetM2VelocityPID(self.addr1, p, i, d, qpps)  # Teker 1
            self.rc.SetM2VelocityPID(self.addr2, p, i, d, qpps)  # Teker 2
            self.hardware_ok = True
        except Exception as e:
            self.get_logger().error(f'PID ayarı başarısız: {e}')

    def _reset_encoders(self):
        try:
            self.rc.SetEncM1(self.addr1, 0)
            self.rc.SetEncM2(self.addr1, 0)
            self.rc.SetEncM1(self.addr2, 0)
            self.rc.SetEncM2(self.addr2, 0)
        except Exception as e:
            self.get_logger().warn(f'Enkoder sıfırlama hatası: {e}')

    def _cmd_vel_callback(self, msg: Twist):
        if not self.hardware_ok:
            return
        vx    = msg.linear.x
        vy    = msg.linear.y
        omega = msg.angular.z

        w1, w2, w3 = self.kinematics.robot_vel_to_wheel_vel(vx, vy, omega)
        t1 = int(self.kinematics.velocity_to_ticks_per_sec(w1))
        t2 = int(self.kinematics.velocity_to_ticks_per_sec(w2))
        t3 = int(self.kinematics.velocity_to_ticks_per_sec(w3))

        try:
            self.rc.SpeedM2(self.addr1, -t1)  # Teker 1
            self.rc.SpeedM1(self.addr1, -t3)  # Teker 3
            self.rc.SpeedM2(self.addr2,  t2)  # Teker 2
        except Exception as e:
            self.get_logger().warn(f'Motor komut hatası: {e}', throttle_duration_sec=2.0)

    def _read_encoders(self):
        if not self.hardware_ok:
            return
        try:
            raw1 = self.rc.ReadEncM2(self.addr1)  # RC1 M2 → Teker 1 (sağ-ön)
            raw2 = self.rc.ReadEncM2(self.addr2)  # RC2 M2 → Teker 2 (sol-ön)
            raw3 = self.rc.ReadEncM1(self.addr1)  # RC1 M1 → Teker 3 (arka)

            if raw1[0] and raw2[0] and raw3[0]:
                msg = Int32MultiArray()
                msg.data = [int(raw1[1]), int(raw2[1]), int(raw3[1])]
                self.wheel_ticks_pub.publish(msg)
                self._reset_encoders()
        except Exception as e:
            self.get_logger().warn(f'Enkoder okuma hatası: {e}', throttle_duration_sec=2.0)

    def stop_all(self):
        if not self.hardware_ok:
            return
        try:
            self.rc.SpeedM1(self.addr1, 0)
            self.rc.SpeedM2(self.addr1, 0)
            self.rc.SpeedM2(self.addr2, 0)
        except Exception:
            pass

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
