"""
roboclaw_hw_node.py
===================
ROS2 node that acts as a hardware bridge between ROS2 and the physical
3-wheeled omnidirectional robot via two RoboClaw 2×15A motor drivers.

Architecture
------------
  /cmd_vel (Twist)  →  inverse kinematics  →  RoboClaw serial commands
  RoboClaw encoders  →  forward kinematics  →  /odom (Odometry)

Motor / encoder wiring (same as MCE402 project):
  RoboClaw 0x80:  M1 = wheel_1,  M2 = wheel_2
  RoboClaw 0x81:  M1 = wheel_3

RoboClaw communication:
  Serial port: /dev/ttyACM0  (or /dev/ttyUSB0)
  Baud rate:   38400

Kinematic parameters:
  r = 0.05 m   (wheel radius)
  L = 0.20 m   (robot base radius)
  GEAR_RATIO = 30   (30:1 gearbox)
  CPR = 25          (encoder counts per motor revolution)
  TICKS_PER_TURN = 750   (= CPR × GEAR_RATIO)

Wheel layout (body frame, top view)  – MCE402 project report:
  Wheel 1 at α = -60°  (front-right)  → ( 0.10, -0.173)
  Wheel 2 at α = +60°  (front-left)   → ( 0.10, +0.173)
  Wheel 3 at α = 180°  (rear)         → (-0.20,  0.000)
  Robot +X = forward,  Wheel 3 Y = 0 in robot frame.

Inverse kinematics (body frame):
  ω_i = (1/r)[ -sin(α_i)·Vx  +  cos(α_i)·Vy  +  L·ω ]

Forward kinematics (MCE402 report formula – verified for this layout):
  [Vx, Vy, ω]ᵀ = (r/3) · [[√3,-√3, 0], [1, 1,-2], [1/L,1/L,1/L]] · [ω1,ω2,ω3]ᵀ
  Equivalently: v_body = r * J_fwd_raw @ wheel_ω  where J_fwd_raw = inv(J_inv_raw)

Note: roboclaw_python library must be installed/available.
      Copy roboclaw_3.py into the package or install from pypi.
"""

import math
import os
import sys
import time
from typing import Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import tf2_ros


# ─────────────────────────────────────────────────────────────────────────────
# RoboClaw import (try from installed roboclaw_python, fall back to local copy)
# ─────────────────────────────────────────────────────────────────────────────
try:
    from roboclaw_python.roboclaw_3 import Roboclaw
except ImportError:
    # Try local copy next to this file
    _here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, _here)
    try:
        from roboclaw_3 import Roboclaw
    except ImportError:
        Roboclaw = None   # node will warn and run in simulation-only mode


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

ADDR_12 = 0x80   # RoboClaw #1 (wheel 1 + wheel 2)
ADDR_3  = 0x81   # RoboClaw #2 (wheel 3)

GEAR_RATIO      = 30
CPR             = 25          # encoder counts per motor revolution (before gearbox)
TICKS_PER_TURN  = CPR * GEAR_RATIO   # = 750

# Wheel angles [rad]: wheel_1=-60°, wheel_2=+60°, wheel_3=180°  (MCE402 report)
ALPHA = [-math.pi / 3, math.pi / 3, math.pi]


def _rad_per_sec_to_qpps(rad_s: float) -> int:
    """Convert wheel angular velocity [rad/s] to RoboClaw QPPS (ticks/s)."""
    rev_per_sec = rad_s / (2 * math.pi)
    return int(rev_per_sec * TICKS_PER_TURN)


def _qpps_to_rad_per_sec(qpps: int) -> float:
    """Convert RoboClaw QPPS (ticks/s) to wheel angular velocity [rad/s]."""
    rev_per_sec = qpps / TICKS_PER_TURN
    return rev_per_sec * 2 * math.pi


def _ticks_to_rad(ticks: int) -> float:
    return ticks * 2 * math.pi / TICKS_PER_TURN


# ─────────────────────────────────────────────────────────────────────────────
# RoboClawHwNode
# ─────────────────────────────────────────────────────────────────────────────

class RoboClawHwNode(Node):

    def __init__(self):
        super().__init__('roboclaw_hw')

        # Parameters
        self.declare_parameter('serial_port',  '/dev/ttyACM0')
        self.declare_parameter('baud_rate',    38400)
        self.declare_parameter('wheel_radius', 0.05)   # r [m]
        self.declare_parameter('robot_radius', 0.20)   # L [m]
        self.declare_parameter('max_qpps',     2000)   # speed limit [ticks/s]
        self.declare_parameter('cmd_timeout',  0.5)    # [s] stop if no cmd

        port     = self.get_parameter('serial_port').value
        baud     = self.get_parameter('baud_rate').value
        self.r   = self.get_parameter('wheel_radius').value
        self.L   = self.get_parameter('robot_radius').value
        self.max_qpps = self.get_parameter('max_qpps').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # Build kinematics matrices
        self._build_kinematics()

        # Odometry state
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._prev_ticks = [0, 0, 0]
        self._last_cmd_time = time.time()

        # TF broadcaster for odom→base_link
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self._odom_pub  = self.create_publisher(Odometry,    '/odom',        10)
        self._js_pub    = self.create_publisher(JointState,  '/joint_states', 10)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        # Connect to RoboClaw hardware
        self._rc = None
        self._hw_ok = False
        if Roboclaw is None:
            self.get_logger().error(
                'roboclaw_python library not found! '
                'Running in NO-HARDWARE mode (odometry disabled).'
            )
        else:
            try:
                self._rc = Roboclaw(port, baud)
                self._rc.Open()
                self._hw_ok = True
                # Read initial encoder positions as reference
                self._prev_ticks = self._read_ticks_raw()
                self.get_logger().info(f'RoboClaw connected on {port} @ {baud} baud')
            except Exception as e:
                self.get_logger().error(f'RoboClaw connection failed: {e}')

        # Odometry loop at 50 Hz
        self.create_timer(0.02, self._odometry_loop)
        # Safety watchdog at 10 Hz
        self.create_timer(0.10, self._watchdog)

        self.get_logger().info('RoboClawHwNode ready.')

    # ── Kinematics ────────────────────────────────────────────────────────

    def _build_kinematics(self):
        r, L = self.r, self.L
        # Inverse kinematics: wheel_ω = J_inv @ [Vx, Vy, ω_robot]
        # J_inv[i,:] = (1/r) * [-sin(α_i), cos(α_i), L]
        # Wheels at α = -60°, +60°, 180°  (MCE402 report layout)
        J_inv_raw = np.array([
            [-math.sin(a), math.cos(a), L]
            for a in ALPHA
        ])   # shape (3, 3), without the 1/r factor
        self._J_inv = J_inv_raw / r

        # Forward kinematics – MCE402 report formula (verified for -60°/60°/180° layout):
        #   [Vx, Vy, ω]ᵀ = (r/3) · [[√3,-√3,0],[1,1,-2],[1/L,1/L,1/L]] · [ω1,ω2,ω3]ᵀ
        s3 = math.sqrt(3)
        self._J_fwd_raw = np.array([
            [ s3, -s3,   0.0],
            [1.0,  1.0, -2.0],
            [1/L,  1/L,  1/L],
        ]) / 3.0   # shape (3, 3); multiply by r at use-time
        self._r = r

    # ── cmd_vel callback ─────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = time.time()
        v = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        wheel_rad = self._J_inv @ v           # [ω1, ω2, ω3] in rad/s
        qpps_list = [_rad_per_sec_to_qpps(w) for w in wheel_rad]

        # Clamp to max speed
        qpps_list = [max(-self.max_qpps, min(self.max_qpps, q)) for q in qpps_list]

        if self._hw_ok:
            self._send_wheel_speeds(qpps_list)

    def _send_wheel_speeds(self, qpps: list):
        rc = self._rc
        q1, q2, q3 = qpps
        try:
            rc.SpeedM1M2(ADDR_12, q1, q2)
            rc.SpeedM1(ADDR_3, q3)
        except Exception as e:
            self.get_logger().warn(f'RoboClaw write error: {e}')

    # ── Odometry ─────────────────────────────────────────────────────────

    def _read_ticks_raw(self) -> list:
        """Read raw encoder counts from both RoboClaws."""
        rc = self._rc
        try:
            _, e1, _ = rc.ReadEncM1(ADDR_12)
            _, e2, _ = rc.ReadEncM2(ADDR_12)
            _, e3, _ = rc.ReadEncM1(ADDR_3)
            return [e1, e2, e3]
        except Exception as e:
            self.get_logger().warn(f'Encoder read error: {e}')
            return list(self._prev_ticks)

    def _odometry_loop(self):
        if not self._hw_ok:
            return

        now_ticks = self._read_ticks_raw()
        delta_ticks = [now_ticks[i] - self._prev_ticks[i] for i in range(3)]
        self._prev_ticks = now_ticks

        # Convert tick deltas to wheel angle deltas [rad]
        d_ang = np.array([_ticks_to_rad(dt) for dt in delta_ticks])

        # Forward kinematics: body-frame displacement
        # v_body = r * J_fwd_raw @ omega_wheels
        # For small steps: d_local ≈ r * J_fwd_raw @ d_ang
        d_local = self._r * (self._J_fwd_raw @ d_ang)   # [dx_b, dy_b, dθ]

        dx_b, dy_b, dtheta = d_local

        # Rotate local displacement to world frame
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        dx_w =  cos_y * dx_b - sin_y * dy_b
        dy_w =  sin_y * dx_b + cos_y * dy_b

        self._x   += dx_w
        self._y   += dy_w
        self._yaw += dtheta
        self._yaw  = (self._yaw + math.pi) % (2 * math.pi) - math.pi

        # Publish odometry
        stamp = self.get_clock().now().to_msg()
        self._publish_odom(stamp, dx_b / 0.02, dy_b / 0.02, dtheta / 0.02)
        self._broadcast_tf(stamp)

    def _publish_odom(self, stamp, vx, vy, vw):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'

        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.w = math.cos(self._yaw / 2)
        msg.pose.pose.orientation.z = math.sin(self._yaw / 2)

        msg.twist.twist.linear.x  = vx
        msg.twist.twist.linear.y  = vy
        msg.twist.twist.angular.z = vw

        self._odom_pub.publish(msg)

    def _broadcast_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp    = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.w    = math.cos(self._yaw / 2)
        tf.transform.rotation.z    = math.sin(self._yaw / 2)
        self._tf_broadcaster.sendTransform(tf)

    # ── Safety watchdog ───────────────────────────────────────────────────

    def _watchdog(self):
        """Stop motors if cmd_vel is stale."""
        if self._hw_ok and (time.time() - self._last_cmd_time) > self.cmd_timeout:
            self._send_wheel_speeds([0, 0, 0])


def main(args=None):
    rclpy.init(args=args)
    node = RoboClawHwNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._hw_ok:
            node._send_wheel_speeds([0, 0, 0])
        node.destroy_node()
        rclpy.shutdown()
