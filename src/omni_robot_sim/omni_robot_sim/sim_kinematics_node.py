"""
sim_kinematics_node.py
======================
Simulation kinematics bridge for the 3-wheeled omni robot.

What this node does:
  1. Receives /cmd_vel  →  relays to Gazebo VelocityControl via /gz_cmd_vel
     (body-frame velocity, applied holonomically by gz-sim-velocity-control-system)

  2. Receives /cmd_vel  →  computes wheel angular velocities via J_inv
     →  publishes Float64 to /gz_wheel_{1,2,3}_vel
     (feeds gz-sim-joint-controller-system so wheels SPIN visually)

  3. Receives /odom from the bridge (Gazebo OdometryPublisher)
     →  broadcasts TF odom → base_link
     →  also integrates wheel angles for /joint_states
         so robot_state_publisher shows spinning wheels in RViz

Wheels at α = -60°, +60°, 180°  (MCE402 report layout)
J_inv[i] = (1/r) * [-sin(αi), cos(αi), L]
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import tf2_ros


# ── Kinematic constants ──────────────────────────────────────────────────────
R = 0.05   # wheel radius  [m]
L = 0.20   # robot radius  [m]
ALPHA = [-math.pi / 3, math.pi / 3, math.pi]   # [-60°, +60°, 180°]

J_INV = np.array([
    [-math.sin(a), math.cos(a), L]
    for a in ALPHA
]) / R   # (3×3)

WHEEL_NAMES = ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint']


class SimKinematicsNode(Node):

    def __init__(self):
        super().__init__('sim_kinematics')

        # TF broadcaster for odom → base_link
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self._gz_cmd_pub = self.create_publisher(Twist,      '/gz_cmd_vel',      10)
        self._w1_pub     = self.create_publisher(Float64,    '/gz_wheel_1_vel',  10)
        self._w2_pub     = self.create_publisher(Float64,    '/gz_wheel_2_vel',  10)
        self._w3_pub     = self.create_publisher(Float64,    '/gz_wheel_3_vel',  10)
        self._js_pub     = self.create_publisher(JointState, '/joint_states',    10)

        # Subscribers
        self.create_subscription(Twist,    '/cmd_vel', self._cmd_cb,  10)
        self.create_subscription(Odometry, '/odom',    self._odom_cb, 10)

        # Integrated wheel angles for joint_states [rad]
        self._wheel_angle = [0.0, 0.0, 0.0]
        self._wheel_speed = [0.0, 0.0, 0.0]   # rad/s

        # Timer: update joint_states at 50 Hz (independent of cmd_vel)
        self._dt = 0.02
        self.create_timer(self._dt, self._joint_state_timer)

        self.get_logger().info(
            'SimKinematicsNode ready.\n'
            '  /cmd_vel → /gz_cmd_vel  (Gazebo VelocityControl)\n'
            '  /cmd_vel → /gz_wheel_{1,2,3}_vel  (wheel spinning)\n'
            '  /odom    → TF odom→base_link'
        )

    # ── cmd_vel callback ────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        # 1. Relay directly to Gazebo VelocityControl (body frame)
        self._gz_cmd_pub.publish(msg)

        # 2. Compute wheel angular velocities  [rad/s]
        v = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        wheel_rad_s = J_INV @ v

        # Store for joint_state integration
        self._wheel_speed = wheel_rad_s.tolist()

        # 3. Publish to each wheel joint controller
        for pub, speed in zip([self._w1_pub, self._w2_pub, self._w3_pub], wheel_rad_s):
            m = Float64()
            m.data = float(speed)
            pub.publish(m)

    # ── Odometry callback → TF ──────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        tf = TransformStamped()
        tf.header.stamp    = msg.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'

        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z

        tf.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(tf)

    # ── Joint state timer ───────────────────────────────────────────────────

    def _joint_state_timer(self):
        """Integrate wheel speeds → wheel angles → publish /joint_states."""
        for i in range(3):
            self._wheel_angle[i] += self._wheel_speed[i] * self._dt

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = WHEEL_NAMES
        js.position = list(self._wheel_angle)
        js.velocity = list(self._wheel_speed)
        self._js_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = SimKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        node._gz_cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
