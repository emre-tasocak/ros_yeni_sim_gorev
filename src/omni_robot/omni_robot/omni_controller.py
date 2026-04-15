"""
omni_controller.py
==================
ROS2 node: 3-wheeled omnidirectional robot kinematic controller.

Converts a Twist (cmd_vel) in the robot body frame to individual wheel
angular velocity commands, using the inverse kinematics of the 3-wheel
omnidirectional drive.

Kinematic model (from MCE402 project report):
  Wheel layout (top view, body frame):
    wheel_1 at α =  -60° → ( L·cos60, -L·sin60) = ( 0.10, -0.173)  front-right
    wheel_2 at α =  +60° → ( L·cos60, +L·sin60) = ( 0.10, +0.173)  front-left
    wheel_3 at α =  180° → (-L, 0)              = (-0.20,  0.000)  rear

  Robot +X = forward (between the two front wheels).
  Wheel 3 is exactly behind the centre → Y = 0 (robot frame).

  Inverse kinematics:
    ω_i = (1/r) [ -sin(α_i)·Vx  +  cos(α_i)·Vy  +  L·ω_robot ]

Subscribes:
  /cmd_vel  (geometry_msgs/Twist)  – body-frame velocity

Publishes:
  /wheel_velocities  (std_msgs/Float64MultiArray) – [ω1, ω2, ω3] rad/s
  /wheel_1_joint/cmd_vel (std_msgs/Float64) – individual publishers for
  /wheel_2_joint/cmd_vel   direct joint velocity control
  /wheel_3_joint/cmd_vel

Parameters:
  wheel_radius  (float, 0.05) – r [m]
  robot_radius  (float, 0.20) – L [m]  distance from centre to wheel
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray


class OmniController(Node):

    def __init__(self):
        super().__init__('omni_controller')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('robot_radius', 0.20)

        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('robot_radius').value

        # Wheel angles: -60°, +60°, 180°  (MCE402 report layout)
        alpha = [-math.pi / 3, math.pi / 3, math.pi]

        # Inverse kinematics matrix J_inv (3×3)
        # Row i: (1/r) * [-sin(αi), cos(αi), L]
        self._J_inv = np.array([
            [-math.sin(a), math.cos(a), L]
            for a in alpha
        ]) / r

        # Publishers
        self._vel_pub  = self.create_publisher(Float64MultiArray, '/wheel_velocities', 10)
        self._pub1     = self.create_publisher(Float64, '/wheel_1_joint/cmd_vel', 10)
        self._pub2     = self.create_publisher(Float64, '/wheel_2_joint/cmd_vel', 10)
        self._pub3     = self.create_publisher(Float64, '/wheel_3_joint/cmd_vel', 10)

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        self.get_logger().info(
            f'OmniController ready  r={r} m  L={L} m  '
            f'J_inv:\n{np.round(self._J_inv, 4)}'
        )

    def _cmd_cb(self, msg: Twist):
        v = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        w = self._J_inv @ v        # [ω1, ω2, ω3]  rad/s

        # Publish multi-array (convenient for logging/plotting)
        ma = Float64MultiArray()
        ma.data = w.tolist()
        self._vel_pub.publish(ma)

        # Publish individual joints
        m1, m2, m3 = Float64(), Float64(), Float64()
        m1.data, m2.data, m3.data = float(w[0]), float(w[1]), float(w[2])
        self._pub1.publish(m1)
        self._pub2.publish(m2)
        self._pub3.publish(m3)

        self.get_logger().debug(
            f'Twist({msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.angular.z:.3f}) → '
            f'ω=[{w[0]:.2f}, {w[1]:.2f}, {w[2]:.2f}] rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OmniController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
