"""
Simulation Cmd_Vel Controller

Serves two purposes in Gazebo Harmonic simulation:
  1. /cmd_vel (Twist) → inverse kinematics → /omni_wheel_controller/commands
     (wheel velocities in rad/s for JointGroupVelocityController)
  2. /joint_states → angle delta → /wheel_ticks (Int32MultiArray)
     (so odometry_node can run with the same code as the real robot)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray

from omni_robot_pkg.omni_kinematics import OmniKinematics

# Wheel joint order expected by the controller
JOINT_ORDER = ['wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint']


class SimCmdVelController(Node):

    def __init__(self):
        super().__init__('sim_cmd_vel_controller')

        self.declare_parameter('wheel_radius',        0.05)
        self.declare_parameter('wheel_base',          0.25)
        self.declare_parameter('ticks_per_revolution', 750)
        self.declare_parameter('gear_ratio',           1.0)

        r        = self.get_parameter('wheel_radius').value
        L        = self.get_parameter('wheel_base').value
        ticks    = self.get_parameter('ticks_per_revolution').value
        gear     = self.get_parameter('gear_ratio').value

        self.kinematics = OmniKinematics(
            wheel_radius=r, wheel_base=L,
            ticks_per_rev=ticks, gear_ratio=gear
        )

        # JointGroupVelocityController command publisher
        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray, '/omni_wheel_controller/commands', 10)

        # Wheel tick publisher for odometry node
        self.ticks_pub = self.create_publisher(
            Int32MultiArray, '/wheel_ticks', 10)

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)

        # Wheel joint angles from previous measurement (radians)
        self._last_pos = None

        self.get_logger().info('Simulation cmd_vel controller started.')

    # ------------------------------------------------------------------

    def _cmd_vel_cb(self, msg: Twist):
        """Twist → wheel linear velocity (m/s) → angular velocity (rad/s) → controller."""
        w1_ms, w2_ms, w3_ms = self.kinematics.robot_vel_to_wheel_vel(
            msg.linear.x, msg.linear.y, msg.angular.z
        )
        r = self.kinematics.r
        cmd = Float64MultiArray()
        cmd.data = [w1_ms / r, w2_ms / r, w3_ms / r]
        self.wheel_cmd_pub.publish(cmd)

    def _joint_states_cb(self, msg: JointState):
        """
        JointState angle delta → encoder tick count → /wheel_ticks
        Formula: delta_ticks = delta_rad * ticks_per_rev / (2π)
        """
        pos_map = dict(zip(msg.name, msg.position))
        if not all(j in pos_map for j in JOINT_ORDER):
            return

        current = [pos_map[j] for j in JOINT_ORDER]

        if self._last_pos is None:
            self._last_pos = current
            return

        tpr = self.kinematics.ticks_per_rev
        ticks = []
        for cur, last in zip(current, self._last_pos):
            delta_rad = cur - last
            ticks.append(int(delta_rad * tpr / (2.0 * math.pi)))

        self._last_pos = current

        tick_msg = Int32MultiArray()
        tick_msg.data = ticks
        self.ticks_pub.publish(tick_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimCmdVelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
