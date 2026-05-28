"""
Navigation Node

Drives the robot toward a target point and orientation.
Uses a simple P-controller; obstacle avoidance is handled in a separate node.

Publications:
  /cmd_vel_nav  → Navigation velocity command (picked up by obstacle_avoidance node)
  /goal_reached → Whether goal has been reached (Bool)

Subscriptions:
  /odom      → Current robot pose
  /goal_pose → Target pose (position + orientation)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor


class NavigationNode(Node):
    """ROS2 navigation node (P-controller)."""

    def __init__(self):
        super().__init__('navigation_node')

        # --- Parameters ---
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('angle_tolerance', 0.05)    # ~3 degrees
        self.declare_parameter('heading_gain', 2.0)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('control_frequency', 20.0)

        self.max_v = self.get_parameter('max_linear_velocity').value
        self.max_omega = self.get_parameter('max_angular_velocity').value
        self.tol = self.get_parameter('goal_tolerance').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.Kp = self.get_parameter('heading_gain').value
        self.Kp_ang = self.get_parameter('angular_gain').value
        freq = self.get_parameter('control_frequency').value

        # Current robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Goal pose
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = 0.0
        self.goal_active = False
        self.goal_reached = False

        # --- Publishers / Subscribers ---
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        # Goal listener (mission_node publishes here)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)

        # Goal cancel service
        self.cancel_srv = self.create_service(
            Trigger, '/cancel_goal', self._cancel_goal_srv)

        # Control loop timer
        self.control_timer = self.create_timer(1.0 / freq, self._control_loop)

        self.get_logger().info('Navigation node started.')

    def _odom_callback(self, msg: Odometry):
        """Updates robot pose."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Quaternion → yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _goal_callback(self, msg: PoseStamped):
        """Sets a new goal when received. Extracts yaw from quaternion."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.goal_yaw = math.atan2(siny, cosy)
        self.goal_active = True
        self.goal_reached = False
        self.get_logger().info(
            f'New goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, '
            f'{math.degrees(self.goal_yaw):.1f}°)'
        )

    def _cancel_goal_srv(self, request, response):
        """Cancels the active goal."""
        self.goal_active = False
        self._publish_stop()
        response.success = True
        response.message = 'Goal cancelled.'
        self.get_logger().info('Goal cancelled.')
        return response

    def _control_loop(self):
        """Generates velocity command toward goal using P-controller."""
        if not self.goal_active or self.goal_x is None:
            return

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist = math.sqrt(dx**2 + dy**2)

        # Yaw error
        yaw_error = self._normalize_angle(self.goal_yaw - self.robot_yaw)

        if dist <= self.tol and abs(yaw_error) <= self.angle_tol:
            # Goal reached (position + orientation)
            if not self.goal_reached:
                self.goal_reached = True
                self.goal_active = False
                self._publish_stop()
                goal_msg = Bool()
                goal_msg.data = True
                self.goal_reached_pub.publish(goal_msg)
                self.get_logger().info(
                    f'Goal reached: ({self.robot_x:.2f}, {self.robot_y:.2f}, '
                    f'{math.degrees(self.robot_yaw):.1f}°)'
                )
            return

        # Compute velocity toward goal (P-controller)
        cmd = Twist()

        if dist > self.tol:
            # Position control: speed proportional to distance, capped at max_v
            scale = min(self.Kp * dist, self.max_v) / dist
            vx_global = dx * scale
            vy_global = dy * scale
            
            # Rotate global velocity to robot's local frame
            cmd.linear.x = vx_global * math.cos(self.robot_yaw) + vy_global * math.sin(self.robot_yaw)
            cmd.linear.y = -vx_global * math.sin(self.robot_yaw) + vy_global * math.cos(self.robot_yaw)
        else:
            # Position reached, only rotate
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

        # Angular control: P-controller for yaw
        omega = self.Kp_ang * yaw_error
        omega = max(-self.max_omega, min(self.max_omega, omega))
        cmd.angular.z = omega

        self.cmd_vel_nav_pub.publish(cmd)

        # Log distance every 2 s
        self.get_logger().info(
            f'Dist: {dist:.2f} m  |  yaw_err: {math.degrees(yaw_error):.1f}°'
            f'  |  vx={cmd.linear.x:.2f}  vy={cmd.linear.y:.2f}'
            f'  omega={omega:.2f}',
            throttle_duration_sec=2.0
        )

    def _publish_stop(self):
        """Publishes a zero velocity (stop) command."""
        stop = Twist()
        self.cmd_vel_nav_pub.publish(stop)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalizes angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
