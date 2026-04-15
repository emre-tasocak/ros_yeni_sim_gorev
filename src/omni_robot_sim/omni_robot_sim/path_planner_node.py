"""
path_planner_node.py
====================
ROS2 node that executes smooth 5th-order (quintic) polynomial trajectories
for the 3-wheeled omnidirectional robot.

Subscribes:
  /odom          (nav_msgs/Odometry)   – current robot pose
  /goal_pose     (geometry_msgs/PoseStamped) – navigation goal
  /path_waypoints (nav_msgs/Path)      – list of waypoints from RRT

Publishes:
  /cmd_vel       (geometry_msgs/Twist) – velocity command for omni controller

Trajectory planning algorithm:
  - Quintic (5th-order) polynomial in x(t), y(t), θ(t)
  - Boundary conditions: pos/vel/acc = 0 at start and end
  - Converts world-frame velocity → body-frame Twist for cmd_vel

Reference: MCE402 Project Report – PathPlanning.py (Chebyshev/quintic section)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool


# ─────────────────────────────────────────────────────────────────────────────
# Quintic polynomial helpers
# ─────────────────────────────────────────────────────────────────────────────

def quintic_coeffs(p0: float, pf: float, tf: float) -> np.ndarray:
    """
    Compute coefficients [a0..a5] for a quintic polynomial
    with zero velocity and acceleration at both endpoints.

    p(t)  = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    p(0)  = p0,  p(tf) = pf
    p'(0) = 0,   p'(tf)= 0
    p''(0)= 0,   p''(tf)= 0
    """
    T = tf
    A = np.array([
        [1, 0,    0,       0,         0,          0        ],
        [0, 1,    0,       0,         0,          0        ],
        [0, 0,    2,       0,         0,          0        ],
        [1, T,    T**2,    T**3,      T**4,       T**5     ],
        [0, 1,    2*T,     3*T**2,    4*T**3,     5*T**4   ],
        [0, 0,    2,       6*T,       12*T**2,    20*T**3  ],
    ])
    b = np.array([p0, 0, 0, pf, 0, 0], dtype=float)
    return np.linalg.solve(A, b)


def quintic_eval(coeffs: np.ndarray, t: float):
    """Evaluate position, velocity, acceleration of quintic at time t."""
    a = coeffs
    pos = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5
    vel = a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4
    return pos, vel


def shortest_angle(a: float, b: float) -> float:
    """Shortest-path angle difference b - a, wrapped to [-π, π]."""
    diff = (b - a + math.pi) % (2 * math.pi) - math.pi
    return diff


# ─────────────────────────────────────────────────────────────────────────────
# PathPlannerNode
# ─────────────────────────────────────────────────────────────────────────────

class PathPlannerNode(Node):

    def __init__(self):
        super().__init__('path_planner')

        # Parameters
        self.declare_parameter('max_linear_vel',  0.4)   # m/s
        self.declare_parameter('max_angular_vel', 1.5)   # rad/s
        self.declare_parameter('traj_time_scale', 1.0)   # speed multiplier
        self.declare_parameter('goal_tolerance',  0.05)  # m

        self.v_max   = self.get_parameter('max_linear_vel').value
        self.w_max   = self.get_parameter('max_angular_vel').value
        self.t_scale = self.get_parameter('traj_time_scale').value
        self.tol     = self.get_parameter('goal_tolerance').value

        # Current robot state
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # Trajectory state
        self._waypoints: list = []      # list of (x, y, yaw)
        self._traj_x    = None          # quintic coeffs for X
        self._traj_y    = None
        self._traj_yaw  = None
        self._tf        = 0.0           # total trajectory time [s]
        self._t_elapsed = 0.0           # time since trajectory started [s]
        self._running   = False

        # Publishers / Subscribers
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._done_pub = self.create_publisher(Bool,  '/path_done', 10)

        self.create_subscription(Odometry,     '/odom',         self._odom_cb,      10)
        self.create_subscription(PoseStamped,  '/goal_pose',    self._goal_cb,      10)
        self.create_subscription(Path,         '/path_waypoints', self._waypoints_cb, 10)

        # Control loop at 50 Hz
        self._dt = 0.02
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info('PathPlannerNode started (quintic polynomial trajectory)')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # quaternion → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def _goal_cb(self, msg: PoseStamped):
        """Single goal pose → plan direct quintic trajectory."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q  = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        gyaw = math.atan2(siny, cosy)
        self._plan_segment(self.x, self.y, self.yaw, gx, gy, gyaw)
        self.get_logger().info(f'Goal received → quintic to ({gx:.2f}, {gy:.2f})')

    def _waypoints_cb(self, msg: Path):
        """Sequence of waypoints from RRT → queue segments."""
        wps = []
        for ps in msg.poses:
            px = ps.pose.position.x
            py = ps.pose.position.y
            q  = ps.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            pyaw = math.atan2(siny, cosy)
            wps.append((px, py, pyaw))
        self._waypoints = wps
        self.get_logger().info(f'Received {len(wps)}-waypoint path from RRT')
        self._start_next_waypoint()

    # ── Trajectory planning ───────────────────────────────────────────────

    def _plan_segment(self, x0, y0, yaw0, xf, yf, yawf):
        """Fit quintic polynomial from current pose to goal pose."""
        dist    = math.hypot(xf - x0, yf - y0)
        dyaw    = abs(shortest_angle(yaw0, yawf))
        # Estimate time from distance + rotation (conservative)
        tf = max(dist / (self.v_max * self.t_scale),
                 dyaw / (self.w_max * self.t_scale),
                 1.0)   # at least 1 s

        self._traj_x   = quintic_coeffs(x0,   xf,   tf)
        self._traj_y   = quintic_coeffs(y0,   yf,   tf)
        # Use cumulative angle (avoid wrap) for smooth interpolation
        self._traj_yaw = quintic_coeffs(yaw0, yaw0 + shortest_angle(yaw0, yawf), tf)
        self._tf       = tf
        self._t_elapsed = 0.0
        self._running   = True

    def _start_next_waypoint(self):
        if not self._waypoints:
            self._running = False
            msg = Bool()
            msg.data = True
            self._done_pub.publish(msg)
            self.get_logger().info('All waypoints reached!')
            return
        wp = self._waypoints.pop(0)
        self._plan_segment(self.x, self.y, self.yaw, wp[0], wp[1], wp[2])

    # ── Control loop ─────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._running:
            return

        t = min(self._t_elapsed, self._tf)

        # Desired world-frame position and velocity from quintic
        _, vx_w = quintic_eval(self._traj_x, t)
        _, vy_w = quintic_eval(self._traj_y, t)
        yaw_d, w = quintic_eval(self._traj_yaw, t)

        # Rotate world-frame velocity into robot body frame
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        vx_b =  cos_yaw * vx_w + sin_yaw * vy_w
        vy_b = -sin_yaw * vx_w + cos_yaw * vy_w

        # Clamp to hardware limits
        speed = math.hypot(vx_b, vy_b)
        if speed > self.v_max:
            vx_b *= self.v_max / speed
            vy_b *= self.v_max / speed
        w = max(-self.w_max, min(self.w_max, w))

        cmd = Twist()
        cmd.linear.x  = vx_b
        cmd.linear.y  = vy_b
        cmd.angular.z = w
        self._cmd_pub.publish(cmd)

        self._t_elapsed += self._dt

        # Segment finished
        if self._t_elapsed >= self._tf:
            # Check if we are close enough to goal
            if self._waypoints:
                self._start_next_waypoint()
            else:
                self._stop_robot()
                self._running = False
                self.get_logger().info('Trajectory complete.')

    def _stop_robot(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()
