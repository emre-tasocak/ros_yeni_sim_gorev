"""
DWA Dynamic Obstacle Avoidance Node

Uses the Dynamic Window Approach (DWA) to convert the raw velocity
command from the navigation node into a safe velocity command.

Architecture:
  /cmd_vel_nav  (desired velocity from navigation)
  /scan/filtered (LiDAR data)
         ↓
    [DWA Algorithm]
         ↓
  /cmd_vel (obstacle-free safe velocity)

DWA Logic:
1. Generate velocity samples aligned with goal direction (dynamic window)
2. Simulate trajectory for each sample
3. Reject unsafe trajectories (obstacle < safety_radius)
4. Select best-scoring trajectory:
   score = α*goal_alignment + β*obstacle_distance + γ*speed_magnitude
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(Node):
    """DWA-based dynamic obstacle avoidance node."""

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # --- Parameters ---
        self.declare_parameter('max_linear_velocity', 0.3)
        self.declare_parameter('max_acceleration', 0.5)
        self.declare_parameter('lidar_min_range', 0.30)
        self.declare_parameter('lidar_max_range', 8.0)
        self.declare_parameter('obstacle_safety_radius', 0.50)
        self.declare_parameter('dwa_predict_time', 2.0)
        self.declare_parameter('dwa_dt', 0.1)
        self.declare_parameter('dwa_velocity_samples', 11)
        self.declare_parameter('dwa_alpha', 1.5)
        self.declare_parameter('dwa_beta', 3.0)
        self.declare_parameter('dwa_gamma', 0.3)
        self.declare_parameter('dwa_min_clearance', 0.55)
        self.declare_parameter('control_frequency', 20.0)

        self.max_v = self.get_parameter('max_linear_velocity').value
        self.max_acc = self.get_parameter('max_acceleration').value
        self.lidar_min = self.get_parameter('lidar_min_range').value
        self.lidar_max = self.get_parameter('lidar_max_range').value
        self.safety_r = self.get_parameter('obstacle_safety_radius').value
        self.predict_t = self.get_parameter('dwa_predict_time').value
        self.dt = self.get_parameter('dwa_dt').value
        self.n_samples = self.get_parameter('dwa_velocity_samples').value
        self.alpha = self.get_parameter('dwa_alpha').value
        self.beta = self.get_parameter('dwa_beta').value
        self.gamma = self.get_parameter('dwa_gamma').value
        self.min_clearance = self.get_parameter('dwa_min_clearance').value

        # Current velocity (used for dynamic window computation)
        self.current_vx = 0.0
        self.current_vy = 0.0

        # LiDAR data (latest received)
        self.scan_ranges: list = []
        self.scan_angles: list = []
        self.has_scan = False

        # --- Publishers / Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cmd_vel_nav_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self._nav_cmd_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan/filtered', self._scan_callback, 10)

        self.get_logger().info('DWA obstacle avoidance node started.')

    def _scan_callback(self, msg: LaserScan):
        """Updates LiDAR data."""
        ranges = []
        angles = []
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and self.lidar_min <= r <= self.lidar_max:
                angle = msg.angle_min + i * msg.angle_increment
                ranges.append(r)
                angles.append(angle)
        self.scan_ranges = ranges
        self.scan_angles = angles
        self.has_scan = True

    def _nav_cmd_callback(self, msg: Twist):
        """
        Receives navigation velocity command, computes safe velocity via DWA, publishes it.
        """
        desired_vx = msg.linear.x
        desired_vy = msg.linear.y
        desired_omega = msg.angular.z

        # If no LiDAR data yet, pass through directly
        if not self.has_scan or len(self.scan_ranges) == 0:
            self.cmd_vel_pub.publish(msg)
            return

        # Nearest obstacle distance
        nearest = min(self.scan_ranges) if self.scan_ranges else self.lidar_max

        # Run DWA only when obstacle is within threat zone
        if nearest > self.safety_r * 2.5:
            # Obstacle far enough — pass desired velocity directly
            safe_vx, safe_vy = desired_vx, desired_vy
        else:
            safe_vx, safe_vy = self._dwa_step(desired_vx, desired_vy)

            # DWA found no path (robot stuck at boundary):
            # If desired velocity moves AWAY from obstacle, allow it (slow)
            if safe_vx == 0.0 and safe_vy == 0.0 and (desired_vx != 0.0 or desired_vy != 0.0):
                safe_vx, safe_vy = self._escape_if_moving_away(
                    desired_vx, desired_vy, nearest)

        # Output command
        out = Twist()
        out.linear.x = safe_vx
        out.linear.y = safe_vy
        out.angular.z = desired_omega
        self.cmd_vel_pub.publish(out)

        # Update current velocity
        self.current_vx = safe_vx
        self.current_vy = safe_vy

    def _dwa_step(self, desired_vx: float, desired_vy: float):
        """
        DWA step: generates samples around desired velocity,
        computes clearance for each, returns the best one.
        """
        # Dynamic window (from current velocity and acceleration limits)
        dt_ctrl = 1.0 / 20.0  # control period
        dv = self.max_acc * dt_ctrl

        vx_min = max(-self.max_v, self.current_vx - dv)
        vx_max = min(self.max_v, self.current_vx + dv)
        vy_min = max(-self.max_v, self.current_vy - dv)
        vy_max = min(self.max_v, self.current_vy + dv)

        # Velocity samples
        vx_samples = np.linspace(vx_min, vx_max, self.n_samples)
        vy_samples = np.linspace(vy_min, vy_max, self.n_samples)

        best_score = -1e9
        best_vx, best_vy = 0.0, 0.0

        for vx in vx_samples:
            for vy in vy_samples:
                speed = math.sqrt(vx**2 + vy**2)
                if speed > self.max_v:
                    continue

                # Minimum clearance along trajectory
                clearance = self._compute_clearance(vx, vy)

                # Unsafe trajectory — skip
                if clearance < self.min_clearance:
                    continue

                # Score computation
                # 1) How well aligned with desired velocity?
                goal_dist = math.sqrt((vx - desired_vx)**2 + (vy - desired_vy)**2)
                heading_score = 1.0 / (1.0 + goal_dist)

                # 2) How far from obstacles? (normalized)
                clearance_score = min(clearance / self.lidar_max, 1.0)

                # 3) Speed magnitude (encourages progress)
                speed_score = speed / self.max_v

                score = (self.alpha * heading_score +
                         self.beta * clearance_score +
                         self.gamma * speed_score)

                if score > best_score:
                    best_score = score
                    best_vx, best_vy = vx, vy

        if best_score == -1e9:
            # No safe trajectory found — stop
            self.get_logger().warn('DWA: No safe trajectory found, stopping!',
                                   throttle_duration_sec=1.0)
            return 0.0, 0.0

        return best_vx, best_vy

    def _compute_clearance(self, vx: float, vy: float) -> float:
        """
        Computes minimum distance to any obstacle along the trajectory
        generated by the given velocity command.

        Robot moves at constant vx, vy for predict_t seconds.
        Robot position is checked at each dt step.
        """
        if not self.scan_ranges:
            return self.lidar_max

        min_clearance = self.lidar_max

        # Robot position at each time step
        steps = int(self.predict_t / self.dt)
        for step in range(1, steps + 1):
            t = step * self.dt
            # Predicted position in robot frame
            px = vx * t
            py = vy * t

            # Distance from this position to each obstacle
            for r, angle in zip(self.scan_ranges, self.scan_angles):
                ox = r * math.cos(angle)
                oy = r * math.sin(angle)
                dist = math.sqrt((px - ox)**2 + (py - oy)**2)
                if dist < min_clearance:
                    min_clearance = dist

        return min_clearance

    def _escape_if_moving_away(self, desired_vx: float, desired_vy: float,
                                nearest: float):
        """
        Last resort when DWA finds no path: robot is stuck at boundary.
        If desired velocity moves AWAY from obstacle, allow slow passage.

        How it works: find direction of nearest obstacle, check if desired
        velocity is opposed to it. If opposed (moving away), allow at half max_v.
        """
        if not self.scan_ranges:
            return 0.0, 0.0

        # Find direction of nearest obstacle (in laser frame)
        min_r = nearest
        min_angle = 0.0
        for r, angle in zip(self.scan_ranges, self.scan_angles):
            if r <= min_r + 0.01:
                min_angle = angle
                break

        # Unit vector toward obstacle
        obs_dx = math.cos(min_angle)
        obs_dy = math.sin(min_angle)

        # Dot product of desired velocity with obstacle direction
        # Negative → desired velocity moves away from obstacle → safe
        dot = desired_vx * obs_dx + desired_vy * obs_dy
        if dot < 0.0:
            # Moving away, normalize and go at max_v/2
            speed = math.sqrt(desired_vx**2 + desired_vy**2)
            if speed < 1e-6:
                return 0.0, 0.0
            scale = min(self.max_v * 0.5 / speed, 1.0)
            self.get_logger().warn(
                'DWA escape: moving away from obstacle, slow transit.',
                throttle_duration_sec=2.0)
            return desired_vx * scale, desired_vy * scale

        # Moving toward obstacle — stop
        return 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
