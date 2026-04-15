"""
rrt_node.py
===========
ROS2 node that builds an occupancy grid from LiDAR scan data and runs
Bidirectional RRT (RRT-Connect) to plan collision-free paths.

Subscribes:
  /scan       (sensor_msgs/LaserScan)  – 2D LiDAR
  /odom       (nav_msgs/Odometry)      – robot pose
  /goal_pose  (geometry_msgs/PoseStamped) – navigation goal

Publishes:
  /path_waypoints (nav_msgs/Path)      – planned path for PathPlannerNode
  /rrt_grid       (nav_msgs/OccupancyGrid) – debug occupancy grid

Reference: MCE402 Project Report – RRT.py (Bidirectional RRT + YDLidar X2)
"""

import math
import random
import time
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


# ─────────────────────────────────────────────────────────────────────────────
# Utility
# ─────────────────────────────────────────────────────────────────────────────

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2)
    q.z = math.sin(yaw / 2)
    return q


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Return all grid cells on line from (x0,y0) to (x1,y1)."""
    cells = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells


# ─────────────────────────────────────────────────────────────────────────────
# OccupancyGrid helper
# ─────────────────────────────────────────────────────────────────────────────

class GridMap:
    """Simple 2D occupancy grid centred on the origin."""

    UNKNOWN  = -1
    FREE     =  0
    OCCUPIED = 100

    def __init__(self, width_m: float, height_m: float, resolution: float):
        self.res   = resolution
        self.w     = int(width_m  / resolution)
        self.h     = int(height_m / resolution)
        self.ox    = self.w // 2   # origin cell x
        self.oy    = self.h // 2   # origin cell y
        self.grid  = np.full((self.h, self.w), self.FREE, dtype=np.int8)

    def world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        cx = int(wx / self.res) + self.ox
        cy = int(wy / self.res) + self.oy
        return cx, cy

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        wx = (cx - self.ox) * self.res
        wy = (cy - self.oy) * self.res
        return wx, wy

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.w and 0 <= cy < self.h

    def is_free(self, cx: int, cy: int) -> bool:
        if not self.in_bounds(cx, cy):
            return False
        return self.grid[cy, cx] < 50   # below 50 → free

    def mark_obstacle(self, cx: int, cy: int, inflate: int = 2):
        for dx in range(-inflate, inflate + 1):
            for dy in range(-inflate, inflate + 1):
                nx, ny = cx + dx, cy + dy
                if self.in_bounds(nx, ny):
                    self.grid[ny, nx] = self.OCCUPIED

    def reset_free_zone(self, radius_cells: int, origin_cx: int, origin_cy: int):
        """Mark cells near the robot as free (it clearly navigated there)."""
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                nx, ny = origin_cx + dx, origin_cy + dy
                if self.in_bounds(nx, ny):
                    self.grid[ny, nx] = self.FREE


# ─────────────────────────────────────────────────────────────────────────────
# Bidirectional RRT
# ─────────────────────────────────────────────────────────────────────────────

Node_ = Tuple[float, float]   # (x, y) in world frame


class BidirectionalRRT:
    def __init__(
        self,
        grid: GridMap,
        step: float = 0.15,    # m per expansion step
        max_iter: int = 3000,
        goal_bias: float = 0.1,
    ):
        self.grid      = grid
        self.step      = step
        self.max_iter  = max_iter
        self.goal_bias = goal_bias

    def _random_sample(self, goal: Node_) -> Node_:
        if random.random() < self.goal_bias:
            return goal
        wx = random.uniform(-(self.grid.ox * self.grid.res),
                             (self.grid.w - self.grid.ox) * self.grid.res)
        wy = random.uniform(-(self.grid.oy * self.grid.res),
                             (self.grid.h - self.grid.oy) * self.grid.res)
        return wx, wy

    def _nearest(self, tree: dict, sample: Node_) -> Node_:
        sx, sy = sample
        return min(tree.keys(), key=lambda n: (n[0]-sx)**2 + (n[1]-sy)**2)

    def _steer(self, src: Node_, dst: Node_) -> Optional[Node_]:
        dx, dy = dst[0] - src[0], dst[1] - src[1]
        dist   = math.hypot(dx, dy)
        if dist < 1e-6:
            return None
        nx = src[0] + (dx / dist) * min(self.step, dist)
        ny = src[1] + (dy / dist) * min(self.step, dist)
        return nx, ny

    def _collision_free(self, a: Node_, b: Node_) -> bool:
        """Bresenham line check on occupancy grid."""
        ax, ay = self.grid.world_to_cell(a[0], a[1])
        bx, by = self.grid.world_to_cell(b[0], b[1])
        for cx, cy in bresenham(ax, ay, bx, by):
            if not self.grid.is_free(cx, cy):
                return False
        return True

    def _reconstruct(self, tree: dict, node: Node_) -> List[Node_]:
        path = []
        cur  = node
        while cur is not None:
            path.append(cur)
            cur = tree[cur]
        return list(reversed(path))

    def plan(self, start: Node_, goal: Node_) -> Optional[List[Node_]]:
        """Return list of (x,y) waypoints from start to goal, or None."""
        # trees: node → parent
        tree_s = {start: None}
        tree_g = {goal:  None}

        for _ in range(self.max_iter):
            # Expand tree from start toward random sample
            sample   = self._random_sample(goal)
            near_s   = self._nearest(tree_s, sample)
            new_s    = self._steer(near_s, sample)
            if new_s and self._collision_free(near_s, new_s):
                tree_s[new_s] = near_s

                # Try to connect goal-tree to new_s
                near_g = self._nearest(tree_g, new_s)
                new_g  = self._steer(near_g, new_s)
                if new_g and self._collision_free(near_g, new_g):
                    tree_g[new_g] = near_g

                    # Check if trees are connected (close enough)
                    if math.hypot(new_s[0] - new_g[0], new_s[1] - new_g[1]) < self.step:
                        path_s = self._reconstruct(tree_s, new_s)
                        path_g = self._reconstruct(tree_g, new_g)
                        return path_s + list(reversed(path_g))

            # Swap trees each iteration (bidirectional)
            tree_s, tree_g = tree_g, tree_s

        return None   # failed


# ─────────────────────────────────────────────────────────────────────────────
# RRT ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class RRTNode(Node):

    def __init__(self):
        super().__init__('rrt_planner')

        self.declare_parameter('grid_width',      8.0)   # m
        self.declare_parameter('grid_height',     6.0)   # m
        self.declare_parameter('grid_resolution', 0.05)  # m/cell
        self.declare_parameter('rrt_step',        0.15)  # m
        self.declare_parameter('rrt_max_iter',    3000)
        self.declare_parameter('inflate_cells',   2)     # obstacle inflation

        gw   = self.get_parameter('grid_width').value
        gh   = self.get_parameter('grid_height').value
        res  = self.get_parameter('grid_resolution').value
        step = self.get_parameter('rrt_step').value
        itr  = self.get_parameter('rrt_max_iter').value
        inf  = self.get_parameter('inflate_cells').value

        self._grid    = GridMap(gw, gh, res)
        self._rrt     = BidirectionalRRT(self._grid, step=step, max_iter=itr)
        self._inflate = inf

        self._robot_x  = 0.0
        self._robot_y  = 0.0
        self._robot_yaw = 0.0

        # Publishers
        self._path_pub = self.create_publisher(Path, '/path_waypoints', 10)
        self._grid_pub = self.create_publisher(OccupancyGrid, '/rrt_grid', 10)

        # Subscribers
        self.create_subscription(LaserScan,   '/scan',      self._scan_cb,   10)
        self.create_subscription(Odometry,    '/odom',      self._odom_cb,   10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb,   10)

        self.get_logger().info('RRTNode started (bidirectional RRT + LiDAR grid)')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _scan_cb(self, msg: LaserScan):
        """Update occupancy grid from LiDAR readings."""
        rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw

        # Clear robot neighbourhood
        rcx, rcy = self._grid.world_to_cell(rx, ry)
        self._grid.reset_free_zone(5, rcx, rcy)

        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            angle += msg.angle_increment
            if math.isinf(r) or math.isnan(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            # World-frame obstacle position
            ox = rx + r * math.cos(ryaw + angle)
            oy = ry + r * math.sin(ryaw + angle)
            cx, cy = self._grid.world_to_cell(ox, oy)
            if self._grid.in_bounds(cx, cy):
                self._grid.mark_obstacle(cx, cy, self._inflate)

    def _goal_cb(self, msg: PoseStamped):
        """Received a new goal → run RRT and publish path."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        q  = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        gyaw = math.atan2(siny, cosy)

        start = (self._robot_x, self._robot_y)
        goal  = (gx, gy)

        self.get_logger().info(f'RRT planning from {start} to ({gx:.2f},{gy:.2f}) …')
        t0    = time.time()
        raw   = self._rrt.plan(start, goal)
        dt    = time.time() - t0

        if raw is None:
            self.get_logger().warn(f'RRT failed after {dt:.2f}s – no path found')
            return

        self.get_logger().info(f'RRT found path ({len(raw)} nodes) in {dt:.2f}s')

        # Build ROS Path message (assign heading = direction of travel)
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for i, (px, py) in enumerate(raw):
            # Heading toward next point (or keep last heading)
            if i + 1 < len(raw):
                nx, ny = raw[i + 1]
                heading = math.atan2(ny - py, nx - px)
            else:
                heading = gyaw

            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x    = px
            ps.pose.position.y    = py
            ps.pose.orientation   = yaw_to_quat(heading)
            path_msg.poses.append(ps)

        self._path_pub.publish(path_msg)

        # Publish occupancy grid for RViz debug
        self._publish_grid()

    # ── Grid visualisation ────────────────────────────────────────────────

    def _publish_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.info.resolution = self._grid.res
        msg.info.width      = self._grid.w
        msg.info.height     = self._grid.h
        msg.info.origin.position.x = -(self._grid.ox * self._grid.res)
        msg.info.origin.position.y = -(self._grid.oy * self._grid.res)
        msg.data = self._grid.grid.flatten().tolist()
        self._grid_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RRTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
