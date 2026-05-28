"""
Mission Control Node (User-Input + Obstacle Detection)

Görev Akışı:
  1. Kullanıcıdan hedef konum al (x y phi)
  2. Hedefe git (navigation_node üzerinden)
  3. Yolda engel varsa → engelden 50 cm önce dur → başlangıca dön
  4. Engel yoksa → hedefe ulaş → 2 saniye bekle → başlangıca dön
  5. Yeni hedef bekle (döngü)

Durum Makinesi:
  WAITING_INPUT → GOING_TO_TARGET → [STOPPED_BY_OBSTACLE | AT_TARGET] → RETURNING_HOME → WAITING_INPUT

LiDAR sadece engel algılama için kullanılır (/nearest_obstacle, /obstacle_detected).

Publications:
  /mission_state (String)     → Mevcut durum
  /goal_pose     (PoseStamped) → Navigation node için hedef

Subscriptions:
  /odom              → Robot pozisyonu
  /nearest_obstacle  → En yakın engel mesafesi (Float32)
  /obstacle_detected → Engel var mı? (Bool)
  /goal_reached      → Hedefe ulaşıldı mı? (Bool)
"""

import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float32
from std_srvs.srv import Trigger


# State constants
class State:
    WAITING_INPUT       = 'WAITING_INPUT'
    GOING_TO_TARGET     = 'GOING_TO_TARGET'
    STOPPED_BY_OBSTACLE = 'STOPPED_BY_OBSTACLE'
    AT_TARGET           = 'AT_TARGET'
    RETURNING_HOME      = 'RETURNING_HOME'
    ERROR               = 'ERROR'


class MissionNode(Node):
    """Main mission control ROS2 node."""

    def __init__(self):
        super().__init__('mission_node')

        # --- Parameters ---
        self.declare_parameter('obstacle_stop_distance', 0.50)
        self.declare_parameter('home_tolerance', 0.15)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('wait_at_target', 2.0)
        self.declare_parameter('control_frequency', 20.0)

        self.obstacle_stop_dist = self.get_parameter('obstacle_stop_distance').value
        self.home_tol = self.get_parameter('home_tolerance').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.wait_time = self.get_parameter('wait_at_target').value
        freq = self.get_parameter('control_frequency').value

        # State machine
        self.state = State.WAITING_INPUT

        # Robot home pose (odom frame, assumed 0,0)
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw = 0.0
        self.home_saved = False

        # Current robot pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Target pose (from user input)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0

        # Goal reached flag
        self.goal_reached_flag = False

        # Obstacle data
        self.nearest_obstacle_dist = float('inf')
        self.obstacle_detected = False

        # Wait timer
        self._wait_timer = None
        self._wait_done = False

        # --- Publishers ---
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        # Stop command sent via /cmd_vel_nav (bypasses navigation_node)
        self.stop_cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # --- Subscribers ---
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        # Obstacle detection from lidar_processor
        self.nearest_sub = self.create_subscription(
            Float32, '/nearest_obstacle', self._nearest_obstacle_callback, 10)

        self.obstacle_sub = self.create_subscription(
            Bool, '/obstacle_detected', self._obstacle_detected_callback, 10)

        # Goal reached notification from navigation_node
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self._goal_reached_callback, 10)

        # --- Services ---
        self.stop_srv = self.create_service(
            Trigger, '/stop_mission', self._stop_mission_srv)
        self.cancel_client = self.create_client(Trigger, '/cancel_goal')

        # State machine timer
        self.mission_timer = self.create_timer(
            1.0 / freq, self._state_machine_step)

        # User input thread
        self._input_thread = threading.Thread(
            target=self._get_user_input, daemon=True)
        self._input_thread.start()

        self.get_logger().info('=' * 50)
        self.get_logger().info('Mission node started.')
        self.get_logger().info('LiDAR: obstacle detection only.')
        self.get_logger().info('=' * 50)

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry):
        """Updates robot pose."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

        # Save home position (first odom reading)
        if not self.home_saved:
            self.home_x = self.robot_x
            self.home_y = self.robot_y
            self.home_yaw = self.robot_yaw
            self.home_saved = True
            self.get_logger().info(
                f'Home position saved: ({self.home_x:.2f}, {self.home_y:.2f})')

    def _nearest_obstacle_callback(self, msg: Float32):
        """Updates nearest obstacle distance from lidar_processor."""
        self.nearest_obstacle_dist = msg.data

    def _obstacle_detected_callback(self, msg: Bool):
        """Updates obstacle detection flag from lidar_processor."""
        self.obstacle_detected = msg.data

    def _goal_reached_callback(self, msg: Bool):
        """Navigation node reported goal reached."""
        if msg.data:
            self.goal_reached_flag = True

    # ------------------------------------------------------------------
    # User input (runs in separate thread)
    # ------------------------------------------------------------------

    def _get_user_input(self):
        """Gets target position from user via console input. Phi is in degrees."""
        while rclpy.ok():
            if self.state == State.WAITING_INPUT:
                try:
                    user_input = input('\nHedef konum girin (x y phi[derece]): ')
                    parts = user_input.strip().split()
                    if len(parts) == 3:
                        self.target_x = float(parts[0])
                        self.target_y = float(parts[1])
                        # phi is given in degrees, convert to radians
                        self.target_yaw = math.radians(float(parts[2]))
                        self.state = State.GOING_TO_TARGET
                        self.goal_reached_flag = False
                        self._send_goal(self.target_x, self.target_y, self.target_yaw)
                        self.get_logger().info(
                            f'Target received: x={self.target_x:.2f}, '
                            f'y={self.target_y:.2f}, phi={float(parts[2]):.1f}°'
                            f' ({self.target_yaw:.2f} rad)')
                        self.get_logger().info('Going to target...')
                    else:
                        print('Hatali giris! Format: x y phi (ornek: 1.0 2.0 90)')
                except ValueError:
                    print('Hatali giris! Sayisal degerler girin.')
                except EOFError:
                    break

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _stop_mission_srv(self, request, response):
        """Emergency stop the mission."""
        self._transition_to(State.WAITING_INPUT)
        self._publish_stop()
        response.success = True
        response.message = 'Mission stopped.'
        self.get_logger().warn('Mission emergency stopped!')
        return response

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _state_machine_step(self):
        """Called every control loop iteration."""
        if self.state == State.WAITING_INPUT:
            pass  # Waiting for user input (handled in thread)

        elif self.state == State.GOING_TO_TARGET:
            self._handle_going_to_target()

        elif self.state == State.STOPPED_BY_OBSTACLE:
            self._handle_stopped_by_obstacle()

        elif self.state == State.AT_TARGET:
            self._handle_at_target()

        elif self.state == State.RETURNING_HOME:
            self._handle_returning_home()

        # Publish state message
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

    def _handle_going_to_target(self):
        """
        Navigating to user-specified target.
        Checks for obstacles during transit.
        """
        # Check if obstacle is within stop distance
        if self.nearest_obstacle_dist <= self.obstacle_stop_dist:
            self.get_logger().warn(
                f'OBSTACLE DETECTED at {self.nearest_obstacle_dist:.2f} m! '
                f'Stopping (limit: {self.obstacle_stop_dist:.2f} m).')
            self._publish_stop()
            self._wait_done = False
            self._transition_to(State.STOPPED_BY_OBSTACLE)
            # Start wait timer for obstacle (uses same wait_time = 2.0s)
            self._wait_timer = self.create_timer(
                self.wait_time, self._wait_callback)
            return

        # Check if goal reached
        if self.goal_reached_flag:
            self.get_logger().info(
                f'Target reached: ({self.robot_x:.2f}, {self.robot_y:.2f})')
            self._publish_stop()
            self._wait_done = False
            self._transition_to(State.AT_TARGET)
            # Start wait timer
            self._wait_timer = self.create_timer(
                self.wait_time, self._wait_callback)

    def _handle_stopped_by_obstacle(self):
        """
        Robot stopped because of an obstacle.
        Wait for wait_at_target seconds, then start returning home.
        """
        if self._wait_done:
            self.get_logger().info(
                f'Obstacle wait ({self.wait_time:.0f}s) complete. Returning to home...')
            self.goal_reached_flag = False
            self._send_goal(self.home_x, self.home_y, self.home_yaw)
            self._transition_to(State.RETURNING_HOME)

    def _handle_at_target(self):
        """
        Stopped at target. Waiting for wait_at_target seconds.
        """
        if self._wait_done:
            self.get_logger().info(
                f'{self.wait_time:.0f}s wait complete. Returning to home...')
            self.goal_reached_flag = False
            self._send_goal(self.home_x, self.home_y, self.home_yaw)
            self._transition_to(State.RETURNING_HOME)

    def _handle_returning_home(self):
        """Checks if return home is complete."""
        if self.goal_reached_flag:
            home_dist = math.sqrt(
                (self.robot_x - self.home_x)**2 +
                (self.robot_y - self.home_y)**2
            )
            if home_dist <= self.home_tol:
                self.get_logger().info('=' * 50)
                self.get_logger().info(
                    f'Home reached! Distance: {home_dist:.2f} m')
                self.get_logger().info(
                    f'Position: x={self.robot_x:.2f}, y={self.robot_y:.2f}, '
                    f'yaw={self.robot_yaw:.2f}')
                self.get_logger().info('=' * 50)
                self._publish_stop()
                self._transition_to(State.WAITING_INPUT)
            else:
                # Not reached yet, resend goal
                self.goal_reached_flag = False
                self._send_goal(self.home_x, self.home_y, self.home_yaw)

    # ------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------

    def _transition_to(self, new_state: str):
        """Transitions to new state and logs it."""
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state

    def _send_goal(self, goal_x: float, goal_y: float, goal_yaw: float = 0.0):
        """Publishes goal to navigation node. Yaw encoded as quaternion."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        # Encode yaw as quaternion (rotation around Z axis)
        msg.pose.orientation.z = math.sin(goal_yaw / 2.0)
        msg.pose.orientation.w = math.cos(goal_yaw / 2.0)
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f'Goal sent: ({goal_x:.2f}, {goal_y:.2f}, '
            f'{math.degrees(goal_yaw):.1f}°)')

    def _publish_stop(self):
        """Zeroes all velocity commands (stops motors) and cancels goal."""
        stop = Twist()
        self.stop_cmd_pub.publish(stop)
        if self.cancel_client.wait_for_service(timeout_sec=0.1):
            self.cancel_client.call_async(Trigger.Request())
        self.get_logger().info('STOP command sent and goal cancelled.')

    def _wait_callback(self):
        """Wait timer completed."""
        self._wait_done = True
        if self._wait_timer:
            self._wait_timer.cancel()
            self._wait_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
