"""
Simulation Odometry → TF Publisher

Receives /odom messages from Gazebo OdometryPublisher and
publishes the odom → base_footprint TF transform.

Why this is needed:
  OdometryPublisher's own TF output produces 'odom → base_link'.
  But the URDF has a fixed joint base_footprint → base_link.
  Two different nodes cannot parent 'base_link' (creates TF loop).
  This node publishes 'odom → base_footprint'; robot_state_publisher
  completes the 'base_footprint → base_link → laser/camera/...' chain.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class SimOdomTFNode(Node):
    """Converts odometry message to odom→base_footprint TF."""

    def __init__(self):
        super().__init__('sim_odom_tf')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self.get_logger().info('Sim odom→base_footprint TF broadcaster started.')

    def _odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0   # 2D robot: ground plane
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimOdomTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
