#!/usr/bin/env python3
"""Publish TF from Gazebo odometry for SLAM compatibility."""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTFPublisher(Node):
    """Publish odom->base_link TF from Gazebo odometry messages."""

    def __init__(self):
        super().__init__('odom_tf_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/sih_ugv/odometry',
            self.odom_callback,
            10
        )

        self.get_logger().info('Odometry TF publisher initialized')

    def odom_callback(self, msg: Odometry):
        """Convert odometry message to TF and broadcast."""
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
