#!/usr/bin/env python3
"""Publish static TF for 3D LiDAR sensor frame."""

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class LidarStaticTFPublisher(Node):
    """Publish static transform from base_link to 3D LiDAR frame."""

    def __init__(self):
        super().__init__('lidar_static_tf_publisher')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transform
        self.publish_static_tf()

        self.get_logger().info('LiDAR static TF publisher initialized (lidar_3d)')

    def publish_static_tf(self):
        """Publish static transform from base_link to lidar_3d frame."""
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'sih_ugv/base_link/lidar_3d'

        # 3D LiDAR pose from SDF: 0.32 0 0.75 relative to base_link
        # (same physical position as former 2D lidar)
        t.transform.translation.x = 0.32
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.75

        # No rotation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LidarStaticTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
