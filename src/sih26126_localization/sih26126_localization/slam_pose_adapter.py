#!/usr/bin/env python3
"""SLAM-derived pose adapter for the DRISTHI localization interface.

The adapter looks up the ``map → base_link`` transform produced by
slam_toolbox and republishes it as :class:`PoseStamped` on
``/localization/pose``.

Timestamp strategy
-------------------
The TF transform's own ``header.stamp`` reflects the sim-time at which
SLAM computed the correction.  Under WSL2 with RTF < 1.0 this stamp can
be non-monotonic (SLAM may publish a transform with an older stamp when
it revisits a previous scan), which caused ``mission_manager`` to
compute a negative ``elapsed`` and flag every legitimate pose update as
``IMPLAUSIBLE_POSE_DELTA``.

The fix: publish the pose with the node clock's current time
(``self.get_clock().now()``) instead of the transform's stamp, and
enforce a monotonic guard so we never publish a stamp that is older
than the previously published one.  This guarantees the downstream
``elapsed`` computation is always non-negative.
"""

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener


class SlamPoseAdapter(Node):
    """Convert SLAM-derived map->base_link transform to localization pose."""

    def __init__(self):
        super().__init__('slam_pose_adapter')

        # TF listener for map->base_link transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers for localization interface
        self.pose_pub = self.create_publisher(
            PoseStamped, '/localization/pose', 10)
        self.source_pub = self.create_publisher(
            String, '/localization/source', 10)
        self.confidence_pub = self.create_publisher(
            Float32, '/localization/confidence', 10)

        # Timer to periodically publish pose from TF
        self.declare_parameter('publish_rate', 10.0)
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.publish_pose)

        # Track SLAM health
        self.last_successful_lookup = None
        self.consecutive_failures = 0

        # Monotonic timestamp guard: never publish a pose stamp older than
        # the previous one.  Initialised to None so the first publish is
        # always accepted.
        self._last_published_stamp_ns: int | None = None

        self.get_logger().info('SLAM pose adapter initialized')

    def publish_pose(self):
        """Extract pose from map->base_link transform and publish."""
        try:
            # Look up the transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            # Use the node clock's current time as the published timestamp.
            # The transform's own stamp (transform.header.stamp) can be
            # non-monotonic under WSL2 low-RTF conditions, causing
            # mission_manager to compute negative elapsed times and flag
            # every legitimate pose as IMPLAUSIBLE_POSE_DELTA.
            now_stamp = self.get_clock().now().to_msg()

            # Monotonic guard: ensure we never publish an older stamp than
            # the previous one.  If the clock jumped backwards (rare but
            # possible during sim-time startup), reuse the previous stamp
            # so elapsed is zero rather than negative.
            now_ns = rclpy.time.Time.from_msg(now_stamp).nanoseconds
            if (
                self._last_published_stamp_ns is not None
                and now_ns < self._last_published_stamp_ns
            ):
                now_stamp = rclpy.time.Time(
                    nanoseconds=self._last_published_stamp_ns
                ).to_msg()
                self.get_logger().debug(
                    'Clock moved backwards; reusing previous stamp for '
                    'monotonicity',
                    throttle_duration_sec=5.0,
                )
            self._last_published_stamp_ns = max(
                now_ns,
                self._last_published_stamp_ns or 0,
            )

            # Convert to PoseStamped
            pose = PoseStamped()
            pose.header.stamp = now_stamp
            pose.header.frame_id = 'map'

            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z

            pose.pose.orientation = transform.transform.rotation

            self.pose_pub.publish(pose)

            # Publish source information
            source = String()
            source.data = 'LiDAR SLAM (slam_toolbox)'
            self.source_pub.publish(source)

            # Calculate confidence based on SLAM health
            confidence = Float32()
            if self.consecutive_failures < 3:
                confidence.data = 1.0  # High confidence when SLAM works
            elif self.consecutive_failures < 10:
                confidence.data = 0.5  # Medium with occasional failures
            else:
                confidence.data = 0.1  # Low with persistent issues
            self.confidence_pub.publish(confidence)

            # Reset failure counter
            self.consecutive_failures = 0
            self.last_successful_lookup = self.get_clock().now()

        except Exception as e:
            self.consecutive_failures += 1

            # Only log errors periodically to avoid spam
            if self.consecutive_failures % 20 == 1:
                self.get_logger().warn(
                    f'Could not get map->base_link transform: {e}')

            # Publish low confidence when SLAM transform unavailable
            confidence = Float32()
            confidence.data = 0.0
            self.confidence_pub.publish(confidence)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPoseAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
