#!/usr/bin/env python3
"""Regression test: LiDAR processor TF-gated scan publication.

Verifies that:
1. Scans are not published when odom->base_link TF is unavailable at the
   cloud's timestamp (LookupException, ExtrapolationException).
2. Scans are published when odom->base_link TF exists at the cloud's
   timestamp, preserving the original cloud timestamp.
3. Scans dropped due to TF unavailability do not block subsequent clouds.
4. Published scans have correct frame_id and scan geometry.
5. No queue accumulation or deadlock: each cloud is independently checked.
"""

import math
import unittest
from unittest.mock import Mock, MagicMock, patch

import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import tf2_ros


def _make_pointcloud(timestamp_sec: float, frame_id: str = 'sih_ugv/base_link/lidar_3d') -> PointCloud2:
    """Create a minimal PointCloud2 with one point at (1, 0, 0)."""
    cloud = PointCloud2()
    cloud.header.stamp.sec = int(timestamp_sec)
    cloud.header.stamp.nanosec = int(
        (timestamp_sec - int(timestamp_sec)) * 1e9)
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = 1
    cloud.point_step = 12
    cloud.row_step = 12
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.data = np.array([1.0, 0.0, 0.0], dtype=np.float32).tobytes()
    cloud.is_dense = True
    return cloud


class TestLidarProcessorTFSync(unittest.TestCase):
    """Test LiDAR processor TF-gated scan publication."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        """Create a LidarProcessor with mocked ROS interfaces.

        We mock the Node base class methods to avoid creating real
        publishers/subscribers, then manually set up the attributes the
        _cloud_callback and _process_cloud methods need.  This tests the
        actual production callback logic, not a simplified copy.
        """
        param_values = {
            'lidar_frame': 'sih_ugv/base_link/lidar_3d',
            'angle_min': -2.35619,
            'angle_max': 2.35619,
            'scan_bins': 720,
            'range_min': 0.2,
            'range_max': 20.0,
            'vertical_tolerance_deg': 5.0,
            'stale_timeout_sec': 2.0,
            'min_valid_ratio': 0.05,
        }

        def _mock_get_parameter(name):
            mock_param = Mock()
            mock_param.value = param_values.get(name, None)
            return mock_param

        mock_logger = Mock()
        mock_logger.info = Mock()
        mock_logger.debug = Mock()
        mock_logger.warn = Mock()

        mock_clock = Mock()
        mock_clock.now = Mock(return_value=Time(seconds=100))

        with patch.object(rclpy.node.Node, '__init__', return_value=None), \
             patch.object(rclpy.node.Node, 'declare_parameter'), \
             patch.object(rclpy.node.Node, 'get_parameter', side_effect=_mock_get_parameter), \
             patch.object(rclpy.node.Node, 'create_subscription'), \
             patch.object(rclpy.node.Node, 'create_publisher'), \
             patch.object(rclpy.node.Node, 'create_timer'), \
             patch.object(rclpy.node.Node, 'get_logger', return_value=mock_logger), \
             patch.object(rclpy.node.Node, 'get_clock', return_value=mock_clock):
            # Patch tf2_ros.TransformListener so it doesn't create real subscriptions
            with patch(
                'sih26126_sensor_contract.lidar_processor.tf2_ros.TransformListener'
            ):
                from sih26126_sensor_contract.lidar_processor import LidarProcessor
                self.node = LidarProcessor()

        # Override get_logger and get_clock as instance attributes so they
        # shadow the class-level property descriptors.
        object.__setattr__(self.node, 'get_logger', Mock(return_value=mock_logger))
        object.__setattr__(self.node, 'get_clock', Mock(return_value=mock_clock))

        # Mock the TF buffer — this is the key control point
        self.node._tf_buffer = Mock(spec=tf2_ros.Buffer)

        # Mock publishers
        self.node._scan_pub = Mock()
        self.node._health_pub = Mock()

    def test_no_scan_published_when_tf_lookup_exception(self):
        """Scan must not be published when odom->base_link TF is unavailable."""
        self.node._tf_buffer.lookup_transform.side_effect = tf2_ros.LookupException()

        cloud = _make_pointcloud(10.0)
        self.node._cloud_callback(cloud)

        self.node._scan_pub.publish.assert_not_called()

    def test_no_scan_published_when_tf_extrapolation_exception(self):
        """Scan must not be published when TF is outside the buffer range."""
        self.node._tf_buffer.lookup_transform.side_effect = \
            tf2_ros.ExtrapolationException()

        cloud = _make_pointcloud(10.0)
        self.node._cloud_callback(cloud)

        self.node._scan_pub.publish.assert_not_called()

    def test_no_scan_published_when_tf_connectivity_exception(self):
        """Scan must not be published when the TF chain is broken."""
        self.node._tf_buffer.lookup_transform.side_effect = \
            tf2_ros.ConnectivityException()

        cloud = _make_pointcloud(10.0)
        self.node._cloud_callback(cloud)

        self.node._scan_pub.publish.assert_not_called()

    def test_scan_published_when_tf_available(self):
        """Scan must be published when odom->base_link TF exists at the timestamp."""
        self.node._tf_buffer.lookup_transform.return_value = Mock()

        cloud = _make_pointcloud(42.0)
        self.node._cloud_callback(cloud)

        self.node._scan_pub.publish.assert_called_once()

    def test_published_scan_preserves_original_timestamp(self):
        """Published scan must carry the original cloud timestamp."""
        self.node._tf_buffer.lookup_transform.return_value = Mock()

        original_time = 42.5
        cloud = _make_pointcloud(original_time)
        self.node._cloud_callback(cloud)

        published_scan = self.node._scan_pub.publish.call_args[0][0]
        scan_time = (
            published_scan.header.stamp.sec
            + published_scan.header.stamp.nanosec * 1e-9
        )
        self.assertAlmostEqual(scan_time, original_time, places=3)

    def test_published_scan_has_correct_frame_id(self):
        """Published scan must use the LiDAR frame ID."""
        self.node._tf_buffer.lookup_transform.return_value = Mock()

        cloud = _make_pointcloud(10.0)
        self.node._cloud_callback(cloud)

        published_scan = self.node._scan_pub.publish.call_args[0][0]
        self.assertEqual(
            published_scan.header.frame_id,
            'sih_ugv/base_link/lidar_3d')

    def test_published_scan_has_valid_geometry(self):
        """Published scan must have valid angle and range parameters."""
        self.node._tf_buffer.lookup_transform.return_value = Mock()

        cloud = _make_pointcloud(10.0)
        self.node._cloud_callback(cloud)

        published_scan = self.node._scan_pub.publish.call_args[0][0]
        self.assertEqual(published_scan.angle_min, -2.35619)
        self.assertEqual(published_scan.angle_max, 2.35619)
        self.assertEqual(published_scan.range_min, 0.2)
        self.assertEqual(published_scan.range_max, 20.0)
        expected_increment = (2.35619 - (-2.35619)) / (720 - 1)
        self.assertAlmostEqual(
            published_scan.angle_increment, expected_increment, places=5)
        self.assertEqual(len(published_scan.ranges), 720)

    def test_dropped_cloud_does_not_block_subsequent(self):
        """A cloud dropped due to TF unavailability must not block later clouds."""
        # First cloud: TF unavailable → dropped
        self.node._tf_buffer.lookup_transform.side_effect = \
            tf2_ros.ExtrapolationException()
        cloud1 = _make_pointcloud(10.0)
        self.node._cloud_callback(cloud1)
        self.node._scan_pub.publish.assert_not_called()

        # Second cloud: TF available → published
        self.node._tf_buffer.lookup_transform.side_effect = None
        self.node._tf_buffer.lookup_transform.return_value = Mock()
        cloud2 = _make_pointcloud(15.0)
        self.node._cloud_callback(cloud2)
        self.node._scan_pub.publish.assert_called_once()

    def test_tf_lookup_uses_cloud_timestamp_not_latest(self):
        """TF lookup must request the cloud's timestamp, not Time() (latest)."""
        self.node._tf_buffer.lookup_transform.return_value = Mock()

        cloud = _make_pointcloud(99.5)
        self.node._cloud_callback(cloud)

        # Verify lookup_transform was called
        self.node._tf_buffer.lookup_transform.assert_called()

        # Check the time argument (3rd positional arg) is the cloud's time
        call_args = self.node._tf_buffer.lookup_transform.call_args
        # lookup_transform('odom', 'base_link', cloud_time, timeout=...)
        # The time is the 3rd positional argument (index 2)
        time_arg = call_args[0][2] if len(call_args[0]) > 2 else \
            call_args[1].get('time')
        self.assertIsInstance(time_arg, Time)

        # Verify it's the cloud's timestamp, not Time(0) (latest)
        time_sec, time_ns = time_arg.seconds_nanoseconds()
        lookup_time = time_sec + time_ns * 1e-9
        self.assertAlmostEqual(lookup_time, 99.5, places=3)

    def test_sample_count_increments_on_every_cloud(self):
        """Sample count must increment for every cloud, even dropped ones."""
        self.node._tf_buffer.lookup_transform.side_effect = tf2_ros.LookupException()

        for i in range(5):
            cloud = _make_pointcloud(float(i))
            self.node._cloud_callback(cloud)

        self.assertEqual(self.node._sample_count, 5)

    def test_no_queue_accumulation(self):
        """No queue should accumulate — each cloud is independently processed or dropped."""
        self.node._tf_buffer.lookup_transform.side_effect = \
            tf2_ros.ExtrapolationException()

        for i in range(10):
            cloud = _make_pointcloud(float(i))
            self.node._cloud_callback(cloud)

        # No scans published
        self.node._scan_pub.publish.assert_not_called()
        # No queue attribute or empty queue
        self.assertFalse(
            hasattr(self.node, '_cloud_queue')
            and len(getattr(self.node, '_cloud_queue', [])) > 0,
            'No cloud queue should exist in the simplified implementation')

    def test_consecutive_clouds_with_available_tf_all_published(self):
        """Multiple consecutive clouds with available TF must all be published."""
        self.node._tf_buffer.lookup_transform.return_value = Mock()

        for i in range(5):
            cloud = _make_pointcloud(float(10 + i))
            self.node._cloud_callback(cloud)

        self.assertEqual(self.node._scan_pub.publish.call_count, 5)


if __name__ == '__main__':
    unittest.main()
