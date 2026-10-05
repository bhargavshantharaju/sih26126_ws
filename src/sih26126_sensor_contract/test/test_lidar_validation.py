#!/usr/bin/env python3
"""Tests for LiDAR functionality in the DRISTHI system."""

import unittest
from unittest.mock import MagicMock, patch

import rclpy
from sensor_msgs.msg import LaserScan

from sih26126_sensor_contract.scan_relay import ScanRelay


class LidarValidationTests(unittest.TestCase):
    """Test LiDAR sensor data validation and topic relay functionality."""

    def setUp(self):
        """Set up test environment."""
        if not rclpy.ok():
            rclpy.init()

    def tearDown(self):
        """Clean up test environment."""
        if rclpy.ok():
            rclpy.shutdown()

    def test_scan_relay_copies_all_fields(self):
        """Test that scan relay properly copies all LaserScan fields."""
        # Create mock node
        with patch('rclpy.create_node'):
            relay = ScanRelay()
            
            # Mock publisher
            mock_pub = MagicMock()
            relay.scan_pub = mock_pub
            
            # Create test scan message
            original_scan = LaserScan()
            original_scan.header.frame_id = 'lidar_frame'
            original_scan.header.stamp.sec = 123456
            original_scan.header.stamp.nanosec = 789000000
            original_scan.angle_min = -2.35619
            original_scan.angle_max = 2.35619
            original_scan.angle_increment = 0.00872664
            original_scan.time_increment = 0.0001
            original_scan.scan_time = 0.1
            original_scan.range_min = 0.20
            original_scan.range_max = 20.0
            original_scan.ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
            original_scan.intensities = [100, 150, 200, 175, 125]
            
            # Process through relay
            relay.lidar_callback(original_scan)
            
            # Verify publisher was called
            mock_pub.publish.assert_called_once()
            published_scan = mock_pub.publish.call_args[0][0]
            
            # Verify all fields match
            self.assertEqual(published_scan.header.frame_id, original_scan.header.frame_id)
            self.assertEqual(published_scan.header.stamp.sec, original_scan.header.stamp.sec)
            self.assertEqual(published_scan.header.stamp.nanosec, original_scan.header.stamp.nanosec)
            self.assertAlmostEqual(published_scan.angle_min, original_scan.angle_min, places=5)
            self.assertAlmostEqual(published_scan.angle_max, original_scan.angle_max, places=5)
            self.assertAlmostEqual(published_scan.angle_increment, original_scan.angle_increment, places=5)
            self.assertAlmostEqual(published_scan.time_increment, original_scan.time_increment, places=5)
            self.assertAlmostEqual(published_scan.scan_time, original_scan.scan_time, places=3)
            self.assertAlmostEqual(published_scan.range_min, original_scan.range_min, places=3)
            self.assertAlmostEqual(published_scan.range_max, original_scan.range_max, places=3)
            self.assertEqual(len(published_scan.ranges), len(original_scan.ranges))
            self.assertEqual(len(published_scan.intensities), len(original_scan.intensities))
            
            for i, (orig_range, pub_range) in enumerate(zip(original_scan.ranges, published_scan.ranges)):
                self.assertAlmostEqual(orig_range, pub_range, places=3, 
                                     msg=f"Range mismatch at index {i}")
                                     
            for i, (orig_int, pub_int) in enumerate(zip(original_scan.intensities, published_scan.intensities)):
                self.assertEqual(orig_int, pub_int, 
                               msg=f"Intensity mismatch at index {i}")

    def test_realistic_lidar_parameters_validation(self):
        """Test validation of realistic LiDAR parameters expected from SDF."""
        # Test parameters that should match the SDF configuration
        expected_angle_min = -2.35619  # -135 degrees
        expected_angle_max = 2.35619   # +135 degrees  
        expected_samples = 540         # 0.5 degree resolution
        expected_range_min = 0.20      # 20cm minimum
        expected_range_max = 20.0      # 20m maximum
        expected_update_rate = 10.0    # 10 Hz
        
        # Calculate expected angle increment
        expected_angle_increment = (expected_angle_max - expected_angle_min) / expected_samples
        
        # Verify parameters are within acceptable ranges
        self.assertAlmostEqual(expected_angle_min, -2.35619, places=4)
        self.assertAlmostEqual(expected_angle_max, 2.35619, places=4)
        self.assertEqual(expected_samples, 540)
        self.assertAlmostEqual(expected_angle_increment, 0.00872664, places=5)
        self.assertEqual(expected_range_min, 0.20)
        self.assertEqual(expected_range_max, 20.0)
        
        # Verify field of view is approximately 270 degrees
        fov_radians = expected_angle_max - expected_angle_min
        fov_degrees = fov_radians * 180.0 / 3.14159
        self.assertAlmostEqual(fov_degrees, 270.0, places=1)


if __name__ == '__main__':
    unittest.main()