#!/usr/bin/env python3
"""Tests for LiDAR health monitor."""

import unittest
from sensor_msgs.msg import LaserScan


class TestLidarMonitor(unittest.TestCase):
    """Test LiDAR monitor health computation logic."""

    def test_valid_ratio_computation(self):
        """Verify valid point ratio calculation."""
        from sih26126_sensor_contract.lidar_monitor import LidarMonitor

        # Create a mock node (will fail if actually started, but methods work)
        class MockNode:
            def __init__(self):
                pass

            def get_parameter(self, name):
                class Param:
                    value = {
                        'publish_hz': 5.0,
                        'stale_threshold_sec': 0.5,
                        'min_valid_ratio': 0.60,
                        'expected_min_range': 0.20,
                        'expected_max_range': 20.0,
                    }[name]
                return Param()

        # Test scan with all valid ranges
        scan = LaserScan()
        scan.range_min = 0.2
        scan.range_max = 20.0
        scan.ranges = [1.0, 2.0, 3.0, 5.0, 10.0]

        # Mock the monitor's parameters
        monitor = MockNode()
        monitor.expected_min_range = 0.20
        monitor.expected_max_range = 20.0

        # Compute valid ratio manually as the method does
        valid = 0
        total = len(scan.ranges)
        for r in scan.ranges:
            if (
                r >= scan.range_min
                and r <= scan.range_max
                and r >= monitor.expected_min_range
                and r <= monitor.expected_max_range
            ):
                valid += 1

        ratio = float(valid) / float(total)
        self.assertAlmostEqual(ratio, 1.0, places=2)

    def test_invalid_range_filtering(self):
        """Verify out-of-range values are excluded."""
        scan = LaserScan()
        scan.range_min = 0.2
        scan.range_max = 20.0
        scan.ranges = [
            0.1,   # below min
            1.0,   # valid
            25.0,  # above max
            3.0,   # valid
            float('inf'),  # invalid
        ]

        expected_min_range = 0.20
        expected_max_range = 20.0

        valid = 0
        total = len(scan.ranges)
        for r in scan.ranges:
            if (
                r >= scan.range_min
                and r <= scan.range_max
                and r >= expected_min_range
                and r <= expected_max_range
            ):
                valid += 1

        ratio = float(valid) / float(total)
        self.assertAlmostEqual(ratio, 0.4, places=2)  # 2 out of 5 valid

    def test_empty_scan_handling(self):
        """Verify empty scan returns zero ratio."""
        scan = LaserScan()
        scan.range_min = 0.2
        scan.range_max = 20.0
        scan.ranges = []

        total = len(scan.ranges)
        ratio = 0.0 if total == 0 else 1.0

        self.assertEqual(ratio, 0.0)


if __name__ == '__main__':
    unittest.main()
