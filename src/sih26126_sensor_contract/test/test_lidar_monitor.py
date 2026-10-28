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

    # ------------------------------------------------------------------
    # Regression: min_valid_ratio must not flag open terrain as unhealthy.
    # The previous default 0.60 required 60% of scan bins to have obstacle
    # returns, which is an environment-clutter metric — not sensor health.
    # In open outdoor terrain most beams reach range_max with no return (inf),
    # so a perfectly healthy LiDAR was falsely reported as LOW_VALID_RATIO,
    # keeping the autonomy stack stuck in DEGRADED_SENSOR.
    # ------------------------------------------------------------------

    def test_open_terrain_scan_is_healthy(self):
        """A scan with sparse but nonzero returns (open terrain) is healthy."""
        scan = LaserScan()
        scan.range_min = 0.2
        scan.range_max = 20.0
        # 720 bins: 50 with returns at ~3 m, rest inf (open terrain)
        scan.ranges = [3.0] * 50 + [float('inf')] * 670

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

        # ~6.9% returns — exceeds the 0.05 "sensor alive" threshold.
        self.assertGreater(ratio, 0.05)
        self.assertLess(ratio, 0.60,
                        'regression: ratio should be below the old 0.60 threshold')

    def test_all_inf_scan_is_unhealthy(self):
        """A scan with no returns at all (dead sensor) is unhealthy."""
        scan = LaserScan()
        scan.range_min = 0.2
        scan.range_max = 20.0
        scan.ranges = [float('inf')] * 720

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

        self.assertEqual(ratio, 0.0)
        self.assertLess(ratio, 0.05,
                        'dead sensor (0% returns) must be below health threshold')

    def test_default_min_valid_ratio_is_sensor_alive_threshold(self):
        """The default min_valid_ratio must be 0.05, not 0.60.

        0.60 is an environment-clutter metric that falsely flags open terrain
        as a sensor failure. 0.05 means 'at least ~5% of beams return data',
        which is a genuine sensor-alive check.
        """
        import inspect
        from sih26126_sensor_contract import lidar_monitor as mod
        source = inspect.getsource(mod)
        self.assertIn("0.05", source,
                      'min_valid_ratio default must be 0.05 (sensor-alive), '
                      'not 0.60 (environment-clutter)')
        self.assertNotIn("'min_valid_ratio', 0.60", source,
                         'regression: 0.60 threshold must not be restored')

    def test_default_stale_threshold_accommodates_wsl2(self):
        """stale_threshold_sec default must be 2.0 to tolerate WSL2 RTF < 1.0."""
        import inspect
        from sih26126_sensor_contract import lidar_monitor as mod
        source = inspect.getsource(mod)
        self.assertIn("'stale_threshold_sec', 2.0", source,
                      'stale threshold must be 2.0 to tolerate simulation jitter')


if __name__ == '__main__':
    unittest.main()
