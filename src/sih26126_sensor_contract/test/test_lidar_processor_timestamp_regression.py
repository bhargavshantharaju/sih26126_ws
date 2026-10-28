#!/usr/bin/env python3
"""Regression tests for LiDAR processor scan timestamp preservation.

Root cause addressed: lidar_processor previously stamped the published
LaserScan with self.get_clock().now().to_msg() (the node's wall/sim clock)
instead of msg.header.stamp (the original cloud acquisition time).  When
clouds arrived before the EKF started publishing odom->base_link TF, the
restamped scan carried a "current" timestamp but the underlying measurement
was stale.  SLAM's MessageFilter accepted the scan but could not look up
odom->base_link at the restamped timestamp because the EKF's TF buffer
did not cover that time yet, resulting in "earlier than transform cache"
errors and repeated MessageFilter drops.

Fix: preserve the original cloud timestamp (scan.header.stamp = msg.header.stamp)
and only publish scans whose odom->base_link TF exists at that timestamp.
Clouds captured before the EKF starts are dropped — restamping them would
break temporal consistency between the scan data and the robot's pose.

These tests verify:
1. The scan timestamp is the original cloud timestamp (not restamped).
2. The source code does NOT restamp with get_clock().now().
"""

import math
import sys
import time

import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Pure-function tests: project_to_scan_2d (no ROS needed)
# ---------------------------------------------------------------------------

try:
    from sih26126_sensor_contract.lidar_processor import (
        project_to_scan_2d,
        pointcloud2_to_xyz,
    )
    _IMPORT_OK = True
except ImportError:
    _IMPORT_OK = False


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_sensor_contract not importable')
def test_project_empty_pointcloud():
    """An empty point cloud produces an all-infinity scan."""
    pts = np.empty((0, 3), dtype=np.float32)
    result = project_to_scan_2d(pts, -math.pi, math.pi, 360, 0.2, 20.0)
    assert result.shape == (360,)
    assert np.all(np.isinf(result))


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_sensor_contract not importable')
def test_project_single_point_front():
    """A single point at 1.0 m directly in front populates the forward bin."""
    pts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    angle_min, angle_max, n_bins = -math.pi, math.pi, 360
    result = project_to_scan_2d(pts, angle_min, angle_max, n_bins, 0.2, 20.0)
    min_idx = int(np.nanargmin(np.where(np.isinf(result), np.inf, result)))
    assert 0.9 <= result[min_idx] <= 1.1


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_sensor_contract not importable')
def test_project_point_outside_range_filtered():
    """Points outside range_max are excluded."""
    pts = np.array([[25.0, 0.0, 0.0]], dtype=np.float32)
    result = project_to_scan_2d(pts, -math.pi, math.pi, 360, 0.2, 20.0)
    assert np.all(np.isinf(result))


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_sensor_contract not importable')
def test_project_point_below_min_range_filtered():
    """Points closer than range_min are excluded."""
    pts = np.array([[0.1, 0.0, 0.0]], dtype=np.float32)
    result = project_to_scan_2d(pts, -math.pi, math.pi, 360, 0.2, 20.0)
    assert np.all(np.isinf(result))


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_sensor_contract not importable')
def test_project_high_elevation_point_filtered():
    """Points with steep elevation (above 5-deg tolerance) are excluded."""
    pts = np.array([[1.0, 0.0, 2.0]], dtype=np.float32)
    result = project_to_scan_2d(pts, -math.pi, math.pi, 360, 0.2, 20.0,
                                vertical_tolerance_rad=math.radians(5.0))
    assert np.all(np.isinf(result))


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_sensor_contract not importable')
def test_project_low_elevation_point_accepted():
    """Points within the elevation tolerance are kept."""
    pts = np.array([[2.0, 0.0, 0.05]], dtype=np.float32)
    result = project_to_scan_2d(pts, -math.pi, math.pi, 360, 0.2, 20.0,
                               vertical_tolerance_rad=math.radians(5.0))
    assert not np.all(np.isinf(result)), 'Expected at least one populated bin'


# ---------------------------------------------------------------------------
# Structural regression test: scan stamp must preserve original cloud stamp
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_sensor_contract not importable')
def test_scan_preserves_original_timestamp():
    """LidarProcessor must stamp scans with the original cloud timestamp.

    Regression: the old bug used scan.header.stamp = self.get_clock().now().
    to_msg(), restamping stale clouds with a current timestamp to force
    SLAM's MessageFilter to accept them.  This broke temporal consistency
    and caused "earlier than transform cache" errors.

    The correct fix preserves scan.header.stamp = msg.header.stamp and only
    publishes scans whose odom->base_link TF exists at that timestamp.

    This structural test verifies the source code preserves the original
    timestamp and does NOT restamp with get_clock().now().
    """
    import ast
    import pathlib

    src_path = pathlib.Path(__file__).parents[1] / (
        'sih26126_sensor_contract/lidar_processor.py'
    )
    assert src_path.exists(), f'Source file not found: {src_path}'
    source = src_path.read_text(encoding='utf-8')
    tree = ast.parse(source)

    # The buggy pattern: scan.header.stamp = self.get_clock().now().to_msg()
    # Look for Assign nodes where target is scan.header.stamp
    # and value involves self.get_clock()
    buggy_pattern_found = False
    correct_pattern_found = False

    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        # Check target is scan.header.stamp
        target_is_scan_stamp = (
            len(node.targets) == 1
            and isinstance(node.targets[0], ast.Attribute)
            and node.targets[0].attr == 'stamp'
            and isinstance(node.targets[0].value, ast.Attribute)
            and node.targets[0].value.attr == 'header'
        )
        if not target_is_scan_stamp:
            continue

        # Check value: is it msg.header.stamp? (the CORRECT pattern)
        if (
            isinstance(node.value, ast.Attribute)
            and node.value.attr == 'stamp'
            and isinstance(node.value.value, ast.Attribute)
            and node.value.value.attr == 'header'
            and isinstance(node.value.value.value, ast.Name)
            and node.value.value.value.id == 'msg'
        ):
            correct_pattern_found = True

        # Check value: does it contain self.get_clock()? (the BUG)
        source_segment = ast.get_source_segment(source, node.value)
        if source_segment and 'get_clock' in source_segment:
            buggy_pattern_found = True

    assert correct_pattern_found, (
        'lidar_processor.py does not preserve the original cloud timestamp '
        '(scan.header.stamp = msg.header.stamp).  The fix may not be present.'
    )
    assert not buggy_pattern_found, (
        'REGRESSION: lidar_processor.py restamps scans with get_clock().now() '
        '— this breaks temporal consistency and causes SLAM MessageFilter '
        '"earlier than transform cache" errors.  Scans must preserve the '
        'original cloud timestamp.'
    )
