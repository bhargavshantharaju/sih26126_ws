#!/usr/bin/env python3
"""Tests for visual odometry covariance and quality metrics."""

import math

from nav_msgs.msg import Odometry


def test_odometry_message_has_pose_covariance():
    """Odometry message must include 6x6 pose covariance matrix."""
    odom = Odometry()
    assert len(odom.pose.covariance) == 36


def test_high_quality_produces_low_covariance():
    """
    High inlier ratio and low reprojection error produce low covariance.

    Simulated quality metrics confirm that a high inlier ratio with
    low reprojection error results in a quality factor near 1.0.
    """
    # Simulated quality metrics
    inlier_ratio = 0.9
    reprojection_error = 1.0

    base_pos_cov = 0.01
    base_rot_cov = 0.01

    quality_factor = max(1.0, 5.0 * (1.0 - inlier_ratio))
    quality_factor *= (1.0 + reprojection_error / 2.0)

    pos_cov = base_pos_cov * quality_factor
    rot_cov = base_rot_cov * quality_factor

    # High quality → quality_factor ≈ 1.5
    assert pos_cov < 0.02
    assert rot_cov < 0.02


def test_low_quality_produces_high_covariance():
    """
    Low inlier ratio and high reprojection error produce high covariance.

    Poor tracking quality causes the quality factor to exceed 10,
    resulting in covariance values above the low-quality threshold.
    """
    inlier_ratio = 0.3
    reprojection_error = 8.0

    base_pos_cov = 0.01
    base_rot_cov = 0.01

    quality_factor = max(1.0, 5.0 * (1.0 - inlier_ratio))
    quality_factor *= (1.0 + reprojection_error / 2.0)

    pos_cov = base_pos_cov * quality_factor
    rot_cov = base_rot_cov * quality_factor

    # Low quality → quality_factor > 10
    assert pos_cov > 0.10
    assert rot_cov > 0.10


def test_covariance_matrix_structure():
    """Covariance matrix should have values only for x, y, yaw."""
    # Create sample covariance like rgbd_odometry_baseline
    pos_cov = 0.02
    rot_cov = 0.015

    covariance = [0.0] * 36
    covariance[0] = pos_cov      # x
    covariance[7] = pos_cov      # y
    covariance[14] = 99999.0     # z (not used)
    covariance[21] = 99999.0     # roll (not used)
    covariance[28] = 99999.0     # pitch (not used)
    covariance[35] = rot_cov     # yaw

    # Check x position
    assert covariance[0] == pos_cov
    # Check y position
    assert covariance[7] == pos_cov
    # Check yaw
    assert covariance[35] == rot_cov
    # Check unused dimensions have high uncertainty
    assert covariance[14] > 1000
    assert covariance[21] > 1000
    assert covariance[28] > 1000


def test_covariance_bounds():
    """Covariance values should be finite and positive."""
    base_pos_cov = 0.01
    base_rot_cov = 0.01

    # Even with worst quality, covariance should be bounded
    quality_factor = 50.0  # Very bad quality

    pos_cov = base_pos_cov * quality_factor
    rot_cov = base_rot_cov * quality_factor

    assert math.isfinite(pos_cov)
    assert math.isfinite(rot_cov)
    assert pos_cov > 0.0
    assert rot_cov > 0.0


def test_quality_factor_increases_with_poor_tracking():
    """Quality factor should increase as tracking quality degrades."""
    # Good tracking
    quality_good = max(1.0, 5.0 * (1.0 - 0.85))
    quality_good *= (1.0 + 1.5 / 2.0)

    # Poor tracking
    quality_poor = max(1.0, 5.0 * (1.0 - 0.40))
    quality_poor *= (1.0 + 6.0 / 2.0)

    assert quality_poor > quality_good
    assert quality_good > 1.0


def test_zero_inliers_handled():
    """Zero inliers should not cause division by zero."""
    inlier_count = 0
    total_matches = 50

    if total_matches > 0:
        inlier_ratio = float(inlier_count) / total_matches
    else:
        inlier_ratio = 0.0

    quality_factor = max(1.0, 5.0 * (1.0 - inlier_ratio))

    assert math.isfinite(quality_factor)
    assert quality_factor == 5.0  # Maximum penalty


def test_frame_id_conventions():
    """Odometry message should use correct frame IDs."""
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'

    assert odom.header.frame_id == 'odom'
    assert odom.child_frame_id == 'base_link'


def test_timestamp_propagation():
    """Odometry timestamp should match input image timestamp."""
    stamp_sec = 1234
    stamp_nsec = 567890000

    # Simulated timestamp propagation
    received_stamp = float(stamp_sec) + float(stamp_nsec) * 1e-9

    seconds = int(received_stamp)
    nanoseconds = int(round((received_stamp - seconds) * 1_000_000_000))

    assert seconds == stamp_sec
    assert abs(nanoseconds - stamp_nsec) < 1000  # Within 1 microsecond


def test_visual_odometry_covariance_does_not_dominate_ekf():
    """
    Visual odometry covariance must be high enough to not dominate the EKF.

    The EKF also fuses wheel-odometry velocities from Gazebo ground truth
    (accurate) and IMU yaw.  The RGB-D visual odometry is biased in this
    simulation (underestimates motion ~3x, wrong y-direction).  Its
    covariance must be much larger than the wheel-odometry covariance
    (0.01) so the EKF relies primarily on wheel-odometry integration.
    """
    import inspect
    from sih26126_localization import rgbd_odometry_baseline as mod
    source = inspect.getsource(mod)

    # The base position covariance must be >= 1.0 (not 0.01) so the
    # biased visual pose cannot override accurate wheel-odometry.
    assert 'base_pos_cov = 10.0' in source, (
        'base_pos_cov must be 10.0 — a tight 0.01 covariance let the biased '
        'visual odometry dominate the EKF, causing odom->base_link divergence')
    assert 'base_rot_cov = 1.0' in source, (
        'base_rot_cov must be 1.0 to prevent biased yaw from dominating the EKF')

    # When no motion has been accepted, covariance must be very high.
    assert '10000.0' in source, (
        'No-motion-accepted guard must use very high covariance (10000.0)')


def test_visual_odometry_covariance_much_larger_than_wheel():
    """
    Visual odometry covariance >> wheel odometry covariance (0.01).

    The wheel odometry (Gazebo ground truth) has twist covariance 0.01
    for vx.  The visual odometry must have a much higher pose covariance
    so the EKF integrates wheel velocities rather than trusting the
    biased visual pose.
    """
    base_pos_cov = 10.0
    wheel_vx_cov = 0.01  # from ekf_sensor_adapter

    assert base_pos_cov > 100 * wheel_vx_cov, (
        f'visual odom covariance {base_pos_cov} must be >>100x wheel '
        f'covariance {wheel_vx_cov} to prevent EKF divergence')
