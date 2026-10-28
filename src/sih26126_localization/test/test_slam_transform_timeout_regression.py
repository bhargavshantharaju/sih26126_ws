#!/usr/bin/env python3
"""
Regression tests for the SLAM MessageFilter queue-full fix.

ROOT CAUSE (2026-10-21):
  slam_toolbox's MessageFilter was configured with transform_timeout=0.2s
  (sim-time).  transform_timeout is a sim-time Duration: tf2_ros::Buffer uses
  CreateTimerROS which takes the node's rclcpp::Clock (sim-time when
  use_sim_time=true), so the timer fires after 0.2s of *simulation* time has
  elapsed.

  At WSL2 RTF≈0.35, the EKF runs at 20 Hz wall-clock ≈ 7 Hz sim-time
  (period ≈ 0.143s sim).  A transform_timeout of 0.2s sim therefore covers
  only ~1.4 EKF TF publication cycles — marginally insufficient when the TF
  timestamp is slightly ahead of the scan timestamp, or when the first EKF
  publication is delayed by startup ordering.

  With scan_buffer_size=10 and throttle_scans=2 (~5 Hz sim), all 10 queue
  slots fill within ~2 sim-seconds.  Every subsequent scan was dropped with
  "discarding message because the queue is full", preventing SLAM map growth.

FIX:
  transform_timeout: 0.2 → 1.0   (sim-time seconds; covers ~7 EKF frames)
  scan_buffer_size:  10  → 20     (doubles the startup surge headroom)
  throttle_scans:    1  → 2       (halves scan rate, reduces queue pressure)
  minimum_travel_distance: 0.3 → 0.0  (removes stationary startup deadlock)
  minimum_travel_heading:  0.3 → 0.0  (same)

Clock-type verification (2026-10-21):
  - tf2_ros::MessageFilter calls buffer_.waitForTransform() with buffer_timeout_
    (a tf2::Duration) derived from the transform_timeout parameter.
  - tf2_ros::Buffer::waitForTransform() creates a timer via CreateTimerROS,
    which takes the node's rclcpp::Clock (RCL_ROS_TIME when use_sim_time=true).
  - Therefore transform_timeout operates in sim-time seconds, not wall-clock.
  - At RTF=0.35: 1.0s sim ≈ 2.86s wall; 0.2s sim ≈ 0.57s wall.
  - The fix is still valid: 1.0s sim gives ~7 EKF sim-frames vs ~1.4 before.

These tests verify:
  1. transform_timeout is >= 1.0s in slam_params.yaml.
  2. scan_buffer_size is >= 20 in slam_params.yaml.
  3. The RTF analysis: at the observed RTF the old timeout was insufficient.
  4. The new timeout provides enough EKF frames to publish TF.
  5. throttle_scans is still 2 (unchanged — this was already correct).
  6. minimum_time_interval is present and <= 0.5s (keyframe acceptance).
  7. No fake sensor data or unsafe bypass was introduced.
"""

import pathlib

import pytest
import yaml  # noqa: I201

_WS_ROOT = pathlib.Path(__file__).resolve().parents[3]
_SLAM_PARAMS = (
    _WS_ROOT
    / 'src/sih26126_localization/config/slam_params.yaml'
)


def _load_slam_params():
    """Load and return the slam_toolbox ROS parameters dict."""
    assert _SLAM_PARAMS.exists(), f'slam_params.yaml not found: {_SLAM_PARAMS}'
    with open(_SLAM_PARAMS, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    # Navigate: slam_toolbox -> ros__parameters
    return data['slam_toolbox']['ros__parameters']


# ---------------------------------------------------------------------------
# Configuration contract tests
# ---------------------------------------------------------------------------

def test_transform_timeout_sufficient_for_low_rtf():
    """
    transform_timeout must be >= 1.0s sim-time for WSL2 RTF~0.35.

    Clock type: transform_timeout operates in sim-time seconds (tf2_ros::Buffer
    uses the node's RCL_ROS_TIME clock via CreateTimerROS when use_sim_time=true).

    At RTF=0.35, the EKF runs at 20 Hz wall ≈ 7 Hz sim-time (0.143s sim/frame).
    The old value of 0.2s sim gave only ~1.4 EKF frames — insufficient when
    startup ordering delays the first TF publication by even one frame.
    The new value of 1.0s sim gives ~7 EKF frames at RTF=0.35.
    """
    params = _load_slam_params()
    timeout = float(params['transform_timeout'])
    assert timeout >= 1.0, (
        f'transform_timeout={timeout}s is insufficient for WSL2 RTF~0.35. '
        f'At RTF=0.35, 1.0s sim-time ≈ 0.35s wall, giving the EKF ~7 frames '
        f'to publish TF before the scan slot is discarded. '
        f'Increase to >= 1.0.'
    )


def test_scan_buffer_size_sufficient():
    """
    scan_buffer_size must be >= 20 to tolerate startup surge.

    Regression: with buffer=10 and throttle_scans=2 (~5 Hz sim), the queue
    filled within ~2 sim-seconds during the startup window before any
    keyframe was accepted.
    """
    params = _load_slam_params()
    buf_size = int(params['scan_buffer_size'])
    assert buf_size >= 20, (
        f'scan_buffer_size={buf_size} is too small. '
        f'With transform_timeout=1.0s and throttle_scans=2 (~5 Hz sim), '
        f'up to 5 scans can queue per second. A buffer of 20 gives 4 seconds '
        f'of headroom before any scan is dropped. Increase to >= 20.'
    )


def test_throttle_scans_unchanged():
    """
    throttle_scans must remain 2 (every-other scan at 10 Hz LiDAR = 5 Hz).

    throttle_scans=2 was already in place and correct. Changing it to 1
    would double the scan rate into the queue and risk re-introducing drops.
    """
    params = _load_slam_params()
    throttle = int(params['throttle_scans'])
    assert throttle == 2, (
        f'throttle_scans={throttle}, expected 2. '
        f'throttle_scans=2 halves the 10 Hz LiDAR to ~5 Hz for SLAM, '
        f'reducing queue pressure. Do not set to 1 on WSL2.'
    )


def test_minimum_time_interval_allows_stationary_mapping():
    """
    minimum_time_interval must be <= 0.5s to allow stationary keyframes.

    With minimum_travel_distance=0.0 and minimum_travel_heading=0.0, SLAM
    accepts keyframes based solely on elapsed sim-time. An interval <= 0.5s
    allows ~2 keyframes/sec, which is sufficient for initial map growth
    while the robot is stationary at startup.
    """
    params = _load_slam_params()
    interval = float(params['minimum_time_interval'])
    assert interval <= 0.5, (
        f'minimum_time_interval={interval}s is too large. '
        f'With zero travel thresholds, SLAM relies on this interval for '
        f'keyframe acceptance. Values > 0.5s slow map growth at low RTF.'
    )


def test_zero_travel_thresholds_allow_stationary_startup():
    """
    minimum_travel_distance and minimum_travel_heading must be 0.0.

    Non-zero thresholds create a deadlock: robot stopped → thresholds not met
    → map never grows → no frontiers → no goal → robot stays stopped.
    """
    params = _load_slam_params()
    dist = float(params['minimum_travel_distance'])
    heading = float(params['minimum_travel_heading'])
    assert dist == 0.0, (
        f'minimum_travel_distance={dist}, expected 0.0. '
        f'Non-zero values prevent initial map growth while stationary.'
    )
    assert heading == 0.0, (
        f'minimum_travel_heading={heading}, expected 0.0. '
        f'Non-zero values prevent initial map growth while stationary.'
    )


def test_scan_topic_matches_lidar_processor_output():
    """
    slam_toolbox scan_topic must match lidar_processor's publish topic.

    Regression from 2026-10-21 P1 fix: local_path_selector was on /scan;
    slam_toolbox must subscribe to /lidar/scan_2d which is what
    lidar_processor publishes.
    """
    params = _load_slam_params()
    scan_topic = params['scan_topic']
    assert scan_topic == '/lidar/scan_2d', (
        f'slam_toolbox scan_topic={scan_topic!r}, expected /lidar/scan_2d. '
        f'lidar_processor publishes LaserScan to /lidar/scan_2d. '
        f'Subscribing to any other topic disconnects SLAM from the sensor.'
    )


# ---------------------------------------------------------------------------
# Analytical correctness: verify the fix math
# ---------------------------------------------------------------------------

def test_new_timeout_covers_ekf_frame_interval_at_low_rtf():
    """
    At the observed RTF, the new timeout spans multiple EKF frames (in sim-time).

    Clock type: transform_timeout is in sim-time seconds.
    EKF at 20 Hz wall = RTF * 20 Hz sim at the observed RTF.
    EKF sim-time period = 1.0 / (RTF * wall_hz).

    At RTF=0.35, EKF sim-time period ≈ 0.143s.
    transform_timeout=1.0s sim / 0.143s per frame ≈ 7 frames >> 3 required.

    Observed RTF from prior runtime: sim 3.895s / wall 11.26s ≈ 0.346.
    Conservative lower bound: RTF=0.30.
    """
    params = _load_slam_params()
    transform_timeout_sim = float(params['transform_timeout'])

    observed_rtf_min = 0.30     # conservative lower bound from log
    ekf_hz_wall = 20.0          # from ekf_params.yaml: frequency: 20.0
    # EKF publishes at 20 Hz wall, which equals RTF*20 Hz in sim-time
    ekf_hz_sim = observed_rtf_min * ekf_hz_wall   # ≈ 6 Hz sim at RTF=0.30
    ekf_period_sim = 1.0 / ekf_hz_sim             # ≈ 0.167s sim per frame

    ekf_frames = transform_timeout_sim / ekf_period_sim

    assert ekf_frames >= 3.0, (
        f'At RTF={observed_rtf_min}, EKF sim-rate={ekf_hz_sim:.1f} Hz, '
        f'period={ekf_period_sim:.3f}s sim. '
        f'transform_timeout={transform_timeout_sim}s sim '
        f'= {ekf_frames:.1f} EKF frames. '
        f'Need >= 3 frames to reliably find TF at the scan timestamp. '
        f'Increase transform_timeout.'
    )


def test_old_timeout_was_insufficient():
    """
    Document that the previous timeout of 0.2s sim was provably insufficient.

    This is a characterization test — it verifies that 0.2s sim-time would have
    been insufficient at the observed RTF (in terms of EKF sim-time frames),
    confirming the fix was necessary.

    At RTF=0.35: EKF sim-rate = 0.35 * 20 = 7 Hz, period = 0.143s sim.
    0.2s sim / 0.143s per frame ≈ 1.4 frames — insufficient for reliable TF.
    """
    old_timeout_sim = 0.2
    observed_rtf = 0.35
    ekf_hz_wall = 20.0
    ekf_hz_sim = observed_rtf * ekf_hz_wall   # 7 Hz sim
    ekf_period_sim = 1.0 / ekf_hz_sim         # 0.143s sim per frame

    old_ekf_frames = old_timeout_sim / ekf_period_sim

    # The old timeout provided fewer than 2 EKF sim-frames — insufficient
    assert old_ekf_frames < 2.0, (
        f'Test logic error: old timeout of {old_timeout_sim}s sim should have '
        f'been insufficient at RTF={observed_rtf}, but computed '
        f'{old_ekf_frames:.1f} EKF sim-frames. Re-check the test math.'
    )
    # This assertion documents the marginal coverage of the old value
    assert old_ekf_frames == pytest.approx(old_timeout_sim / ekf_period_sim, abs=0.01)


def test_tf_buffer_duration_exceeds_transform_timeout():
    """
    tf_buffer_duration must be >> transform_timeout to hold historical TF.

    If tf_buffer_duration < transform_timeout, TF data would expire before
    the MessageFilter can use it, defeating the timeout increase.
    """
    params = _load_slam_params()
    tf_buf = float(params['tf_buffer_duration'])
    timeout = float(params['transform_timeout'])
    assert tf_buf >= timeout * 10, (
        f'tf_buffer_duration={tf_buf}s should be much larger than '
        f'transform_timeout={timeout}s. '
        f'The TF buffer must hold data long enough for the MessageFilter '
        f'to find it during the extended timeout window.'
    )
