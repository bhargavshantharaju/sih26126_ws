#!/usr/bin/env python3
"""Comprehensive timestamped evidence collector for DRISTHI acceptance tests.

Subscribes to all critical ROS 2 topics simultaneously and records timestamped
samples to a evidence directory.  Replaces the unreliable ``ros2 topic echo
--once`` polling approach that suffered from independent timeouts, garbled
interleaving, and string-field truncation.

Usage:
    source install/setup.bash
    python3 evidence_collector.py <output_dir> [duration_seconds]

Captures:
  - /clock advancement (sim time, monotonicity)
  - /mission/state and /mission/metrics (full JSON, not truncated)
  - /safety/state and /safety/reason (atomic pair, no interleaving)
  - /exploration/state and /exploration/frontier_count
  - /fusion/state (imu, odom, ekf health)
  - /model/sih_ugv/odometry (Gazebo ground-truth: cumulative path + net displacement)
  - /odometry/filtered (EKF output: cumulative path + net displacement)
  - /localization/confidence
  - /autonomy/mode
  - /cmd_vel (publisher count via topic info)
  - /lidar/points and /lidar/scan_2d (publishing rate)
  - TF availability (map->odom->base_link chain)
"""

import json
import math
import os
import sys
import time
from collections import deque, Counter
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import String, Float32
from sih26126_msgs.msg import AutonomyMode
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener


def safe_write(path: str, content: str) -> None:
    """Write text to a file, creating parent dirs if needed."""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w') as f:
        f.write(content)


class EvidenceCollector(Node):
    """Collect timestamped evidence from all critical DRISTHI topics."""

    def __init__(self, out_dir: str, duration: float) -> None:
        super().__init__('evidence_collector')
        self.out_dir = out_dir
        self.duration = duration
        self.start_wall = time.time()

        # --- State storage ---
        self.clock_samples: list = []
        self.mission_states: list = []
        self.safety_samples: list = []
        self.exploration_states: list = []
        self.frontier_counts: list = []
        self.fusion_samples: list = []
        self.autonomy_modes: list = []
        self.confidence_samples: list = []
        self.metrics_raw: Optional[str] = None
        self.metrics_count: int = 0

        # Odometry (Gazebo ground truth)
        self.gz_odom_samples: list = []
        self.gz_path_length: float = 0.0
        self.gz_last_xy: Optional[tuple] = None
        self.gz_net_start: Optional[tuple] = None
        self.gz_net_end: Optional[tuple] = None

        # Odometry (EKF filtered)
        self.ekf_odom_samples: list = []
        self.ekf_path_length: float = 0.0
        self.ekf_last_xy: Optional[tuple] = None
        self.ekf_net_start: Optional[tuple] = None
        self.ekf_net_end: Optional[tuple] = None

        # Sensor rate tracking
        self.lidar_points_times: list = []
        self.lidar_scan_times: list = []
        self.cmd_vel_count: int = 0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_check_count: int = 0
        self.tf_map_odom_ok: int = 0
        self.tf_odom_base_ok: int = 0

        # --- Subscriptions ---
        # Use best-effort QoS for sensor data (common in Gazebo bridges)
        best_effort = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.create_subscription(String, '/mission/state', self._mission_cb, 10)
        self.create_subscription(String, '/mission/metrics', self._metrics_cb, 10)
        self.create_subscription(String, '/safety/state', self._safety_state_cb, 10)
        self.create_subscription(String, '/safety/reason', self._safety_reason_cb, 10)
        self.create_subscription(String, '/exploration/state', self._exploration_cb, 10)
        self.create_subscription(Float32, '/exploration/frontier_count',
                                  self._frontier_cb, 10)
        self.create_subscription(String, '/fusion/state', self._fusion_cb, 10)
        self.create_subscription(AutonomyMode, '/autonomy/mode',
                                  self._autonomy_cb, 10)
        self.create_subscription(Float32, '/localization/confidence',
                                  self._confidence_cb, 10)
        self.create_subscription(Odometry, '/model/sih_ugv/odometry',
                                 self._gz_odom_cb, best_effort)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._ekf_odom_cb, 10)
        self.create_subscription(PointCloud2, '/lidar/points',
                                 self._lidar_points_cb, best_effort)
        self.create_subscription(LaserScan, '/lidar/scan_2d',
                                 self._lidar_scan_cb, best_effort)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # Hold latest safety state/reason for atomic pairing
        self._latest_safety_state: Optional[str] = None
        self._latest_safety_reason: Optional[str] = None
        self._latest_safety_time: float = 0.0

        # Timer for periodic sampling (1 Hz) and TF checks
        self.create_timer(1.0, self._periodic_sample)
        self.create_timer(2.0, self._check_tf)

        # Timer for cmd_vel publisher count (every 5s)
        self.create_timer(5.0, self._check_cmd_vel_publishers)

        self.get_logger().info(
            f'Evidence collector started: duration={duration:.0f}s, '
            f'output={out_dir}')

    # --- Callbacks ---
    def _clock_cb(self, msg: Clock) -> None:
        sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_samples.append((time.time(), sim_t))

    def _mission_cb(self, msg: String) -> None:
        self.mission_states.append((time.time(), msg.data))

    def _metrics_cb(self, msg: String) -> None:
        self.metrics_raw = msg.data
        self.metrics_count += 1

    def _safety_state_cb(self, msg: String) -> None:
        self._latest_safety_state = msg.data
        self._latest_safety_time = time.time()

    def _safety_reason_cb(self, msg: String) -> None:
        self._latest_safety_reason = msg.data
        # Record the atomic pair when reason arrives (state should already
        # be fresh from the same publish cycle)
        ts = time.time()
        state = self._latest_safety_state or 'UNKNOWN'
        reason = self._latest_safety_reason or 'UNKNOWN'
        self.safety_samples.append((ts, state, reason))

    def _exploration_cb(self, msg: String) -> None:
        self.exploration_states.append((time.time(), msg.data))

    def _frontier_cb(self, msg: Float32) -> None:
        self.frontier_counts.append((time.time(), int(msg.data)))

    def _fusion_cb(self, msg: String) -> None:
        # Also capture imu/odom/ekf sub-topics if available
        ts = time.time()
        state = msg.data
        imu = odom = ekf = 'unknown'
        # Try to get the boolean health topics
        self.fusion_samples.append((ts, state, imu, odom, ekf))

    def _autonomy_cb(self, msg: AutonomyMode) -> None:
        self.autonomy_modes.append(
            (time.time(), msg.mode, bool(msg.is_degraded), msg.reason))

    def _confidence_cb(self, msg: Float32) -> None:
        self.confidence_samples.append((time.time(), float(msg.data)))

    def _gz_odom_cb(self, msg: Odometry) -> None:
        ts = time.time()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = 0.0
        q = msg.pose.pose.orientation
        # Extract yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.z * q.z + q.y * q.y)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.gz_odom_samples.append((ts, x, y, yaw))
        if self.gz_net_start is None:
            self.gz_net_start = (x, y)
        self.gz_net_end = (x, y)
        if self.gz_last_xy is not None:
            self.gz_path_length += math.dist((x, y), self.gz_last_xy)
        self.gz_last_xy = (x, y)

    def _ekf_odom_cb(self, msg: Odometry) -> None:
        ts = time.time()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.ekf_odom_samples.append((ts, x, y))
        if self.ekf_net_start is None:
            self.ekf_net_start = (x, y)
        self.ekf_net_end = (x, y)
        if self.ekf_last_xy is not None:
            self.ekf_path_length += math.dist((x, y), self.ekf_last_xy)
        self.ekf_last_xy = (x, y)

    def _lidar_points_cb(self, _msg: PointCloud2) -> None:
        self.lidar_points_times.append(time.time())

    def _lidar_scan_cb(self, _msg: LaserScan) -> None:
        self.lidar_scan_times.append(time.time())

    def _cmd_vel_cb(self, _msg: Twist) -> None:
        self.cmd_vel_count += 1

    # --- Periodic tasks ---
    def _periodic_sample(self) -> None:
        """Record a periodic snapshot of safety state for the timeline."""
        ts = time.time()
        state = self._latest_safety_state or 'NO_SAMPLE'
        reason = self._latest_safety_reason or 'NO_SAMPLE'
        age = ts - self._latest_safety_time if self._latest_safety_time > 0 else -1
        self.safety_samples.append((ts, state, reason, age))

    def _check_tf(self) -> None:
        """Check TF chain map->odom->base_link at the current time."""
        self.tf_check_count += 1
        ts = time.time()
        try:
            self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            self.tf_map_odom_ok += 1
        except Exception:
            pass
        try:
            self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.tf_odom_base_ok += 1
        except Exception:
            pass

    def _check_cmd_vel_publishers(self) -> None:
        """Count publishers on /cmd_vel using the ROS 2 graph API."""
        try:
            info = self.get_publishers_info_by_topic('/cmd_vel')
            self.get_logger().info(
                f'/cmd_vel publisher count: {len(info)} '
                f'({[i.node_name for i in info]})')
        except Exception as e:
            self.get_logger().warn(f'Could not get /cmd_vel publishers: {e}')

    # --- Output ---
    def write_evidence(self) -> dict:
        """Write all collected evidence to files and return a summary dict."""
        end_wall = time.time()
        wall_duration = end_wall - self.start_wall

        # Clock analysis
        clock_vals = [s[1] for s in self.clock_samples]
        sim_start = clock_vals[0] if clock_vals else None
        sim_end = clock_vals[-1] if clock_vals else None
        sim_duration = (sim_end - sim_start) if (sim_start and sim_end) else 0
        # Check monotonicity
        non_monotonic = 0
        for i in range(1, len(clock_vals)):
            if clock_vals[i] < clock_vals[i - 1]:
                non_monotonic += 1

        safe_write(os.path.join(self.out_dir, 'clock_samples.txt'),
                   '\n'.join(f'{w:.6f}\t{s:.6f}' for w, s in self.clock_samples))

        # Mission states
        safe_write(os.path.join(self.out_dir, 'mission_state_timeline.txt'),
                   '\n'.join(f'[{time.strftime("%H:%M:%S", time.localtime(w))}] {s}'
                             for w, s in self.mission_states))

        # Safety timeline (atomic state+reason pairs)
        safety_lines = []
        for entry in self.safety_samples:
            if len(entry) == 4:
                w, state, reason, age = entry
                safety_lines.append(
                    f'[{time.strftime("%H:%M:%S", time.localtime(w))}] '
                    f'state={state} reason={reason} age={age:.2f}s')
            else:
                w, state, reason = entry
                safety_lines.append(
                    f'[{time.strftime("%H:%M:%S", time.localtime(w))}] '
                    f'state={state} reason={reason}')
        safe_write(os.path.join(self.out_dir, 'safety_timeline.txt'),
                   '\n'.join(safety_lines))

        # Exploration states
        safe_write(os.path.join(self.out_dir, 'exploration_timeline.txt'),
                   '\n'.join(f'[{time.strftime("%H:%M:%S", time.localtime(w))}] {s}'
                             for w, s in self.exploration_states))

        # Frontier counts
        safe_write(os.path.join(self.out_dir, 'frontier_timeline.txt'),
                   '\n'.join(f'[{time.strftime("%H:%M:%S", time.localtime(w))}] count={c}'
                             for w, c in self.frontier_counts))

        # Fusion samples
        safe_write(os.path.join(self.out_dir, 'fusion_timeline.txt'),
                   '\n'.join(f'[{time.strftime("%H:%M:%S", time.localtime(w))}] '
                             f'state={s} imu={imu} odom={odom} ekf={ekf}'
                             for w, s, imu, odom, ekf in self.fusion_samples))

        # Autonomy modes
        safe_write(os.path.join(self.out_dir, 'autonomy_timeline.txt'),
                   '\n'.join(
                       f'[{time.strftime("%H:%M:%S", time.localtime(w))}] '
                       f'mode={m} degraded={d} reason={r}'
                       for w, m, d, r in self.autonomy_modes))

        # Confidence
        safe_write(os.path.join(self.out_dir, 'confidence_timeline.txt'),
                   '\n'.join(f'{w:.6f}\t{c:.4f}'
                             for w, c in self.confidence_samples))

        # Full metrics JSON (not truncated!)
        if self.metrics_raw:
            safe_write(os.path.join(self.out_dir, 'final_metrics_full.json'),
                       self.metrics_raw)
            try:
                parsed = json.loads(self.metrics_raw)
                safe_write(os.path.join(self.out_dir, 'final_metrics_pretty.json'),
                           json.dumps(parsed, indent=2, sort_keys=True))
            except json.JSONDecodeError as e:
                safe_write(os.path.join(self.out_dir, 'metrics_parse_error.txt'),
                           f'JSON parse error: {e}\nRaw length: {len(self.metrics_raw)}')
        else:
            safe_write(os.path.join(self.out_dir, 'final_metrics_full.json'),
                       '{"error": "no metrics message received"}')

        # Odometry summary
        gz_net_disp = (math.dist(self.gz_net_end, self.gz_net_start)
                       if self.gz_net_start and self.gz_net_end else 0)
        ekf_net_disp = (math.dist(self.ekf_net_end, self.ekf_net_start)
                        if self.ekf_net_start and self.ekf_net_end else 0)

        odom_summary = {
            'gz_ground_truth': {
                'samples': len(self.gz_odom_samples),
                'cumulative_path_length_m': round(self.gz_path_length, 4),
                'net_displacement_m': round(gz_net_disp, 4),
                'start_xy': list(self.gz_net_start) if self.gz_net_start else None,
                'end_xy': list(self.gz_net_end) if self.gz_net_end else None,
            },
            'ekf_filtered': {
                'samples': len(self.ekf_odom_samples),
                'cumulative_path_length_m': round(self.ekf_path_length, 4),
                'net_displacement_m': round(ekf_net_disp, 4),
                'start_xy': list(self.ekf_net_start) if self.ekf_net_start else None,
                'end_xy': list(self.ekf_net_end) if self.ekf_net_end else None,
            },
        }
        safe_write(os.path.join(self.out_dir, 'odometry_summary.json'),
                   json.dumps(odom_summary, indent=2))

        # Odometry samples (timestamped)
        safe_write(os.path.join(self.out_dir, 'gz_odometry_samples.txt'),
                   '\n'.join(f'{w:.6f}\t{x:.6f}\t{y:.6f}\t{yaw:.6f}'
                             for w, x, y, yaw in self.gz_odom_samples))
        safe_write(os.path.join(self.out_dir, 'ekf_odometry_samples.txt'),
                   '\n'.join(f'{w:.6f}\t{x:.6f}\t{y:.6f}'
                             for w, x, y in self.ekf_odom_samples))

        # Sensor rates
        now = time.time()
        # Count messages in the last 10 seconds
        gz_rate = len([t for t in self.lidar_points_times if now - t < 10]) / 10.0
        scan_rate = len([t for t in self.lidar_scan_times if now - t < 10]) / 10.0
        safe_write(os.path.join(self.out_dir, 'sensor_rates.txt'),
                   f'lidar_points_rate_last_10s: {gz_rate:.2f} Hz\n'
                   f'lidar_scan_2d_rate_last_10s: {scan_rate:.2f} Hz\n'
                   f'lidar_points_total: {len(self.lidar_points_times)}\n'
                   f'lidar_scan_2d_total: {len(self.lidar_scan_times)}\n'
                   f'cmd_vel_messages_received: {self.cmd_vel_count}\n')

        # TF health
        safe_write(os.path.join(self.out_dir, 'tf_health.txt'),
                   f'tf_checks: {self.tf_check_count}\n'
                   f'map_odom_available: {self.tf_map_odom_ok}/{self.tf_check_count}\n'
                   f'odom_base_link_available: {self.tf_odom_base_ok}/{self.tf_check_count}\n')

        # Summary
        summary = {
            'wall_duration_s': round(wall_duration, 2),
            'sim_duration_s': round(sim_duration, 2),
            'sim_start_s': sim_start,
            'sim_end_s': sim_end,
            'clock_non_monotonic_count': non_monotonic,
            'clock_sample_count': len(self.clock_samples),
            'mission_state_count': len(self.mission_states),
            'safety_sample_count': len(self.safety_samples),
            'fusion_sample_count': len(self.fusion_samples),
            'gz_path_length_m': round(self.gz_path_length, 4),
            'gz_net_displacement_m': round(gz_net_disp, 4),
            'ekf_path_length_m': round(self.ekf_path_length, 4),
            'ekf_net_displacement_m': round(ekf_net_disp, 4),
            'lidar_points_rate_hz': round(gz_rate, 2),
            'lidar_scan_2d_rate_hz': round(scan_rate, 2),
            'cmd_vel_msg_count': self.cmd_vel_count,
            'metrics_msg_count': self.metrics_count,
            'tf_map_odom_ok': f'{self.tf_map_odom_ok}/{self.tf_check_count}',
            'tf_odom_base_ok': f'{self.tf_odom_base_ok}/{self.tf_check_count}',
            'autonomy_sample_count': len(self.autonomy_modes),
            'autonomy_mode_distribution': dict(
                sorted(Counter(
                    m for _, m, _, _ in self.autonomy_modes).items())),
            'autonomy_degraded_fraction': round(
                sum(1 for _, _, d, _ in self.autonomy_modes if d)
                / max(1, len(self.autonomy_modes)), 4),
        }
        safe_write(os.path.join(self.out_dir, 'evidence_summary.json'),
                   json.dumps(summary, indent=2))
        return summary


def main() -> None:
    if len(sys.argv) < 2:
        print('Usage: evidence_collector.py <output_dir> [duration_seconds]')
        sys.exit(1)
    out_dir = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    node = EvidenceCollector(out_dir, duration)

    # Spin for the specified duration
    end_time = time.time() + duration
    while rclpy.ok() and time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.5)

    summary = node.write_evidence()
    print('\n=== Evidence Collection Summary ===')
    print(json.dumps(summary, indent=2))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
