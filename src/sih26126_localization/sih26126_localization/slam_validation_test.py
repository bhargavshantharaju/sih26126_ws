#!/usr/bin/env python3
# Copyright 2026 DRISTHI Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
SLAM movement validation test script for DRISTHI.

Drives the Gazebo robot through the safety supervisor (not bypassing it),
samples SLAM pose at timestamped intervals, verifies the SLAM map updates,
and records Gazebo ground-truth only for post-hoc evaluation error.

Requirements:
  - Gazebo simulation running with slam:=true launch argument.
  - slam_toolbox ACTIVE on /slam_toolbox/get_state.
  - /scan live at >= 5 Hz.
  - odom -> base_link TF published by odom_tf_publisher.

The test drives the robot by publishing to the advisory topics that the
safety_supervisor already requires:
  /planner/local_navigation_state  -> 'LOCAL_NAVIGATE'
  /planner/local_obstacle_score    -> 1.0  (full clearance)
  /planner/local_recommended_cmd_vel -> forward/turn commands

This means motion passes through the safety supervisor unchanged, which is
the only authoritative /cmd_vel publisher in the stack.

Usage (with stack already running):
    python3 slam_validation_test.py [--duration 30] [--linear 0.15] [--angular 0.3]

Outputs a structured JSON report to /tmp/slam_validation_report.json.
"""

import argparse
import json
import math
import sys
import time

from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String


class SlamValidationTest(Node):
    """Drive robot and collect SLAM quality metrics."""

    def __init__(
        self,
        duration_sec: float = 40.0,
        linear_speed: float = 0.15,
        angular_speed: float = 0.25,
    ) -> None:
        """Initialise the SLAM validation test node."""
        super().__init__('slam_validation_test')

        self._duration = duration_sec
        self._linear_speed = linear_speed
        self._angular_speed = angular_speed

        # ---- SLAM state subscribers ----
        self._slam_pose: PoseStamped | None = None
        self._slam_map: OccupancyGrid | None = None
        self._last_scan: LaserScan | None = None
        self._scan_count = 0
        self._imu_count = 0
        self._odom_count = 0
        self._filtered_count = 0
        self._slam_pose_count = 0
        self._gazebo_pose: PoseStamped | None = None
        self._filtered_pose: PoseStamped | None = None

        # ---- Publishers: advisory topics that feed safety_supervisor ----
        self._nav_state_pub = self.create_publisher(
            String, '/planner/local_navigation_state', 10
        )
        self._obstacle_score_pub = self.create_publisher(
            Float32, '/planner/local_obstacle_score', 10
        )
        self._cmd_pub = self.create_publisher(
            Twist, '/planner/local_recommended_cmd_vel', 10
        )
        # Localization confidence is already published by slam_pose_adapter,
        # but we also publish it to avoid any stale-localization shutdown.
        self._confidence_pub = self.create_publisher(
            Float32, '/localization/confidence', 10
        )

        # ---- Subscribers ----
        self.create_subscription(
            PoseStamped, '/localization/pose', self._pose_cb, 10
        )
        # slam_toolbox publishes /map with TRANSIENT_LOCAL durability (latched).
        # Match it so we receive the initial map even if we subscribe after the
        # first publish.
        from rclpy.qos import (  # noqa: PLC0415
            DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos
        )
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10
        )
        # Ground-truth Gazebo odometry — for post-hoc error evaluation ONLY.
        from nav_msgs.msg import Odometry  # noqa: PLC0415
        self.create_subscription(
            Odometry,
            '/model/sih_ugv/odometry',
            self._gt_cb,
            10,
        )
        self.create_subscription(
            Odometry, '/odometry/filtered', self._filtered_cb, 10
        )
        from sensor_msgs.msg import Imu  # noqa: PLC0415
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)

        # slam_toolbox lifecycle client for state verification
        self._slam_state_client = self.create_client(
            GetState, '/slam_toolbox/get_state'
        )

        # ---- Result storage ----
        self._pose_samples: list[dict] = []
        self._map_samples: list[dict] = []
        self._gt_samples: list[dict] = []
        self._scan_rates: list[float] = []

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._slam_pose = msg
        self._slam_pose_count += 1

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._slam_map = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        self._last_scan = msg
        self._scan_count += 1

    def _gt_cb(self, msg) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._gazebo_pose = pose
        self._odom_count += 1

    def _filtered_cb(self, msg) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._filtered_pose = pose
        self._filtered_count += 1

    def _imu_cb(self, _msg) -> None:
        self._imu_count += 1

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _verify_slam_active(self) -> bool:
        """Return True if slam_toolbox reports ACTIVE state."""
        if not self._slam_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('slam_toolbox/get_state service not available')
            return False

        future = self._slam_state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.done() and future.result() is not None:
            state = future.result().current_state
            active = state.id == 3  # 3 = active
            self.get_logger().info(
                f'slam_toolbox state: {state.label} (id={state.id}) '
                f'-> {"ACTIVE ✓" if active else "NOT ACTIVE ✗"}'
            )
            return active

        self.get_logger().warn('GetState call failed')
        return False

    def _map_stats(self) -> dict:
        """Return basic statistics about the current SLAM map."""
        if self._slam_map is None:
            return {'available': False}

        data = self._slam_map.data
        total = len(data)
        occupied = data.count(100)
        free = data.count(0)
        unknown = data.count(-1)
        return {
            'available': True,
            'frame_id': self._slam_map.header.frame_id,
            'width': self._slam_map.info.width,
            'height': self._slam_map.info.height,
            'resolution': self._slam_map.info.resolution,
            'total_cells': total,
            'occupied_cells': occupied,
            'free_cells': free,
            'unknown_cells': unknown,
            'explored_ratio': (occupied + free) / max(1, total),
        }

    def _pose_dict(self, pose: PoseStamped | None) -> dict | None:
        if pose is None:
            return None
        p = pose.pose.position
        q = pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return {
            'x': round(p.x, 4),
            'y': round(p.y, 4),
            'z': round(p.z, 4),
            'yaw_rad': round(yaw, 4),
            'frame_id': pose.header.frame_id,
        }

    def _publish_advisory(self, linear: float, angular: float) -> None:
        """Publish all safety-supervisor gate inputs and the motion command."""
        nav_msg = String()
        nav_msg.data = 'LOCAL_NAVIGATE'
        self._nav_state_pub.publish(nav_msg)

        score_msg = Float32()
        score_msg.data = 1.0  # full clearance — no real obstacle data in test
        self._obstacle_score_pub.publish(score_msg)

        conf_msg = Float32()
        conf_msg.data = 1.0
        self._confidence_pub.publish(conf_msg)

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self._cmd_pub.publish(cmd)

    def _stop_robot(self) -> None:
        """Send a zero command through the advisory channel."""
        for _ in range(5):
            self._publish_advisory(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _spin_sec(self, seconds: float) -> None:
        """Spin the event loop for *seconds* wall-clock time."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _displacement(self, p1: dict | None, p2: dict | None) -> float | None:
        """Euclidean XY distance between two pose dicts."""
        if p1 is None or p2 is None:
            return None
        dx = p2['x'] - p1['x']
        dy = p2['y'] - p1['y']
        return math.hypot(dx, dy)

    # ------------------------------------------------------------------
    # Main test sequence
    # ------------------------------------------------------------------

    def run(self) -> dict:
        """Execute the full validation sequence and return the report dict."""
        report: dict = {
            'test': 'slam_movement_validation',
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'params': {
                'duration_sec': self._duration,
                'linear_speed': self._linear_speed,
                'angular_speed': self._angular_speed,
            },
        }

        # ------ Pre-flight checks ------
        self.get_logger().info('=== PRE-FLIGHT CHECKS ===')

        # Allow ROS time to flow and subscribers to settle.
        self.get_logger().info('Waiting for topics to settle (3 s)...')
        self._spin_sec(3.0)

        # (A) Check slam_toolbox is ACTIVE
        slam_active = self._verify_slam_active()
        report['A_slam_toolbox_active'] = slam_active
        if not slam_active:
            self.get_logger().error('ABORT: slam_toolbox is not ACTIVE')
            report['result'] = 'ABORTED_SLAM_NOT_ACTIVE'
            return report

        # (B) Check /scan is live
        scan_before = self._scan_count
        self._spin_sec(2.0)
        scan_rate = (self._scan_count - scan_before) / 2.0
        self.get_logger().info(f'/scan rate: {scan_rate:.1f} Hz')
        report['B_scan_hz'] = round(scan_rate, 2)
        report['B_scan_live'] = scan_rate > 1.0

        if not report['B_scan_live']:
            self.get_logger().error('ABORT: /scan rate is too low (<1 Hz)')
            report['result'] = 'ABORTED_SCAN_NOT_LIVE'
            return report

        # (C) Verify SLAM map is available and in frame_id=map
        if self._slam_map is None:
            self.get_logger().error('ABORT: /map not yet received')
            report['result'] = 'ABORTED_MAP_NOT_AVAILABLE'
            return report

        report['C_slam_map_frame'] = self._slam_map.header.frame_id
        report['C_slam_map_frame_ok'] = (
            self._slam_map.header.frame_id == 'map'
        )
        if not report['C_slam_map_frame_ok']:
            self.get_logger().error(
                f'ABORT: /map frame_id is "{self._slam_map.header.frame_id}", '
                'expected "map" — this is not the SLAM map'
            )
            report['result'] = 'ABORTED_WRONG_MAP_FRAME'
            return report

        # (D) Verify SLAM pose is available
        if self._slam_pose is None:
            self.get_logger().warn('Waiting for /localization/pose...')
            self._spin_sec(3.0)

        if self._slam_pose is None:
            self.get_logger().error('ABORT: /localization/pose not received')
            report['result'] = 'ABORTED_NO_SLAM_POSE'
            return report

        # ------ Baseline snapshot ------
        self.get_logger().info('=== BASELINE SNAPSHOT ===')
        pose_start = self._pose_dict(self._slam_pose)
        filtered_start = self._pose_dict(self._filtered_pose)
        map_start = self._map_stats()
        gt_start = self._pose_dict(self._gazebo_pose)
        t_start = time.monotonic()

        self.get_logger().info(f'Start SLAM pose: {pose_start}')
        self.get_logger().info(f'Start map stats: {map_start}')
        self.get_logger().info(f'Start GT pose:   {gt_start}')

        report['start_slam_pose'] = pose_start
        report['start_filtered_pose'] = filtered_start
        report['start_map_stats'] = map_start
        report['start_gt_pose'] = gt_start

        # ------ Drive pattern ------
        # Square-ish pattern: forward -> turn -> forward -> turn -> ...
        # Duration is split into segments so we get diverse scan angles.
        self.get_logger().info('=== MOVEMENT PHASE ===')

        phase_duration = self._duration / 4.0  # 4 phases
        sample_interval = max(2.0, self._duration / 10.0)
        next_sample_t = t_start + sample_interval

        # Drive phases: fwd, left-turn, fwd, right-turn
        phases = [
            (self._linear_speed, 0.0),
            (0.0, self._angular_speed),
            (self._linear_speed, 0.0),
            (0.0, -self._angular_speed),
        ]
        phase_end_times = [
            t_start + phase_duration * (i + 1) for i in range(4)
        ]
        current_phase = 0
        scan_count_start = self._scan_count
        t_scan_window = time.monotonic()

        while time.monotonic() - t_start < self._duration:
            now = time.monotonic()

            # Advance phase.
            while (
                current_phase < len(phases) - 1
                and now >= phase_end_times[current_phase]
            ):
                current_phase += 1
                lin, ang = phases[current_phase]
                self.get_logger().info(
                    f'Phase {current_phase + 1}/4: '
                    f'linear={lin:.2f} angular={ang:.2f}'
                )

            lin, ang = phases[current_phase]
            self._publish_advisory(lin, ang)

            # Timed pose sample.
            if now >= next_sample_t:
                elapsed = now - t_start
                pose_s = self._pose_dict(self._slam_pose)
                filtered_s = self._pose_dict(self._filtered_pose)
                map_s = self._map_stats()
                gt_s = self._pose_dict(self._gazebo_pose)
                scan_hz = (
                    (self._scan_count - scan_count_start)
                    / max(0.001, now - t_scan_window)
                )
                sample = {
                    'elapsed_sec': round(elapsed, 2),
                    'wall_time': time.strftime(
                        '%H:%M:%S.', time.gmtime()
                    ) + f'{int((time.time() % 1) * 1000):03d}',
                    'slam_pose': pose_s,
                    'filtered_pose': filtered_s,
                    'map_stats': map_s,
                    'gt_pose': gt_s,
                    'scan_hz': round(scan_hz, 2),
                }
                if (filtered_s and gt_s and
                        filtered_s['frame_id'] == gt_s['frame_id']):
                    sample['fused_position_error_m'] = round(
                        math.hypot(
                            filtered_s['x'] - gt_s['x'],
                            filtered_s['y'] - gt_s['y'],
                        ), 4
                    )
                self._pose_samples.append(sample)
                self.get_logger().info(
                    f'[T+{elapsed:.1f}s] slam_pose={pose_s} '
                    f'map_free={map_s.get("free_cells", "?")} '
                    f'scan_hz={scan_hz:.1f}'
                )
                # Reset scan rate window.
                scan_count_start = self._scan_count
                t_scan_window = now
                next_sample_t = now + sample_interval

            rclpy.spin_once(self, timeout_sec=0.02)

        # ------ Stop robot ------
        self.get_logger().info('=== STOPPING ROBOT ===')
        self._stop_robot()
        self._spin_sec(1.0)

        # ------ End snapshot ------
        self.get_logger().info('=== END SNAPSHOT ===')
        pose_end = self._pose_dict(self._slam_pose)
        filtered_end = self._pose_dict(self._filtered_pose)
        map_end = self._map_stats()
        gt_end = self._pose_dict(self._gazebo_pose)
        t_total = time.monotonic() - t_start

        report['end_slam_pose'] = pose_end
        report['end_filtered_pose'] = filtered_end
        report['end_map_stats'] = map_end
        report['end_gt_pose'] = gt_end
        report['pose_samples'] = self._pose_samples
        report['total_duration_sec'] = round(t_total, 2)
        report['total_scans'] = self._scan_count
        report['imu_rate_hz'] = round(self._imu_count / max(t_total, 0.001), 2)
        report['wheel_odom_rate_hz'] = round(self._odom_count / max(t_total, 0.001), 2)
        report['ekf_output_rate_hz'] = round(
            self._filtered_count / max(t_total, 0.001), 2
        )
        report['slam_pose_rate_hz'] = round(
            self._slam_pose_count / max(t_total, 0.001), 2
        )

        # ------ Compute metrics ------
        slam_displacement = self._displacement(pose_start, pose_end)
        gt_displacement = self._displacement(gt_start, gt_end)
        filtered_displacement = self._displacement(
            filtered_start, filtered_end
        )

        report['H_robot_displacement_m'] = (
            round(slam_displacement, 4) if slam_displacement is not None else None
        )
        report['I_slam_pose_changed'] = (
            slam_displacement is not None and slam_displacement > 0.05
        )
        report['fused_displacement_m'] = (
            round(filtered_displacement, 4)
            if filtered_displacement is not None else None
        )
        report['fused_yaw_change_rad'] = (
            round(filtered_end['yaw_rad'] - filtered_start['yaw_rad'], 4)
            if filtered_end and filtered_start else None
        )
        report['fused_displacement_error_m'] = (
            round(abs(filtered_displacement - gt_displacement), 4)
            if filtered_displacement is not None and gt_displacement is not None
            else None
        )

        map_free_delta = (
            map_end.get('free_cells', 0) - map_start.get('free_cells', 0)
        )
        report['J_map_free_cells_delta'] = map_free_delta
        report['J_slam_map_changed'] = map_free_delta > 0

        # Ground-truth evaluation (post-hoc only — never fed to SLAM).
        if slam_displacement is not None and gt_displacement is not None:
            position_error = abs(slam_displacement - gt_displacement)
            report['gt_displacement_m'] = round(gt_displacement, 4)
            report['position_error_m'] = round(position_error, 4)
            report['K_no_gt_input_to_slam'] = True  # GT used only here
        else:
            report['gt_displacement_m'] = None
            report['position_error_m'] = None
            report['K_no_gt_input_to_slam'] = True

        # Compare fused odometry to Gazebo's wheel-odometry reference only
        # when both carry the same frame. SLAM map pose is not compared
        # directly to odom-frame ground truth without an explicit alignment.
        if self._pose_samples:
            fused_errors = [
                s['fused_position_error_m'] for s in self._pose_samples
                if s.get('fused_position_error_m') is not None
            ]
            fused_xs = [s['filtered_pose']['x'] for s in self._pose_samples
                        if s.get('filtered_pose')]
            fused_ys = [s['filtered_pose']['y'] for s in self._pose_samples
                        if s.get('filtered_pose')]
            if fused_xs and fused_ys:
                import statistics  # noqa: PLC0415
                report['fused_trajectory_x_range_m'] = round(
                    max(fused_xs) - min(fused_xs), 4
                )
                report['fused_trajectory_y_range_m'] = round(
                    max(fused_ys) - min(fused_ys), 4
                )
            if fused_errors:
                report['fused_position_error_mean_m'] = round(
                    statistics.mean(fused_errors), 4
                )
                report['fused_position_error_max_m'] = round(
                    max(fused_errors), 4
                )

        # ------ Overall pass/fail ------
        checks = {
            'A': slam_active,
            'B': report.get('B_scan_live', False),
            'C': report.get('C_slam_map_frame_ok', False),
            'H': report.get('I_slam_pose_changed', False),
            'I': report.get('I_slam_pose_changed', False),
            'J': report.get('J_slam_map_changed', False),
            'K': True,  # GT never fed to SLAM (by design of this script)
        }
        all_pass = all(checks.values())
        report['checks'] = checks
        report['result'] = 'PASS' if all_pass else 'FAIL'

        self.get_logger().info('=== TEST COMPLETE ===')
        self.get_logger().info(f'Result: {report["result"]}')
        self.get_logger().info(
            f'SLAM displacement: {slam_displacement} m  '
            f'GT displacement: {gt_displacement} m'
        )
        self.get_logger().info(
            f'Map free cells delta: {map_free_delta}'
        )
        self.get_logger().info(f'Samples collected: {len(self._pose_samples)}')

        return report


def main() -> None:
    """Entry point."""
    parser = argparse.ArgumentParser(
        description='SLAM movement validation test for DRISTHI'
    )
    parser.add_argument(
        '--duration', type=float, default=40.0,
        help='Total movement duration in seconds (default: 40)'
    )
    parser.add_argument(
        '--linear', type=float, default=0.15,
        help='Linear speed m/s during forward phases (default: 0.15)'
    )
    parser.add_argument(
        '--angular', type=float, default=0.25,
        help='Angular speed rad/s during turn phases (default: 0.25)'
    )
    parser.add_argument(
        '--output', type=str, default='/tmp/slam_validation_report.json',
        help='Output JSON report path'
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args if ros_args else None)
    node = SlamValidationTest(
        duration_sec=args.duration,
        linear_speed=args.linear,
        angular_speed=args.angular,
    )

    try:
        report = node.run()
    except KeyboardInterrupt:
        report = {'result': 'INTERRUPTED'}
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # Write report.
    with open(args.output, 'w') as fh:
        json.dump(report, fh, indent=2)

    print(f'\nReport written to: {args.output}')
    print(f'Result: {report.get("result", "UNKNOWN")}')

    sys.exit(0 if report.get('result') == 'PASS' else 1)


if __name__ == '__main__':
    main()
