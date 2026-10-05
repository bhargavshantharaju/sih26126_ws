#!/usr/bin/env python3
"""
Paired wheel-input isolation test for a running DRISTHI simulation.

Run against ``slam:=true fusion:=true``. The test drives only the existing
safety-supervisor advisory interface. It compares wheel+IMU with IMU-only,
resetting Gazebo and the EKF before each identical straight-line run. Gazebo
odometry is recorded solely as a post-hoc evaluation reference.
"""

import argparse
import json
import math
import statistics
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String


def _pose(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return {
        'x': p.x,
        'y': p.y,
        'yaw': math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        ),
        'frame_id': msg.header.frame_id,
    }


def _distance(a, b):
    return math.hypot(b['x'] - a['x'], b['y'] - a['y'])


class WheelContributionTest(Node):
    """Run the paired experiment and save measurements as JSON."""

    def __init__(self, args):
        super().__init__('wheel_contribution_test')
        self.args = args
        self.latest = {'imu': None, 'raw_wheel': None, 'wheel': None,
                       'filtered': None}
        self.counts = {'imu': 0, 'raw_wheel': 0, 'wheel': 0, 'filtered': 0}
        self.samples = []
        self.create_subscription(Imu, '/imu', self._imu, 20)
        self.create_subscription(Odometry, '/model/sih_ugv/odometry',
                                 self._raw_wheel, 20)
        self.create_subscription(Odometry, '/fusion/wheel_odometry',
                                 self._wheel, 20)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._filtered, 20)
        self.nav_pub = self.create_publisher(
            String, '/planner/local_navigation_state', 10)
        self.clearance_pub = self.create_publisher(
            Float32, '/planner/local_obstacle_score', 10)
        self.confidence_pub = self.create_publisher(
            Float32, '/localization/confidence', 10)
        self.command_pub = self.create_publisher(
            Twist, '/planner/local_recommended_cmd_vel', 10)
        self.adapter_params = self.create_client(
            SetParameters, '/ekf_sensor_adapter/set_parameters')

    def _imu(self, msg):
        self.latest['imu'] = msg
        self.counts['imu'] += 1

    def _raw_wheel(self, msg):
        self.latest['raw_wheel'] = msg
        self.counts['raw_wheel'] += 1

    def _wheel(self, msg):
        self.latest['wheel'] = msg
        self.counts['wheel'] += 1

    def _filtered(self, msg):
        self.latest['filtered'] = msg
        self.counts['filtered'] += 1
        if self.latest['raw_wheel'] is not None:
            self.samples.append((
                _pose(msg), _pose(self.latest['raw_wheel']), time.monotonic()))

    def _spin_for(self, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.02)

    def _request(self, client, request, label):
        if not client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError(f'{label} service is unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'{label} service call failed')
        return future.result()

    def _set_wheel_gate(self, enabled):
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                               bool_value=enabled)
        request = SetParameters.Request(parameters=[
            Parameter(name='wheel_input_enabled', value=value)])
        result = self._request(self.adapter_params, request,
                               'adapter parameter')
        if not result.results[0].successful:
            raise RuntimeError(result.results[0].reason)

    def _publish(self, linear):
        state = String(data='LOCAL_NAVIGATE')
        obstacle = Float32(data=1.0)
        confidence = Float32(data=1.0)
        command = Twist()
        command.linear.x = linear
        self.nav_pub.publish(state)
        self.clearance_pub.publish(obstacle)
        self.confidence_pub.publish(confidence)
        self.command_pub.publish(command)

    def _run_case(self, enabled):
        label = 'wheel_plus_imu' if enabled else 'imu_only_wheel_disabled'
        self.get_logger().info(
            f'{label}: wheel input {"ACTIVE" if enabled else "DISABLED"}; '
            'IMU input ACTIVE')
        self._set_wheel_gate(enabled)
        self._spin_for(3.0)
        if not all(self.latest[k] is not None
                   for k in ('imu', 'raw_wheel', 'filtered')):
            raise RuntimeError('IMU, wheel, or EKF output not live after reset')
        start = dict(self.counts)
        start_filtered = _pose(self.latest['filtered'])
        start_gt = _pose(self.latest['raw_wheel'])
        self.samples.clear()
        start_time = time.monotonic()
        end = start_time + self.args.duration
        next_sample = start_time
        while time.monotonic() < end:
            now = time.monotonic()
            self._publish(self.args.linear_speed)
            if now >= next_sample:
                self._spin_for(0.01)
                if self.latest['filtered'] and self.latest['raw_wheel']:
                    self.samples.append((
                        _pose(self.latest['filtered']),
                        _pose(self.latest['raw_wheel']), now))
                next_sample = now + 0.2
            rclpy.spin_once(self, timeout_sec=0.02)
        self._publish(0.0)
        self._spin_for(1.0)
        duration = time.monotonic() - start_time
        finish_filtered = _pose(self.latest['filtered'])
        finish_gt = _pose(self.latest['raw_wheel'])
        delta = {k: self.counts[k] - start[k] for k in start}
        aligned = (start_filtered['frame_id'] == start_gt['frame_id'] ==
                   finish_filtered['frame_id'] == finish_gt['frame_id'])
        errors = []
        if aligned:
            for fused, gt, _stamp in self.samples:
                errors.append(_distance(fused, gt))
        return {
            'case': label,
            'wheel_input': 'ACTIVE' if enabled else 'DISABLED',
            'imu_input': 'ACTIVE',
            'start_fused': start_filtered,
            'end_fused': finish_filtered,
            'start_ground_truth_evaluation_only': start_gt,
            'end_ground_truth_evaluation_only': finish_gt,
            'fused_displacement_m': _distance(start_filtered, finish_filtered),
            'ground_truth_displacement_m': _distance(start_gt, finish_gt),
            'fused_yaw_change_rad': finish_filtered['yaw'] - start_filtered['yaw'],
            'position_error_mean_m': statistics.mean(errors) if errors else None,
            'position_error_max_m': max(errors) if errors else None,
            'position_error_samples': len(errors),
            'duration_sec': duration,
            'rates_hz': {k: round(v / duration, 2) for k, v in delta.items()},
            'ekf_input_state': {
                'subscribed_wheel_topic': '/fusion/wheel_odometry',
                'wheel_relay_messages': delta['wheel'],
                'imu_topic': '/fusion/imu',
                'raw_imu_messages': delta['imu'],
            },
            'fused_samples': [
                {'fused': f, 'ground_truth_evaluation_only': g, 't': t}
                for f, g, t in self.samples
            ],
        }

    def run_case(self):
        self._spin_for(2.0)
        enabled = self.args.case == 'wheel_on'
        case = self._run_case(enabled)
        return {
            'test': 'wheel_input_isolation_single_case',
            'case': self.args.case,
            'timestamp_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'design': {
                'motion': 'constant forward command through safety advisory input',
                'duration_sec': self.args.duration,
                'linear_speed_mps': self.args.linear_speed,
                'ground_truth_role': 'post-hoc evaluation only',
                'initial_state_requirement': (
                    'start a fresh full_stack launch for each case; both '
                    'cases start from the same SDF spawn pose with a fresh EKF'
                ),
            },
            'measurements': case,
        }

    @staticmethod
    def compare(normal, isolated):
        """Apply the fixed paired acceptance criteria to saved case reports."""
        a = normal['measurements']
        b = isolated['measurements']
        gt_distance = a['ground_truth_displacement_m']
        paired_difference = abs(
            a['fused_displacement_m'] - b['fused_displacement_m'])
        a_dx = a['end_fused']['x'] - a['start_fused']['x']
        a_dy = a['end_fused']['y'] - a['start_fused']['y']
        b_dx = b['end_fused']['x'] - b['start_fused']['x']
        b_dy = b['end_fused']['y'] - b['start_fused']['y']
        trajectory_separation = math.hypot(a_dx - b_dx, a_dy - b_dy)
        # Prior stack runs produced about 0.56 m GT travel in a 32 s mixed
        # motion. A 0.5 m paired gap is therefore at least one full prior-run
        # travel scale and cannot be explained by small sample jitter.
        criterion = {
            'minimum_normal_gt_displacement_m': 0.5,
            'maximum_normal_position_error_m': 0.25,
            'minimum_paired_trajectory_separation_m': 0.5,
            'minimum_wheel_off_relay_messages': 0,
            'minimum_wheel_on_relay_rate_hz': 1.0,
            'minimum_imu_rate_hz': 10.0,
        }
        acceptance = {
            'normal_translation_sufficient': gt_distance >= 0.5,
            'normal_position_error_bounded': (
                a['position_error_mean_m'] is not None and
                a['position_error_mean_m'] <= 0.25),
            'paired_trajectory_difference_material':
                trajectory_separation >= 0.5,
            'wheel_relay_stopped_when_disabled':
                b['rates_hz']['wheel'] == 0.0,
            'wheel_relay_active_when_enabled':
                a['rates_hz']['wheel'] >= 1.0,
            'imu_active_both_cases': min(
                a['rates_hz']['imu'], b['rates_hz']['imu']) >= 10.0,
        }
        return {
            'test': 'paired_wheel_input_isolation',
            'acceptance_criterion': criterion,
            'normal': normal,
            'wheel_off': isolated,
            'paired_fused_displacement_difference_m': paired_difference,
            'paired_terminal_trajectory_separation_m': trajectory_separation,
            'acceptance': acceptance,
            'result': 'PASS' if all(acceptance.values()) else 'INCONCLUSIVE',
        }


def main():
    """Run one paired-test case or compare two saved case artifacts."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=45.0)
    parser.add_argument('--linear-speed', type=float, default=0.12)
    parser.add_argument('--case', choices=['wheel_on', 'wheel_off'],
                        default='wheel_on')
    parser.add_argument('--compare', nargs=2, metavar=('NORMAL_JSON', 'WHEEL_OFF_JSON'))
    parser.add_argument('--output', default='/tmp/wheel_contribution.json')
    args, ros_args = parser.parse_known_args()
    if args.compare:
        with open(args.compare[0], encoding='utf-8') as stream:
            normal = json.load(stream)
        with open(args.compare[1], encoding='utf-8') as stream:
            isolated = json.load(stream)
        # Comparison is a pure post-hoc calculation; it does not publish data.
        result = WheelContributionTest.compare(normal, isolated)
        with open(args.output, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, indent=2)
        print(json.dumps({k: v for k, v in result.items()
                          if k not in ('normal', 'wheel_off')}, indent=2))
        return
    rclpy.init(args=ros_args or None)
    node = WheelContributionTest(args)
    try:
        result = node.run_case()
        with open(args.output, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, indent=2)
        print(json.dumps({k: v for k, v in result.items()
                          if k != 'measurements'}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
