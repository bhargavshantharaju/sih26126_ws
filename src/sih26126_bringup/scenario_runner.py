#!/usr/bin/env python3
"""
DRISTHI Scenario Validation Framework.

A standalone CLI script (uses rclpy) that subscribes to ROS 2 topics for a
fixed duration, checks success/failure criteria, and writes a JSON result
artifact to artifacts/scenarios/<scenario_name>.json.

Usage::

    ros2 run sih26126_bringup scenario_runner <scenario_id_or_name> [--timeout <sec>]

  or::

    python3 scenario_runner.py --list
    python3 scenario_runner.py --scenario scenario_1

Each scenario definition includes:
  name, description, world_file, timeout_sec,
  expected_topics, success_criteria, failure_criteria

The script does NOT launch Gazebo or the full stack. It assumes the stack is
already running and observes topics for `timeout_sec` seconds.
"""

import argparse
import datetime
import json
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, String

# ──────────────────────────────────────────────────────────────────────────────
# Scenario definitions
# ──────────────────────────────────────────────────────────────────────────────

SCENARIOS: List[Dict[str, Any]] = [
    {
        'name': 'scenario_1',
        'description': (
            'Normal exploration: robot starts with all-unknown map, '
            'should navigate, produce a /map, and reach SAFE safety state.'
        ),
        'world_file': 'showcase_terrain.sdf',
        'timeout_sec': 60,
        'expected_topics': [
            '/safety/state',
            '/map',
            '/cmd_vel',
        ],
        'success_criteria': [
            # At least one SAFE safety state observed
            {'type': 'topic_value_seen', 'topic': '/safety/state', 'value': 'SAFE'},
            # /map must be published at least once
            {'type': 'topic_published', 'topic': '/map'},
            # Robot must move (non-zero linear velocity published)
            {'type': 'nonzero_linear', 'topic': '/cmd_vel'},
        ],
        'failure_criteria': [
            # If no /safety/state at all within timeout, that is a failure
            {'type': 'topic_never_published', 'topic': '/safety/state'},
        ],
    },
    {
        'name': 'scenario_2',
        'description': (
            'Obstacle response: robot encounters the moving obstacle in '
            'dynamic_obstacle_world and should reach a STOPPED state.'
        ),
        'world_file': 'dynamic_obstacle_world.sdf',
        'timeout_sec': 90,
        'expected_topics': [
            '/safety/state',
            '/safety/reason',
        ],
        'success_criteria': [
            {'type': 'topic_value_seen', 'topic': '/safety/state', 'value': 'STOPPED'},
        ],
        'failure_criteria': [
            {'type': 'topic_never_published', 'topic': '/safety/state'},
        ],
    },
    {
        'name': 'scenario_3',
        'description': (
            'Sensor dropout: lidar_processor is killed after 5 s. '
            'The DegradedAutonomyManager should detect the sensor loss '
            'and publish DEGRADED_SENSOR on /autonomy/mode.'
        ),
        'world_file': 'showcase_terrain.sdf',
        'timeout_sec': 30,
        'expected_topics': [
            '/autonomy/mode',
        ],
        'success_criteria': [
            {
                'type': 'autonomy_mode_seen',
                'topic': '/autonomy/mode',
                'mode': 'DEGRADED_SENSOR',
            },
        ],
        'failure_criteria': [
            {'type': 'topic_never_published', 'topic': '/autonomy/mode'},
        ],
        # Trigger: kill lidar_processor after 5 seconds
        'trigger': {
            'delay_sec': 5,
            'action': 'kill_node',
            'node_name': 'lidar_processor',
        },
    },
    {
        'name': 'scenario_4',
        'description': (
            'Localization degradation: a low localization confidence value '
            'is injected on /localization/confidence. The DegradedAutonomyManager '
            'should detect this and publish DEGRADED_LOCALIZATION on /autonomy/mode.'
        ),
        'world_file': 'showcase_terrain.sdf',
        'timeout_sec': 30,
        'expected_topics': [
            '/autonomy/mode',
            '/localization/confidence',
        ],
        'success_criteria': [
            {
                'type': 'autonomy_mode_seen',
                'topic': '/autonomy/mode',
                'mode': 'DEGRADED_LOCALIZATION',
            },
        ],
        'failure_criteria': [
            {'type': 'topic_never_published', 'topic': '/autonomy/mode'},
        ],
        # Trigger: inject low localization confidence after 3 seconds
        'trigger': {
            'delay_sec': 3,
            'action': 'inject_low_confidence',
            'topic': '/localization/confidence',
            'value': 0.05,
        },
    },
]

# Map name → definition for easy lookup
SCENARIO_BY_NAME: Dict[str, Dict[str, Any]] = {s['name']: s for s in SCENARIOS}

# ──────────────────────────────────────────────────────────────────────────────
# Observer node
# ──────────────────────────────────────────────────────────────────────────────


class ScenarioObserver(Node):
    """
    Observe ROS 2 topics for scenario evaluation.

    Subscribes to topics defined in a scenario and tracks observations
    needed to evaluate success/failure criteria.
    """

    def __init__(self, scenario: Dict[str, Any]) -> None:
        """Initialise observer and subscribe to all scenario topics."""
        super().__init__('scenario_observer')
        self._scenario = scenario
        self._lock = threading.Lock()

        # Raw observations: topic → list of values (or timestamps)
        self._observations: Dict[str, List[Any]] = {}
        self._published: Dict[str, bool] = {}
        self._nonzero_linear_seen: Dict[str, bool] = {}

        # Subscribe to known topics based on criteria
        all_topics = set(scenario.get('expected_topics', []))
        for criterion in scenario.get('success_criteria', []) + scenario.get(
            'failure_criteria', []
        ):
            t = criterion.get('topic')
            if t:
                all_topics.add(t)

        self._subs = []
        for topic in all_topics:
            sub = self._subscribe_topic(topic)
            if sub:
                self._subs.append(sub)
                self._observations[topic] = []
                self._published[topic] = False

        # Trigger publisher for confidence injection
        self._confidence_pub = self.create_publisher(
            Float32, '/localization/confidence', 10)

        self.get_logger().info(
            f'[scenario_runner] Observing {len(self._subs)} topics '
            f"for scenario '{scenario['name']}'"
        )

    def _subscribe_topic(self, topic: str):
        """Create appropriate subscription based on topic name."""
        if topic in ('/safety/state', '/safety/reason', '/planner/local_navigation_state'):
            return self.create_subscription(
                String, topic, lambda msg, t=topic: self._str_callback(t, msg), 10
            )
        elif topic == '/map':
            return self.create_subscription(
                OccupancyGrid, topic, lambda msg, t=topic: self._map_callback(t, msg), 10
            )
        elif topic == '/cmd_vel':
            return self.create_subscription(
                Twist, topic, lambda msg, t=topic: self._twist_callback(t, msg), 10
            )
        elif topic in ('/localization/confidence', '/obstacles/dynamic_risk'):
            return self.create_subscription(
                Float32, topic, lambda msg, t=topic: self._float_callback(t, msg), 10
            )
        elif topic == '/autonomy/mode':
            # Import lazily so this script still runs without sih26126_msgs
            try:
                from sih26126_msgs.msg import AutonomyMode  # type: ignore
                return self.create_subscription(
                    AutonomyMode,
                    topic,
                    lambda msg, t=topic: self._autonomy_mode_callback(t, msg),
                    10,
                )
            except ImportError:
                self.get_logger().warn(
                    'sih26126_msgs not available — /autonomy/mode will not be observed'
                )
                return None
        else:
            # Generic String fallback
            return self.create_subscription(
                String, topic, lambda msg, t=topic: self._str_callback(t, msg), 10
            )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _str_callback(self, topic: str, msg: String) -> None:
        with self._lock:
            self._published[topic] = True
            self._observations[topic].append({'t': time.monotonic(), 'value': msg.data})

    def _map_callback(self, topic: str, _msg: OccupancyGrid) -> None:
        with self._lock:
            self._published[topic] = True
            self._observations[topic].append(
                {'t': time.monotonic(), 'value': 'received'})

    def _twist_callback(self, topic: str, msg: Twist) -> None:
        with self._lock:
            self._published[topic] = True
            linear_x = msg.linear.x
            self._observations[topic].append(
                {'t': time.monotonic(), 'linear_x': linear_x})
            if abs(linear_x) > 0.01:
                self._nonzero_linear_seen[topic] = True

    def _float_callback(self, topic: str, msg: Float32) -> None:
        with self._lock:
            self._published[topic] = True
            self._observations[topic].append(
                {'t': time.monotonic(), 'value': float(msg.data)})

    def _autonomy_mode_callback(self, topic: str, msg) -> None:
        with self._lock:
            self._published[topic] = True
            self._observations[topic].append({'t': time.monotonic(), 'mode': msg.mode})

    # ── Trigger actions ───────────────────────────────────────────────────────

    def inject_confidence(self, value: float) -> None:
        """Publish a low confidence value on /localization/confidence."""
        msg = Float32()
        msg.data = value
        self._confidence_pub.publish(msg)
        self.get_logger().info(
            f'[trigger] Injected localization confidence={value:.3f}')

    # ── Evaluation ────────────────────────────────────────────────────────────

    def evaluate_criteria(
        self, criteria: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """Return a list of {criterion, passed, detail} records."""
        results = []
        with self._lock:
            for c in criteria:
                ctype = c['type']
                topic = c.get('topic', '')

                if ctype == 'topic_published':
                    passed = self._published.get(topic, False)
                    results.append(
                        {
                            'criterion': c,
                            'passed': passed,
                            'detail': (
                                f"{'Published' if passed else 'NOT published'}: {topic}"
                            ),
                        }
                    )

                elif ctype == 'topic_never_published':
                    never = not self._published.get(topic, False)
                    results.append(
                        {
                            'criterion': c,
                            'passed': never,
                            'detail': (
                                f"{'NEVER published (failure)' if never else 'Was published'}"
                                f': {topic}'
                            ),
                        }
                    )

                elif ctype == 'topic_value_seen':
                    expected_value = c['value']
                    obs = self._observations.get(topic, [])
                    seen = any(o.get('value') == expected_value for o in obs)
                    results.append(
                        {
                            'criterion': c,
                            'passed': seen,
                            'detail': (
                                f"{'Seen' if seen else 'NOT seen'}: "
                                f'{topic}=={expected_value!r}'
                            ),
                        }
                    )

                elif ctype == 'nonzero_linear':
                    seen = self._nonzero_linear_seen.get(topic, False)
                    results.append(
                        {
                            'criterion': c,
                            'passed': seen,
                            'detail': (
                                f"{'Non-zero linear velocity seen' if seen else 'No non-zero'}"
                                f' linear: {topic}'
                            ),
                        }
                    )

                elif ctype == 'autonomy_mode_seen':
                    expected_mode = c['mode']
                    obs = self._observations.get(topic, [])
                    seen = any(o.get('mode') == expected_mode for o in obs)
                    results.append(
                        {
                            'criterion': c,
                            'passed': seen,
                            'detail': (
                                f"{'Seen' if seen else 'NOT seen'}: "
                                f'{topic}.mode=={expected_mode!r}'
                            ),
                        }
                    )
                else:
                    results.append(
                        {
                            'criterion': c,
                            'passed': False,
                            'detail': f'Unknown criterion type: {ctype}',
                        }
                    )
        return results

    def get_summary_observations(self) -> Dict[str, Any]:
        """Return a compact summary of observations for the artifact."""
        with self._lock:
            summary = {}
            for topic, obs in self._observations.items():
                summary[topic] = {
                    'published': self._published.get(topic, False),
                    'count': len(obs),
                    'last_values': obs[-5:] if obs else [],
                }
        return summary


# ──────────────────────────────────────────────────────────────────────────────
# Runner
# ──────────────────────────────────────────────────────────────────────────────


def run_scenario(
    scenario: Dict[str, Any],
    timeout_override: Optional[float] = None,
) -> Dict[str, Any]:
    """
    Execute one scenario and return a result dict.

    Spins the observer for timeout_sec, evaluates criteria,
    and returns the result dict.
    """
    timeout = timeout_override or scenario['timeout_sec']
    started_at = datetime.datetime.utcnow().isoformat() + 'Z'
    t0 = time.monotonic()

    print(f"\n{'=' * 60}")
    print(f"  SCENARIO: {scenario['name']}")
    print(f"  {scenario['description']}")
    print(f"  World: {scenario.get('world_file', 'N/A')}")
    print(f'  Timeout: {timeout}s')
    print(f"{'=' * 60}")

    rclpy.init(args=None)
    observer = ScenarioObserver(scenario)
    executor = MultiThreadedExecutor()
    executor.add_node(observer)

    # Handle trigger if present
    trigger = scenario.get('trigger')
    trigger_fired = False

    def spin_loop():
        nonlocal trigger_fired
        deadline = t0 + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)

            # Fire trigger if defined
            if trigger and not trigger_fired:
                delay = trigger.get('delay_sec', 0)
                if time.monotonic() - t0 >= delay:
                    trigger_fired = True
                    action = trigger['action']
                    if action == 'inject_low_confidence':
                        observer.inject_confidence(trigger.get('value', 0.05))
                    elif action == 'kill_node':
                        node_name = trigger.get('node_name', '')
                        print(f'[trigger] Attempting to kill node: {node_name}')
                        os.system(
                            f'ros2 lifecycle set /{node_name} shutdown 2>/dev/null || true'
                        )

    spin_thread = threading.Thread(target=spin_loop, daemon=True)
    spin_thread.start()
    spin_thread.join(timeout=timeout + 2.0)

    duration = time.monotonic() - t0

    # Evaluate criteria
    success_results = observer.evaluate_criteria(scenario.get('success_criteria', []))
    failure_results = observer.evaluate_criteria(scenario.get('failure_criteria', []))

    # A scenario passes when:
    # - All success criteria pass AND
    # - No failure criteria pass (failure criteria passing means failure condition triggered)
    success_all = all(r['passed'] for r in success_results)
    # failure_criteria of type "topic_never_published" — if it passes (never published),
    # that IS the failure condition.
    failure_triggered = any(
        r['passed'] and r['criterion']['type'] == 'topic_never_published'
        for r in failure_results
    )

    overall_pass = success_all and not failure_triggered

    # Build observations summary
    observations = observer.get_summary_observations()

    # Compose result
    result = {
        'scenario_name': scenario['name'],
        'description': scenario['description'],
        'world_file': scenario.get('world_file', ''),
        'started_at': started_at,
        'duration_sec': round(duration, 2),
        'timeout_sec': timeout,
        'passed': overall_pass,
        'success_criteria_results': [
            {'criterion': r['criterion'], 'passed': r['passed'], 'detail': r['detail']}
            for r in success_results
        ],
        'failure_criteria_results': [
            {'criterion': r['criterion'], 'passed': r['passed'], 'detail': r['detail']}
            for r in failure_results
        ],
        'trigger_fired': trigger_fired,
        'relevant_observations': observations,
    }

    # Print summary
    status = 'PASS' if overall_pass else 'FAIL'
    print(f'\nResult: {status}  (duration={duration:.1f}s)')
    print('Success criteria:')
    for r in success_results:
        mark = '✓' if r['passed'] else '✗'
        print(f"  {mark} {r['detail']}")
    if failure_results:
        print('Failure criteria (triggered=failure):')
        for r in failure_results:
            mark = '!' if r['passed'] else 'ok'
            print(f"  [{mark}] {r['detail']}")

    # Cleanup
    executor.shutdown()
    observer.destroy_node()
    rclpy.shutdown()

    return result


def write_artifact(
    result: Dict[str, Any],
    output_dir: str = 'artifacts/scenarios',
) -> str:
    """Write result dict as JSON artifact. Returns the file path."""
    os.makedirs(output_dir, exist_ok=True)
    filename = os.path.join(output_dir, f"{result['scenario_name']}.json")
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, default=str)
    return filename


# ──────────────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────────────


def main() -> None:
    """Parse CLI arguments and run selected scenarios."""
    parser = argparse.ArgumentParser(
        description='DRISTHI Scenario Validation Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '--list', action='store_true', help='List all available scenarios and exit'
    )
    parser.add_argument(
        '--scenario',
        type=str,
        default=None,
        help='Scenario name to run (e.g. scenario_1). Omit to run ALL scenarios.',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=None,
        help='Override timeout_sec for the selected scenario.',
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help=(
            'Directory to write artifact JSON files '
            '(default: artifacts/scenarios relative to script).'
        ),
    )

    args = parser.parse_args()

    # Resolve output dir relative to this script's location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_root = os.path.abspath(os.path.join(script_dir, '..', '..'))
    default_output = os.path.join(ws_root, 'artifacts', 'scenarios')
    output_dir = args.output_dir or default_output

    if args.list:
        print('Available scenarios:')
        for s in SCENARIOS:
            print(f"  {s['name']}: {s['description'][:80]}")
        sys.exit(0)

    if args.scenario:
        if args.scenario not in SCENARIO_BY_NAME:
            print(f"ERROR: Unknown scenario '{args.scenario}'.")
            print(f'Available: {list(SCENARIO_BY_NAME.keys())}')
            sys.exit(1)
        scenarios_to_run = [SCENARIO_BY_NAME[args.scenario]]
    else:
        print('No --scenario specified. Running ALL scenarios sequentially.')
        scenarios_to_run = SCENARIOS

    all_results = []
    for scenario in scenarios_to_run:
        result = run_scenario(scenario, timeout_override=args.timeout)
        artifact_path = write_artifact(result, output_dir)
        print(f'Artifact written: {artifact_path}')
        all_results.append(result)

    # Summary
    passed = sum(1 for r in all_results if r['passed'])
    total = len(all_results)
    print(f"\n{'=' * 60}")
    print(f'  Scenarios run: {total}  Passed: {passed}  Failed: {total - passed}')
    print(f"{'=' * 60}")

    sys.exit(0 if passed == total else 1)


if __name__ == '__main__':
    main()
