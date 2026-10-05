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
"""DRISTHI mission intelligence layer.

Turns real autonomy events into human-readable mission information.
Generates structured MissionEvent messages only when system state
genuinely changes — no fabricated events, no fake metrics.

Publishes:
  /autonomy/events   -- MissionEvent stream
  /autonomy/summary  -- std_msgs/String (human-readable compact summary)
"""

import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sih26126_msgs.msg import AutonomyMode, MissionEvent, TrackedObstacleArray
from std_msgs.msg import Float32, String


class MissionIntelligence(Node):
    """Generate human-readable mission events from real autonomy signals."""

    def __init__(self) -> None:
        """Initialize mission intelligence subscriptions and publishers."""
        super().__init__('mission_intelligence')

        # --- Previous state for change detection ---
        self._prev_autonomy_mode: Optional[str] = None
        self._prev_safety_state: Optional[str] = None
        self._prev_safety_reason: Optional[str] = None
        self._prev_recovery_state: Optional[str] = None
        self._prev_mission_state: Optional[str] = None
        self._prev_exploration_state: Optional[str] = None
        self._prev_dynamic_obs_count = 0

        # --- Current state ---
        self._autonomy_mode: Optional[str] = None
        self._safety_state: Optional[str] = None
        self._safety_reason: Optional[str] = None
        self._recovery_state: Optional[str] = None
        self._mission_state: Optional[str] = None
        self._exploration_state: Optional[str] = None
        self._localization_source: Optional[str] = None
        self._localization_confidence: Optional[float] = None
        self._dynamic_obs_count = 0
        self._dynamic_risk: Optional[float] = None
        self._exploration_goal_status: Optional[str] = None

        # --- Subscriptions ---
        self.create_subscription(
            AutonomyMode, '/autonomy/mode', self._autonomy_cb, 10)
        self.create_subscription(
            String, '/safety/state', self._safety_state_cb, 10)
        self.create_subscription(
            String, '/safety/reason', self._safety_reason_cb, 10)
        self.create_subscription(
            String, '/recovery/state', self._recovery_cb, 10)
        self.create_subscription(
            String, '/mission/state', self._mission_cb, 10)
        self.create_subscription(
            String, '/exploration/state', self._exploration_cb, 10)
        self.create_subscription(
            String, '/localization/source', self._loc_source_cb, 10)
        self.create_subscription(
            Float32, '/localization/confidence', self._loc_conf_cb, 10)
        self.create_subscription(
            TrackedObstacleArray, '/obstacles/tracked',
            self._tracked_obs_cb, 5)
        self.create_subscription(
            Float32, '/obstacles/dynamic_risk', self._dynamic_risk_cb, 10)
        self.create_subscription(
            String, '/exploration/goal_status',
            self._goal_status_cb, 10)

        # --- Publishers ---
        self._event_pub = self.create_publisher(MissionEvent, '/autonomy/events', 10)
        self._summary_pub = self.create_publisher(String, '/autonomy/summary', 10)

        # Evaluate state changes at 1 Hz, summary at 2 Hz
        self.create_timer(1.0, self._evaluate)
        self.create_timer(0.5, self._publish_summary)

        self.get_logger().info('Mission intelligence initialized')

    # --- Callbacks ---

    def _autonomy_cb(self, msg: AutonomyMode) -> None:
        self._autonomy_mode = msg.mode

    def _safety_state_cb(self, msg: String) -> None:
        self._safety_state = msg.data

    def _safety_reason_cb(self, msg: String) -> None:
        self._safety_reason = msg.data

    def _recovery_cb(self, msg: String) -> None:
        self._recovery_state = msg.data

    def _mission_cb(self, msg: String) -> None:
        self._mission_state = msg.data

    def _exploration_cb(self, msg: String) -> None:
        self._exploration_state = msg.data

    def _loc_source_cb(self, msg: String) -> None:
        self._localization_source = msg.data

    def _loc_conf_cb(self, msg: Float32) -> None:
        self._localization_confidence = float(msg.data)

    def _tracked_obs_cb(self, msg: TrackedObstacleArray) -> None:
        self._dynamic_obs_count = sum(
            1 for o in msg.obstacles if o.is_dynamic)

    def _dynamic_risk_cb(self, msg: Float32) -> None:
        self._dynamic_risk = float(msg.data)

    def _goal_status_cb(self, msg: String) -> None:
        self._exploration_goal_status = msg.data

    # --- Event generation ---

    def _emit(
        self,
        severity: str,
        subsystem: str,
        description: str,
        cause: str,
        action: str,
        resolution: str = '',
    ) -> None:
        """Publish a structured mission event."""
        msg = MissionEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.severity = severity
        msg.subsystem = subsystem
        msg.description = description
        msg.cause = cause
        msg.action_taken = action
        msg.autonomy_mode = self._autonomy_mode or 'UNKNOWN'
        msg.resolution = resolution
        self._event_pub.publish(msg)
        self.get_logger().info(
            f'[{severity}] {subsystem}: {description}',
            throttle_duration_sec=1.0,
        )

    def _evaluate(self) -> None:
        """Detect state changes and emit mission events."""

        # -- Autonomy mode change --
        if (self._autonomy_mode is not None
                and self._autonomy_mode != self._prev_autonomy_mode):
            prev = self._prev_autonomy_mode or 'UNKNOWN'
            mode = self._autonomy_mode
            sev = 'INFO' if mode == 'FULL_AUTONOMY' else 'WARNING'
            descriptions = {
                'FULL_AUTONOMY': 'Autonomy restored to full capability.',
                'DEGRADED_PERCEPTION': 'Perception degraded; reducing speed.',
                'DEGRADED_LOCALIZATION': 'Localization degraded; limiting motion.',
                'DEGRADED_SENSOR': 'Sensor health degraded; safe hold if needed.',
                'RECOVERY': 'Recovery maneuver in progress.',
                'SAFE_HOLD': 'Safe hold: no safe motion available.',
                'RETURN_HOME': 'Returning home after degraded operation.',
            }
            self._emit(
                severity=sev,
                subsystem='AUTONOMY',
                description=descriptions.get(
                    mode, f'Autonomy mode changed to {mode}.'),
                cause=f'mode changed from {prev}',
                action=f'Operating in {mode} mode.',
            )
            self._prev_autonomy_mode = mode

        # -- Recovery state change --
        if (self._recovery_state is not None
                and self._recovery_state != self._prev_recovery_state):
            prev = self._prev_recovery_state or 'UNKNOWN'
            state = self._recovery_state
            descriptions = {
                'STUCK_SUSPECTED': 'Robot may be stuck; monitoring.',
                'RECOVERY_ACTIVE': 'Recovery initiated: robot executing maneuver.',
                'RECOVERY_SUCCESS': 'Recovery succeeded; navigation resumed.',
                'RECOVERY_FAILED': 'Recovery failed; attempting alternatives.',
                'STOPPED': 'Robot stopped after failed recovery.',
                'NORMAL': 'Robot operating normally.',
            }
            sev = 'RECOVERY' if 'RECOVERY' in state else 'INFO'
            if state in ('RECOVERY_FAILED', 'STOPPED'):
                sev = 'CRITICAL'
            self._emit(
                severity=sev,
                subsystem='RECOVERY',
                description=descriptions.get(state, f'Recovery state: {state}'),
                cause=f'state changed from {prev}',
                action='Recovery FSM executing.',
            )
            self._prev_recovery_state = state

        # -- Safety reason change (meaningful transitions) --
        if (self._safety_reason is not None
                and self._safety_reason != self._prev_safety_reason):
            reason = self._safety_reason
            if reason == 'SAFE_COMMAND':
                if self._prev_safety_reason not in (None, 'SAFE_COMMAND'):
                    self._emit(
                        severity='INFO',
                        subsystem='SAFETY',
                        description='Robot is moving safely.',
                        cause=f'safety reason: {reason}',
                        action='Normal navigation resumed.',
                    )
            elif reason in ('STALE_LIDAR', 'STALE_LOCALIZATION',
                            'LOW_LOCALIZATION_CONFIDENCE'):
                self._emit(
                    severity='WARNING',
                    subsystem='SAFETY',
                    description=f'Robot stopped: {reason.replace("_", " ").title()}.',
                    cause=reason,
                    action='Motion halted pending sensor recovery.',
                )
            elif reason == 'OBSTACLE_STOP':
                self._emit(
                    severity='WARNING',
                    subsystem='SAFETY',
                    description='Robot stopped: obstacle in path.',
                    cause=reason,
                    action='Waiting for path to clear.',
                )
            self._prev_safety_reason = reason

        # -- Dynamic obstacle appeared/disappeared --
        if self._dynamic_obs_count > 0 and self._prev_dynamic_obs_count == 0:
            self._emit(
                severity='WARNING',
                subsystem='OBSTACLE',
                description=(
                    f'Dynamic obstacle detected: '
                    f'{self._dynamic_obs_count} moving object(s) tracked.'),
                cause='obstacle tracker reports dynamic obstacle',
                action='Planner updating dynamic risk.',
            )
        elif self._dynamic_obs_count == 0 and self._prev_dynamic_obs_count > 0:
            self._emit(
                severity='INFO',
                subsystem='OBSTACLE',
                description='Dynamic obstacle cleared.',
                cause='obstacle tracker: no dynamic objects',
                action='Normal planning resumed.',
                resolution='Obstacle left field of view or stopped.',
            )
        self._prev_dynamic_obs_count = self._dynamic_obs_count

        # -- Mission state change --
        if (self._mission_state is not None
                and self._mission_state != self._prev_mission_state):
            descriptions = {
                'MAPPING': 'Exploration started; building map.',
                'RETURN_HOME': 'Mission complete; returning home.',
                'MISSION_COMPLETE': 'Mission successfully completed.',
                'FINAL_ALIGNMENT': 'Final alignment to home position.',
            }
            self._emit(
                severity='MISSION',
                subsystem='MISSION',
                description=descriptions.get(
                    self._mission_state,
                    f'Mission state: {self._mission_state}'),
                cause='mission manager state transition',
                action='Mission manager executing.',
            )
            self._prev_mission_state = self._mission_state

    def _publish_summary(self) -> None:
        """Publish a compact human-readable mission summary."""
        lines = []
        lines.append(f'Mode: {self._autonomy_mode or "UNKNOWN"}')
        lines.append(f'Mission: {self._mission_state or "UNKNOWN"}')
        lines.append(f'Safety: {self._safety_state or "UNKNOWN"}')
        if self._safety_reason and self._safety_reason != 'SAFE_COMMAND':
            lines.append(f'Stop reason: {self._safety_reason}')
        if self._localization_confidence is not None:
            lines.append(
                f'Localization: '
                f'{self._localization_source or "unknown"} '
                f'conf={self._localization_confidence:.2f}'
            )
        if self._dynamic_obs_count > 0:
            risk_str = (f'{self._dynamic_risk:.2f}'
                        if self._dynamic_risk is not None else '?')
            lines.append(
                f'Dynamic obstacles: {self._dynamic_obs_count} '
                f'(risk={risk_str})'
            )
        if self._recovery_state and self._recovery_state != 'NORMAL':
            lines.append(f'Recovery: {self._recovery_state}')
        msg = String()
        msg.data = ' | '.join(lines)
        self._summary_pub.publish(msg)


def main(args=None) -> None:
    """Run the mission intelligence node."""
    rclpy.init(args=args)
    node = MissionIntelligence()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
