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
DRISTHI degraded autonomy state machine.

Monitors real autonomy signals and transitions between operating modes:

FULL_AUTONOMY       -- all systems nominal
DEGRADED_PERCEPTION -- obstacle/terrain perception health low
DEGRADED_LOCALIZATION -- localization confidence low or stale
DEGRADED_SENSOR     -- LiDAR or other critical sensor stale/unhealthy
RECOVERY            -- recovery FSM is active
SAFE_HOLD           -- no safe motion possible; hold position
RETURN_HOME         -- returning home after degraded operation

Transitions have hysteresis to prevent rapid oscillation.
Publishes AutonomyMode to /autonomy/mode.
"""

import math
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sih26126_msgs.msg import AutonomyMode, SensorHealth
from std_msgs.msg import Float32, String


# Numeric priority per mode (lower = more degraded)
_MODE_PRIORITY = {
    'SAFE_HOLD': 0,
    'RECOVERY': 1,
    'DEGRADED_SENSOR': 2,
    'DEGRADED_LOCALIZATION': 3,
    'DEGRADED_PERCEPTION': 4,
    'RETURN_HOME': 5,
    'FULL_AUTONOMY': 6,
}


def _mode_is_degraded(mode: str) -> bool:
    return mode != 'FULL_AUTONOMY'


class DegradedAutonomyManager(Node):
    """Autonomy mode state machine with hysteresis."""

    def __init__(self) -> None:
        """Initialize autonomy manager with configurable thresholds."""
        super().__init__('degraded_autonomy_manager')

        # --- Thresholds ---
        self.declare_parameter('localization_confidence_min', 0.4)
        self.declare_parameter('localization_stale_sec', 1.5)
        self.declare_parameter('lidar_stale_sec', 1.5)
        self.declare_parameter('degradation_hysteresis_sec', 2.0)
        self.declare_parameter('recovery_hysteresis_sec', 5.0)
        self.declare_parameter('publish_hz', 2.0)

        self._loc_conf_min = float(
            self.get_parameter('localization_confidence_min').value)
        self._loc_stale = float(
            self.get_parameter('localization_stale_sec').value)
        self._lidar_stale = float(self.get_parameter('lidar_stale_sec').value)
        self._degrad_hyst = float(
            self.get_parameter('degradation_hysteresis_sec').value)
        self._recovery_hyst = float(
            self.get_parameter('recovery_hysteresis_sec').value)

        # Startup grace period: don't enter DEGRADED_SENSOR for missing LiDAR
        # health until this many seconds have elapsed. This prevents false
        # degradation while the LiDAR health monitor is still starting up.
        self._startup_time = time.monotonic()
        self._startup_grace_sec = 15.0

        # --- State ---
        self._current_mode = 'FULL_AUTONOMY'
        self._mode_entered_at = time.monotonic()
        self._candidate_mode: Optional[str] = None
        self._candidate_since: Optional[float] = None
        self._active_degradations: list = []

        # --- Sensor state ---
        self._loc_confidence: Optional[float] = None
        self._loc_pose_time: Optional[float] = None
        self._lidar_healthy: Optional[bool] = None
        self._lidar_stale_flag: Optional[bool] = None
        self._lidar_health_time: Optional[float] = None
        self._recovery_state: Optional[str] = None
        self._recovery_state_time: Optional[float] = None
        self._mission_state: Optional[str] = None

        # --- Subscriptions ---
        self.create_subscription(
            Float32, '/localization/confidence',
            self._loc_conf_cb, 10)
        self.create_subscription(
            PoseStamped, '/localization/pose',
            self._loc_pose_stamped_cb, 10)
        self.create_subscription(
            SensorHealth, '/sensor_health/lidar',
            self._lidar_health_cb, 10)
        self.create_subscription(
            String, '/recovery/state',
            self._recovery_state_cb, 10)
        self.create_subscription(
            String, '/mission/state',
            self._mission_cb, 10)

        # --- Publisher ---
        self._mode_pub = self.create_publisher(AutonomyMode, '/autonomy/mode', 10)
        self._cap_pub = self.create_publisher(Float32, '/autonomy/capability_score', 10)

        hz = float(self.get_parameter('publish_hz').value)
        self.create_timer(1.0 / max(1.0, hz), self._tick)

        self.get_logger().info('Degraded autonomy manager initialized')

    # --- Callbacks ---

    def _loc_conf_cb(self, msg: Float32) -> None:
        self._loc_confidence = float(msg.data)

    def _loc_pose_stamped_cb(self, _msg) -> None:
        self._loc_pose_time = time.monotonic()

    def _lidar_health_cb(self, msg: SensorHealth) -> None:
        self._lidar_healthy = bool(msg.healthy)
        self._lidar_stale_flag = bool(msg.stale)
        self._lidar_health_time = time.monotonic()

    def _recovery_state_cb(self, msg: String) -> None:
        self._recovery_state = msg.data
        self._recovery_state_time = time.monotonic()

    def _mission_cb(self, msg: String) -> None:
        self._mission_state = msg.data

    # --- Main tick ---

    def _tick(self) -> None:
        """Evaluate conditions and publish autonomy mode."""
        degradations: list = []
        target_mode = 'FULL_AUTONOMY'
        now = time.monotonic()

        # --- Check: recovery active ---
        if (self._recovery_state == 'RECOVERY_ACTIVE'
                and self._recovery_state_time is not None
                and (now - self._recovery_state_time) < 2.0):
            degradations.append('recovery_active')
            target_mode = _higher_priority(target_mode, 'RECOVERY')

        # --- Check: LiDAR stale or unhealthy ---
        if self._lidar_health_time is not None:
            lidar_age = now - self._lidar_health_time
            if lidar_age > self._lidar_stale:
                degradations.append('lidar_health_stale')
                target_mode = _higher_priority(target_mode, 'DEGRADED_SENSOR')
            elif self._lidar_stale_flag:
                degradations.append('lidar_data_stale')
                target_mode = _higher_priority(target_mode, 'DEGRADED_SENSOR')
            elif self._lidar_healthy is False:
                degradations.append('lidar_unhealthy')
                target_mode = _higher_priority(target_mode, 'DEGRADED_SENSOR')
        else:
            # Never received lidar health. Only degrade after the startup grace
            # period has elapsed — the health monitor may still be starting up.
            uptime = now - self._startup_time
            if uptime >= self._startup_grace_sec:
                degradations.append('lidar_no_data')
                target_mode = _higher_priority(target_mode, 'DEGRADED_SENSOR')

        # --- Check: localization confidence ---
        if self._loc_confidence is not None:
            if (not math.isfinite(self._loc_confidence)
                    or self._loc_confidence < self._loc_conf_min):
                degradations.append(
                    f'localization_confidence={self._loc_confidence:.2f}')
                target_mode = _higher_priority(
                    target_mode, 'DEGRADED_LOCALIZATION')
        if self._loc_pose_time is not None:
            loc_age = now - self._loc_pose_time
            if loc_age > self._loc_stale:
                degradations.append(f'localization_pose_age={loc_age:.1f}s')
                target_mode = _higher_priority(
                    target_mode, 'DEGRADED_LOCALIZATION')

        # --- Check: RETURN_HOME ---
        if self._mission_state in ('RETURN_HOME', 'FINAL_ALIGNMENT'):
            target_mode = _higher_priority(target_mode, 'RETURN_HOME')

        # --- Apply hysteresis ---
        new_mode = self._apply_hysteresis(target_mode, now)
        self._active_degradations = degradations

        if new_mode != self._current_mode:
            self.get_logger().info(
                f'autonomy_mode: {self._current_mode} -> {new_mode} '
                f'reason={degradations}'
            )
            self._current_mode = new_mode
            self._mode_entered_at = now

        self._publish(now)

    def _apply_hysteresis(self, target: str, now: float) -> str:
        """Apply hysteresis: require condition to persist before switching."""
        if target == self._current_mode:
            self._candidate_mode = None
            self._candidate_since = None
            return target

        # Switching to more degraded? Use degradation hysteresis.
        # Switching to less degraded? Use recovery hysteresis.
        target_pri = _MODE_PRIORITY.get(target, 0)
        current_pri = _MODE_PRIORITY.get(self._current_mode, 0)
        threshold = (self._degrad_hyst if target_pri <= current_pri
                     else self._recovery_hyst)

        if self._candidate_mode != target:
            self._candidate_mode = target
            self._candidate_since = now
            return self._current_mode

        elapsed = now - (self._candidate_since or now)
        if elapsed >= threshold:
            return target
        return self._current_mode

    def _publish(self, now: float) -> None:
        """Publish current autonomy mode and capability score."""
        msg = AutonomyMode()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = self._current_mode
        msg.reason = (', '.join(self._active_degradations)
                      if self._active_degradations else 'nominal')
        msg.is_degraded = _mode_is_degraded(self._current_mode)

        # Capability score: 1.0=full, lower=more degraded
        cap_scores = {
            'FULL_AUTONOMY': 1.0,
            'RETURN_HOME': 0.75,
            'DEGRADED_PERCEPTION': 0.6,
            'DEGRADED_LOCALIZATION': 0.45,
            'DEGRADED_SENSOR': 0.3,
            'RECOVERY': 0.15,
            'SAFE_HOLD': 0.0,
        }
        msg.capability_score = float(
            cap_scores.get(self._current_mode, 0.5))
        msg.active_degradations = list(self._active_degradations)
        self._mode_pub.publish(msg)

        cap_msg = Float32()
        cap_msg.data = msg.capability_score
        self._cap_pub.publish(cap_msg)


def _higher_priority(current: str, candidate: str) -> str:
    """Return the mode with lower priority value (more degraded)."""
    if _MODE_PRIORITY.get(candidate, 0) < _MODE_PRIORITY.get(current, 0):
        return candidate
    return current


def main(args=None) -> None:
    """Run the degraded autonomy manager."""
    rclpy.init(args=args)
    node = DegradedAutonomyManager()
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
