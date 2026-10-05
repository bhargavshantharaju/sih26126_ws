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
EKF fusion health monitor for DRISTHI.

Monitors the staleness of EKF input sensors (IMU and wheel odometry) and the
EKF output (/odometry/filtered).  Publishes health flags that downstream
nodes (e.g., the safety supervisor) can consume.

Published topics:
  /fusion/imu_healthy        std_msgs/Bool
  /fusion/odom_healthy       std_msgs/Bool
  /fusion/ekf_healthy        std_msgs/Bool
  /fusion/state              std_msgs/String  (summary: HEALTHY / DEGRADED / FAILED)
"""

import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String


_STALE_SEC = 0.5   # consider a sensor stale after this many seconds


class FusionHealthMonitor(Node):
    """Track sensor staleness and publish fusion health status."""

    def __init__(self) -> None:
        """Initialise subscribers, publishers and the health-check timer."""
        super().__init__('fusion_health_monitor')

        self._imu_time: float | None = None
        self._odom_time: float | None = None
        self._ekf_time: float | None = None

        # Subscribers
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)
        self.create_subscription(
            Odometry, '/model/sih_ugv/odometry', self._odom_cb, 10
        )
        self.create_subscription(
            Odometry, '/odometry/filtered', self._ekf_cb, 10
        )

        # Publishers
        self._imu_pub = self.create_publisher(Bool, '/fusion/imu_healthy', 10)
        self._odom_pub = self.create_publisher(Bool, '/fusion/odom_healthy', 10)
        self._ekf_pub = self.create_publisher(Bool, '/fusion/ekf_healthy', 10)
        self._state_pub = self.create_publisher(String, '/fusion/state', 10)

        # Health-check timer at 5 Hz
        self.create_timer(0.2, self._publish_health)

        self.get_logger().info('Fusion health monitor started')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _imu_cb(self, _msg: Imu) -> None:
        self._imu_time = time.monotonic()

    def _odom_cb(self, _msg: Odometry) -> None:
        self._odom_time = time.monotonic()

    def _ekf_cb(self, _msg: Odometry) -> None:
        self._ekf_time = time.monotonic()

    # ------------------------------------------------------------------
    # Health publishing
    # ------------------------------------------------------------------

    def _age(self, t: float | None) -> float:
        if t is None:
            return float('inf')
        return time.monotonic() - t

    def _publish_health(self) -> None:
        imu_ok = self._age(self._imu_time) < _STALE_SEC
        odom_ok = self._age(self._odom_time) < _STALE_SEC
        ekf_ok = self._age(self._ekf_time) < _STALE_SEC

        self._imu_pub.publish(Bool(data=imu_ok))
        self._odom_pub.publish(Bool(data=odom_ok))
        self._ekf_pub.publish(Bool(data=ekf_ok))

        if imu_ok and odom_ok and ekf_ok:
            state = 'HEALTHY'
        elif ekf_ok and (imu_ok or odom_ok):
            state = 'DEGRADED'
        else:
            state = 'FAILED'

        self._state_pub.publish(String(data=state))

        self.get_logger().info(
            f'fusion state={state} imu={imu_ok} odom={odom_ok} ekf={ekf_ok}',
            throttle_duration_sec=5.0,
        )


def main(args=None) -> None:
    """Entry point for the fusion_health_monitor executable."""
    rclpy.init(args=args)
    node = FusionHealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
