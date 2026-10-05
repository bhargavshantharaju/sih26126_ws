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
Deterministic auto-activator for the slam_toolbox lifecycle node.

Polls the slam_toolbox get_state service with configurable retries instead of
relying on bare time.sleep() races.  The node exits cleanly after activation
(or after all retry attempts are exhausted).

The activation sequence runs in a background thread so that the ROS executor
continues spinning (enabling service calls to complete) without re-entrant
spin_until_future_complete deadlocks.

Lifecycle state IDs as per lifecycle_msgs:
    1 = unconfigured
    2 = inactive
    3 = active
    4 = finalized

Transition IDs:
    1 = configure  (unconfigured -> inactive)
    3 = activate   (inactive -> active)
"""

import threading
import time

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

_STATE_UNCONFIGURED = 1
_STATE_INACTIVE = 2
_STATE_ACTIVE = 3

_TRANSITION_CONFIGURE = Transition.TRANSITION_CONFIGURE
_TRANSITION_ACTIVATE = Transition.TRANSITION_ACTIVATE


class SlamActivator(Node):
    """Automatically configure and activate the slam_toolbox lifecycle node."""

    def __init__(self) -> None:
        """Initialise the activator node."""
        super().__init__('slam_activator')

        self.declare_parameter('service_timeout_sec', 30.0)
        self.declare_parameter('call_timeout_sec', 10.0)
        self.declare_parameter('retry_interval_sec', 1.0)
        self.declare_parameter('max_retries', 10)

        self._service_timeout = float(
            self.get_parameter('service_timeout_sec').value
        )
        self._call_timeout = float(
            self.get_parameter('call_timeout_sec').value
        )
        self._retry_interval = float(
            self.get_parameter('retry_interval_sec').value
        )
        self._max_retries = int(self.get_parameter('max_retries').value)

        self.get_logger().info(
            'SLAM activator starting '
            f'(service_timeout={self._service_timeout}s, '
            f'max_retries={self._max_retries})'
        )

        self._get_state_client = self.create_client(
            GetState, '/slam_toolbox/get_state'
        )
        self._change_state_client = self.create_client(
            ChangeState, '/slam_toolbox/change_state'
        )

        # Run the activation sequence in a daemon thread so the executor
        # keeps spinning and service futures can complete.
        self._activation_thread = threading.Thread(
            target=self._startup_sequence,
            daemon=True,
            name='slam_activator_thread',
        )
        self._activation_thread.start()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _wait_for_services(self) -> bool:
        """Return True when both lifecycle services are available."""
        if not self._get_state_client.wait_for_service(
            timeout_sec=self._service_timeout
        ):
            self.get_logger().error(
                'slam_toolbox/get_state service not available after '
                f'{self._service_timeout}s'
            )
            return False

        if not self._change_state_client.wait_for_service(
            timeout_sec=self._service_timeout
        ):
            self.get_logger().error(
                'slam_toolbox/change_state service not available after '
                f'{self._service_timeout}s'
            )
            return False

        self.get_logger().info('slam_toolbox lifecycle services are available')
        return True

    def _call_with_timeout(self, client, request, timeout_sec: float):
        """
        Submit a service call and block until done or timeout.

        Returns the result or None on timeout/failure.
        This is safe to call from a non-executor thread.
        """
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        if future.exception() is not None:
            return None
        return future.result()

    def _get_state(self):
        """Return (state_id, state_label) or (None, None) on failure."""
        result = self._call_with_timeout(
            self._get_state_client,
            GetState.Request(),
            self._call_timeout,
        )
        if result is not None:
            state = result.current_state
            return state.id, state.label

        self.get_logger().warn('GetState call timed out or returned no result')
        return None, None

    def _change_state(self, transition_id: int, label: str) -> bool:
        """Request a lifecycle transition; return True on success."""
        req = ChangeState.Request()
        req.transition.id = transition_id

        result = self._call_with_timeout(
            self._change_state_client,
            req,
            self._call_timeout,
        )
        if result is not None:
            success = result.success
            if success:
                self.get_logger().info(f'Transition "{label}" succeeded')
            else:
                self.get_logger().error(
                    f'Transition "{label}" was rejected by slam_toolbox'
                )
            return success

        self.get_logger().warn(f'ChangeState call for "{label}" timed out')
        return False

    def _poll_for_state(self, target_id: int, target_label: str) -> bool:
        """Poll until slam_toolbox reaches *target_id* or retries exhausted."""
        for attempt in range(1, self._max_retries + 1):
            state_id, state_label = self._get_state()
            if state_id == target_id:
                self.get_logger().info(
                    f'slam_toolbox reached state {target_label} '
                    f'(attempt {attempt}/{self._max_retries})'
                )
                return True

            self.get_logger().info(
                f'Waiting for state {target_label} '
                f'(current={state_label}, attempt {attempt}/{self._max_retries})'
            )
            time.sleep(self._retry_interval)

        self.get_logger().error(
            f'slam_toolbox did not reach {target_label} after '
            f'{self._max_retries} attempts'
        )
        return False

    # ------------------------------------------------------------------
    # Main activation sequence (runs in background thread)
    # ------------------------------------------------------------------

    def _startup_sequence(self) -> None:
        """Run the full configure->activate sequence from the worker thread."""
        if not self._wait_for_services():
            self.get_logger().error(
                'Cannot activate slam_toolbox: services unavailable'
            )
            return

        # Read the initial state.
        state_id, state_label = self._get_state()
        if state_id is None:
            self.get_logger().error('Failed to read slam_toolbox initial state')
            return

        self.get_logger().info(
            f'slam_toolbox initial state: {state_label} (id={state_id})'
        )

        # Step 1: configure if needed.
        if state_id == _STATE_UNCONFIGURED:
            self.get_logger().info('Configuring slam_toolbox...')
            if not self._change_state(_TRANSITION_CONFIGURE, 'configure'):
                return
            if not self._poll_for_state(_STATE_INACTIVE, 'inactive'):
                return
            state_id = _STATE_INACTIVE

        # Step 2: activate if needed.
        if state_id == _STATE_INACTIVE:
            self.get_logger().info('Activating slam_toolbox...')
            if not self._change_state(_TRANSITION_ACTIVATE, 'activate'):
                return
            if not self._poll_for_state(_STATE_ACTIVE, 'active'):
                return
            state_id = _STATE_ACTIVE

        if state_id == _STATE_ACTIVE:
            self.get_logger().info(
                'slam_toolbox is ACTIVE — SLAM is running and processing scans'
            )
        else:
            self.get_logger().warn(
                f'Unexpected final slam_toolbox state: {state_label} (id={state_id})'
            )


def main(args=None) -> None:
    """Entry point for the slam_activator executable."""
    rclpy.init(args=args)
    node = SlamActivator()

    # Use a MultiThreadedExecutor so service call futures complete while the
    # background activation thread is also running service calls.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
