"""
Stuck detection and safe recovery node for the DRISTHI UGV autonomy stack.

This node monitors commanded vs measured speed to detect when the vehicle is
stuck, then executes a recovery sequence (stop -> reverse -> rotate -> stop).
It publishes advisory suggestions to /recovery/cmd_vel_suggestion; the
safety_supervisor remains the sole authoritative /cmd_vel publisher.
"""

from enum import Enum
import math
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String


class RecoveryState(str, Enum):
    """Finite state machine states for stuck detection and recovery."""

    NORMAL = 'NORMAL'
    STUCK_SUSPECTED = 'STUCK_SUSPECTED'
    RECOVERY_ACTIVE = 'RECOVERY_ACTIVE'
    RECOVERY_SUCCESS = 'RECOVERY_SUCCESS'
    RECOVERY_FAILED = 'RECOVERY_FAILED'
    STOPPED = 'STOPPED'


class RecoveryPhase(str, Enum):
    """Sub-phases of the RECOVERY_ACTIVE state."""

    STOP_INITIAL = 'STOP_INITIAL'
    REVERSE = 'REVERSE'
    ROTATE = 'ROTATE'
    STOP_FINAL = 'STOP_FINAL'
    DONE = 'DONE'


class StuckRecoveryNode(Node):
    """Detect vehicle stuck condition and execute a safe recovery sequence."""

    def __init__(self) -> None:
        """Initialize the stuck recovery node with subscriptions, publishers, and timer."""
        super().__init__('stuck_recovery_node')

        # ------------------------------------------------------------------ parameters
        self.declare_parameter('stuck_command_speed_threshold', 0.05)
        self.declare_parameter('stuck_measured_speed_threshold', 0.02)
        self.declare_parameter('stuck_timeout_sec', 3.0)
        self.declare_parameter('stuck_suspect_timeout_sec', 2.0)
        self.declare_parameter('recovery_reverse_speed', 0.10)
        self.declare_parameter('recovery_reverse_time_sec', 2.0)
        self.declare_parameter('recovery_rotate_speed', 0.3)
        self.declare_parameter('recovery_rotate_time_sec', 2.0)
        self.declare_parameter('recovery_success_distance', 0.05)
        self.declare_parameter('recovery_timeout_sec', 10.0)
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('min_clearance_for_reverse', 0.3)

        # ------------------------------------------------------------------ state
        self._fsm_state: RecoveryState = RecoveryState.NORMAL
        self._recovery_phase: RecoveryPhase = RecoveryPhase.STOP_INITIAL

        # Latest sensor values
        self._measured_speed: Optional[float] = None        # m/s (magnitude)
        self._commanded_speed: Optional[float] = None       # m/s (magnitude)
        self._safety_state: Optional[str] = None
        self._localization_confidence: Optional[float] = None
        self._obstacle_clearance: Optional[float] = None

        # Message receipt timestamps (monotonic)
        self._odom_time: Optional[float] = None
        self._cmd_time: Optional[float] = None
        self._safety_time: Optional[float] = None
        self._confidence_time: Optional[float] = None
        self._clearance_time: Optional[float] = None

        # Robot position for displacement tracking (from odometry)
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0
        self._recovery_start_pos_x: float = 0.0
        self._recovery_start_pos_y: float = 0.0

        # Timing for stuck detection
        self._stuck_condition_start: Optional[float] = None  # when condition first triggered
        self._suspect_entry_time: Optional[float] = None     # when STUCK_SUSPECTED entered

        # Timing for recovery phases
        self._phase_start_time: Optional[float] = None
        self._recovery_start_time: Optional[float] = None

        # Attempt counter
        self._recovery_attempts: int = 0

        # Previous FSM state for transition logging
        self._prev_state: RecoveryState = RecoveryState.NORMAL

        # ------------------------------------------------------------------ subscriptions
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self._odom_callback,
            10,
        )
        self._cmd_sub = self.create_subscription(
            Twist,
            '/planner/local_recommended_cmd_vel',
            self._cmd_callback,
            10,
        )
        self._safety_sub = self.create_subscription(
            String,
            '/safety/state',
            self._safety_callback,
            10,
        )
        self._confidence_sub = self.create_subscription(
            Float32,
            '/localization/confidence',
            self._confidence_callback,
            10,
        )
        self._clearance_sub = self.create_subscription(
            Float32,
            '/planner/local_obstacle_clearance',
            self._clearance_callback,
            10,
        )

        # ------------------------------------------------------------------ publishers
        self._state_pub = self.create_publisher(String, '/recovery/state', 10)
        self._cmd_vel_suggestion_pub = self.create_publisher(
            Twist, '/recovery/cmd_vel_suggestion', 10
        )
        self._reason_pub = self.create_publisher(String, '/recovery/reason', 10)

        # ------------------------------------------------------------------ timer
        self._timer = self.create_timer(0.1, self._update)  # 10 Hz

        self.get_logger().info('stuck_recovery_node started — monitoring for stuck condition.')

    # ------------------------------------------------------------------ callbacks

    def _odom_callback(self, msg: Odometry) -> None:
        """Extract measured speed magnitude and position from filtered odometry."""
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._measured_speed = math.sqrt(vx * vx + vy * vy)
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y
        self._odom_time = time.monotonic()

    def _cmd_callback(self, msg: Twist) -> None:
        """Extract commanded speed magnitude from planner recommendation."""
        vx = msg.linear.x
        vy = msg.linear.y
        self._commanded_speed = math.sqrt(vx * vx + vy * vy)
        self._cmd_time = time.monotonic()

    def _safety_callback(self, msg: String) -> None:
        """Store the latest safety supervisor state."""
        self._safety_state = msg.data
        self._safety_time = time.monotonic()

    def _confidence_callback(self, msg: Float32) -> None:
        """Store the latest localization confidence."""
        if math.isfinite(float(msg.data)):
            self._localization_confidence = float(msg.data)
        self._confidence_time = time.monotonic()

    def _clearance_callback(self, msg: Float32) -> None:
        """Store the latest obstacle clearance distance."""
        if math.isfinite(float(msg.data)):
            self._obstacle_clearance = float(msg.data)
        self._clearance_time = time.monotonic()

    # ------------------------------------------------------------------ helpers

    def _age(self, timestamp: Optional[float]) -> float:
        """Return age in seconds of a monotonic timestamp, or inf if None."""
        if timestamp is None:
            return float('inf')
        return max(0.0, time.monotonic() - timestamp)

    def _displacement_from_recovery_start(self) -> float:
        """Euclidean distance moved since recovery began."""
        dx = self._pos_x - self._recovery_start_pos_x
        dy = self._pos_y - self._recovery_start_pos_y
        return math.sqrt(dx * dx + dy * dy)

    def _publish_zero_cmd(self) -> None:
        """Publish a zero-velocity suggestion."""
        self._cmd_vel_suggestion_pub.publish(Twist())

    def _publish_reverse_cmd(self) -> None:
        """Publish the reverse velocity suggestion."""
        speed = float(self.get_parameter('recovery_reverse_speed').value)
        msg = Twist()
        msg.linear.x = -abs(speed)
        self._cmd_vel_suggestion_pub.publish(msg)

    def _publish_rotate_cmd(self) -> None:
        """Publish the rotate velocity suggestion."""
        speed = float(self.get_parameter('recovery_rotate_speed').value)
        msg = Twist()
        msg.angular.z = speed
        self._cmd_vel_suggestion_pub.publish(msg)

    def _publish_state_and_reason(self, reason: str) -> None:
        """Publish the current FSM state and reason strings."""
        state_msg = String()
        state_msg.data = self._fsm_state.value
        self._state_pub.publish(state_msg)

        reason_msg = String()
        reason_msg.data = reason
        self._reason_pub.publish(reason_msg)

    def _transition(self, new_state: RecoveryState, reason: str) -> None:
        """Perform a state transition with INFO-level logging."""
        if new_state != self._fsm_state:
            self.get_logger().info(
                f'[StuckRecovery] {self._fsm_state.value} -> {new_state.value} | {reason}'
            )
        self._fsm_state = new_state
        self._publish_state_and_reason(reason)

    def _guard_conditions_block_recovery(self) -> Optional[str]:
        """
        Return a non-empty reason string if guard conditions prevent recovery.

        Returns None when recovery is allowed.
        """
        # Guard 1: localization confidence too low
        if (self._localization_confidence is None
                or self._localization_confidence < 0.1):
            return 'LOW_LOCALIZATION_CONFIDENCE'

        # Guard 2: safety supervisor has stopped the system
        if self._safety_state in ('STOPPED', 'E_STOP'):
            return f'SAFETY_SUPERVISOR_STATE_{self._safety_state}'

        # Guard 3: no planner command received yet
        if self._cmd_time is None:
            return 'NO_PLANNER_COMMAND'

        # Guard 4: commanded speed is effectively zero (not supposed to move)
        cmd_threshold = float(
            self.get_parameter('stuck_command_speed_threshold').value
        )
        if (self._commanded_speed is None
                or self._commanded_speed < cmd_threshold):
            return 'COMMANDED_SPEED_ZERO'

        return None

    def _is_stuck_condition_active(self) -> bool:
        """Return True when commanded speed is high but measured speed is near zero."""
        if self._commanded_speed is None or self._measured_speed is None:
            return False

        cmd_threshold = float(
            self.get_parameter('stuck_command_speed_threshold').value
        )
        meas_threshold = float(
            self.get_parameter('stuck_measured_speed_threshold').value
        )

        return (self._commanded_speed > cmd_threshold
                and self._measured_speed < meas_threshold)

    # ------------------------------------------------------------------ FSM update

    def _update(self) -> None:
        """Tick the FSM at 10 Hz — dispatch to the current state handler."""
        now = time.monotonic()

        state = self._fsm_state

        if state == RecoveryState.NORMAL:
            self._handle_normal(now)

        elif state == RecoveryState.STUCK_SUSPECTED:
            self._handle_stuck_suspected(now)

        elif state == RecoveryState.RECOVERY_ACTIVE:
            self._handle_recovery_active(now)

        elif state == RecoveryState.RECOVERY_SUCCESS:
            self._handle_recovery_success()

        elif state == RecoveryState.RECOVERY_FAILED:
            self._handle_recovery_failed(now)

        elif state == RecoveryState.STOPPED:
            self._handle_stopped()

    # ------------------------------------------------------------------ state handlers

    def _handle_normal(self, now: float) -> None:
        """Handle NORMAL state — monitor for stuck condition onset."""
        if self._is_stuck_condition_active():
            if self._stuck_condition_start is None:
                self._stuck_condition_start = now
            else:
                stuck_timeout = float(
                    self.get_parameter('stuck_timeout_sec').value
                )
                elapsed = now - self._stuck_condition_start
                if elapsed > stuck_timeout:
                    # Check guards before suspecting
                    block_reason = self._guard_conditions_block_recovery()
                    if block_reason:
                        self._stuck_condition_start = None  # reset timer
                        self._publish_state_and_reason(
                            f'NORMAL_GUARD_BLOCKED:{block_reason}'
                        )
                        return
                    self._suspect_entry_time = now
                    self._transition(
                        RecoveryState.STUCK_SUSPECTED,
                        f'Commanded {self._commanded_speed:.3f} m/s, '
                        f'measured {self._measured_speed:.3f} m/s for '
                        f'{elapsed:.1f}s',
                    )
                    return
        else:
            # Clear the stuck timer if condition clears
            if self._stuck_condition_start is not None:
                self._stuck_condition_start = None

        self._publish_state_and_reason('Monitoring')

    def _handle_stuck_suspected(self, now: float) -> None:
        """STUCK_SUSPECTED: hysteresis window before committing to recovery."""
        # If vehicle starts moving, return to NORMAL
        if not self._is_stuck_condition_active():
            self._stuck_condition_start = None
            self._transition(RecoveryState.NORMAL, 'Vehicle resumed motion')
            return

        # Re-check guards — if something changed that blocks recovery, return to NORMAL
        block_reason = self._guard_conditions_block_recovery()
        if block_reason:
            self._stuck_condition_start = None
            self._transition(RecoveryState.NORMAL, f'Guard blocked: {block_reason}')
            return

        # After the suspect hysteresis window, escalate to active recovery
        suspect_timeout = float(
            self.get_parameter('stuck_suspect_timeout_sec').value
        )
        if self._suspect_entry_time is not None:
            elapsed = now - self._suspect_entry_time
            if elapsed > suspect_timeout:
                self._start_recovery(now)
                return

        self._publish_state_and_reason(
            f'Suspected stuck — confirming ({self._measured_speed:.3f} m/s measured)'
        )

    def _start_recovery(self, now: float) -> None:
        """Initiate the recovery sequence."""
        self._recovery_attempts += 1
        self._recovery_start_time = now
        self._recovery_start_pos_x = self._pos_x
        self._recovery_start_pos_y = self._pos_y
        self._recovery_phase = RecoveryPhase.STOP_INITIAL
        self._phase_start_time = now
        self._stuck_condition_start = None
        self._suspect_entry_time = None

        self._transition(
            RecoveryState.RECOVERY_ACTIVE,
            f'Attempt {self._recovery_attempts} — beginning stop/reverse/rotate sequence',
        )

    def _handle_recovery_active(self, now: float) -> None:
        """RECOVERY_ACTIVE: execute the stop → reverse → rotate → stop sequence."""
        max_recovery_attempts = int(
            self.get_parameter('max_recovery_attempts').value
        )
        recovery_timeout = float(self.get_parameter('recovery_timeout_sec').value)
        reverse_time = float(self.get_parameter('recovery_reverse_time_sec').value)
        rotate_time = float(self.get_parameter('recovery_rotate_time_sec').value)
        min_clearance = float(self.get_parameter('min_clearance_for_reverse').value)
        success_distance = float(self.get_parameter('recovery_success_distance').value)

        phase_elapsed = (
            (now - self._phase_start_time) if self._phase_start_time is not None else 0.0
        )
        total_elapsed = (
            (now - self._recovery_start_time) if self._recovery_start_time is not None else 0.0
        )

        # Overall recovery timeout guard
        if total_elapsed > recovery_timeout:
            self._publish_zero_cmd()
            if self._recovery_attempts >= max_recovery_attempts:
                self._transition(
                    RecoveryState.RECOVERY_FAILED,
                    f'Recovery timeout after {total_elapsed:.1f}s — max attempts reached',
                )
            else:
                self._transition(
                    RecoveryState.RECOVERY_FAILED,
                    f'Recovery timeout after {total_elapsed:.1f}s',
                )
            return

        # ---- Phase 1: STOP_INITIAL (0.5 s) ----
        if self._recovery_phase == RecoveryPhase.STOP_INITIAL:
            self._publish_zero_cmd()
            if phase_elapsed >= 0.5:
                self._recovery_phase = RecoveryPhase.REVERSE
                self._phase_start_time = now
                self.get_logger().info('[StuckRecovery] Phase: REVERSE')
            else:
                self._publish_state_and_reason('Recovery phase: initial stop')
            return

        # ---- Phase 2: REVERSE ----
        if self._recovery_phase == RecoveryPhase.REVERSE:
            # Abort reverse if obstacle clearance too small
            clearance_ok = (
                self._obstacle_clearance is None
                or self._obstacle_clearance >= min_clearance
            )
            # Note: clearance is for forward obstacles; reversing checks rear clearance.
            # We treat a low clearance reading conservatively and skip reverse.
            if not clearance_ok:
                self.get_logger().info(
                    f'[StuckRecovery] Reverse aborted — clearance '
                    f'{self._obstacle_clearance:.2f}m < {min_clearance:.2f}m; skipping to ROTATE'
                )
                self._recovery_phase = RecoveryPhase.ROTATE
                self._phase_start_time = now
                self._publish_zero_cmd()
                return

            # Check displacement success during reverse
            displacement = self._displacement_from_recovery_start()
            if displacement >= success_distance:
                self._publish_zero_cmd()
                self._transition(
                    RecoveryState.RECOVERY_SUCCESS,
                    f'Displaced {displacement:.3f}m during reverse — recovery successful',
                )
                return

            if phase_elapsed >= reverse_time:
                self._recovery_phase = RecoveryPhase.ROTATE
                self._phase_start_time = now
                self.get_logger().info('[StuckRecovery] Phase: ROTATE')
                self._publish_zero_cmd()
            else:
                self._publish_reverse_cmd()
                self._publish_state_and_reason(
                    f'Recovery phase: reverse ({phase_elapsed:.1f}s / {reverse_time:.1f}s)'
                )
            return

        # ---- Phase 3: ROTATE ----
        if self._recovery_phase == RecoveryPhase.ROTATE:
            # Check displacement success during rotate
            displacement = self._displacement_from_recovery_start()
            if displacement >= success_distance:
                self._publish_zero_cmd()
                self._transition(
                    RecoveryState.RECOVERY_SUCCESS,
                    f'Displaced {displacement:.3f}m during rotate — recovery successful',
                )
                return

            if phase_elapsed >= rotate_time:
                self._recovery_phase = RecoveryPhase.STOP_FINAL
                self._phase_start_time = now
                self.get_logger().info('[StuckRecovery] Phase: STOP_FINAL')
                self._publish_zero_cmd()
            else:
                self._publish_rotate_cmd()
                self._publish_state_and_reason(
                    f'Recovery phase: rotate ({phase_elapsed:.1f}s / {rotate_time:.1f}s)'
                )
            return

        # ---- Phase 4: STOP_FINAL ----
        if self._recovery_phase == RecoveryPhase.STOP_FINAL:
            self._publish_zero_cmd()
            # Give a brief pause (0.3 s) then evaluate outcome
            if phase_elapsed >= 0.3:
                displacement = self._displacement_from_recovery_start()
                if displacement >= success_distance:
                    self._transition(
                        RecoveryState.RECOVERY_SUCCESS,
                        f'Displaced {displacement:.3f}m — recovery successful',
                    )
                else:
                    if self._recovery_attempts >= max_recovery_attempts:
                        self._transition(
                            RecoveryState.RECOVERY_FAILED,
                            f'No displacement after {self._recovery_attempts} attempts — STOPPED',
                        )
                    else:
                        self._transition(
                            RecoveryState.RECOVERY_FAILED,
                            f'Attempt {self._recovery_attempts} failed — will retry',
                        )
            else:
                self._publish_state_and_reason('Recovery phase: final stop')
            return

    def _handle_recovery_success(self) -> None:
        """RECOVERY_SUCCESS: brief acknowledgment then return to NORMAL."""
        self._recovery_attempts = 0
        self._stuck_condition_start = None
        self._suspect_entry_time = None
        self._publish_zero_cmd()
        self._transition(RecoveryState.NORMAL, 'Recovery succeeded — resuming normal operation')

    def _handle_recovery_failed(self, now: float) -> None:
        """RECOVERY_FAILED: retry if attempts remain, otherwise STOPPED."""
        max_attempts = int(self.get_parameter('max_recovery_attempts').value)

        if self._recovery_attempts < max_attempts:
            # Schedule another attempt via STUCK_SUSPECTED
            self._suspect_entry_time = now
            self._transition(
                RecoveryState.STUCK_SUSPECTED,
                f'Retry {self._recovery_attempts + 1}/{max_attempts}',
            )
        else:
            self._transition(
                RecoveryState.STOPPED,
                f'Max recovery attempts ({max_attempts}) exhausted — terminal stop',
            )

    def _handle_stopped(self) -> None:
        """Handle STOPPED terminal state — publish zero cmd and announce stop."""
        self._publish_zero_cmd()
        self._publish_state_and_reason(
            'Terminal stop — max recovery attempts exhausted'
        )


def main(args=None) -> None:
    """Run the stuck recovery node."""
    rclpy.init(args=args)
    node = StuckRecoveryNode()

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
