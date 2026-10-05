"""Safety supervisor for advisory planner commands."""

import math
import time
from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sih26126_msgs.msg import AutonomyMode
from std_msgs.msg import Float32, String


def finite(value: float) -> bool:
    """Return whether a numeric value is finite."""
    return math.isfinite(float(value))


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a numeric value to a closed interval."""
    return max(lower, min(upper, value))


def sanitize_command(
    command: Twist,
    max_linear: float = 0.30,
    max_angular: float = 0.60,
) -> Tuple[Twist, bool]:
    """Clamp a Twist and report whether its values are valid."""
    values = (
        command.linear.x,
        command.linear.y,
        command.linear.z,
        command.angular.x,
        command.angular.y,
        command.angular.z,
    )

    if not all(finite(value) for value in values):
        return Twist(), False

    output = Twist()

    output.linear.x = clamp(
        command.linear.x,
        -max_linear,
        max_linear,
    )
    output.linear.y = 0.0
    output.linear.z = 0.0

    output.angular.x = 0.0
    output.angular.y = 0.0
    output.angular.z = clamp(
        command.angular.z,
        -max_angular,
        max_angular,
    )

    return output, True


def safety_decision(
    navigation_state: Optional[str],
    obstacle_score: Optional[float],
    command: Twist,
    command_age: float,
    state_age: float,
    obstacle_age: float,
    localization_age: float = 0.0,
    localization_confidence: Optional[float] = 1.0,
    timeout: float = 0.75,
    obstacle_stop_threshold: float = 0.01,
    localization_confidence_threshold: float = 0.2,
) -> Tuple[Twist, str]:
    """Determine whether an advisory command may reach /cmd_vel."""
    zero = Twist()

    if navigation_state is None:
        return zero, 'WAITING_FOR_NAVIGATION_STATE'

    if state_age > timeout:
        return zero, 'STALE_NAVIGATION_STATE'

    if localization_age > timeout:
        return zero, 'STALE_LOCALIZATION'

    if (localization_confidence is None
            or not finite(localization_confidence)
            or localization_confidence < localization_confidence_threshold):
        return zero, 'LOW_LOCALIZATION_CONFIDENCE'

    if navigation_state == 'LOCAL_STOP_OBSTACLE':
        return zero, 'OBSTACLE_STOP'

    if navigation_state != 'LOCAL_NAVIGATE':
        return zero, 'NAVIGATION_NOT_ACTIVE'

    if command_age > timeout:
        return zero, 'STALE_COMMAND'

    if obstacle_score is None:
        return zero, 'WAITING_FOR_OBSTACLE_SCORE'

    if obstacle_age > timeout:
        return zero, 'STALE_OBSTACLE_SCORE'

    if not finite(obstacle_score):
        return zero, 'INVALID_OBSTACLE_SCORE'

    if obstacle_score <= obstacle_stop_threshold:
        return zero, 'OBSTACLE_STOP'

    safe_command, valid = sanitize_command(command)

    if not valid:
        return zero, 'INVALID_COMMAND'

    return safe_command, 'SAFE_COMMAND'


def recovery_safety_decision(
    recovery_suggestion: Twist,
    recovery_suggestion_age: float,
    obstacle_score: Optional[float],
    obstacle_age: float,
    localization_age: float,
    localization_confidence: Optional[float],
    timeout: float = 0.75,
    obstacle_stop_threshold: float = 0.01,
    localization_confidence_threshold: float = 0.2,
) -> Tuple[Twist, str]:
    """
    Evaluate a recovery suggestion through safety checks identical to planner commands.

    Called only when the recovery FSM is in RECOVERY_ACTIVE state.
    Navigation-state checks are omitted because the planner may be non-navigate
    precisely because the vehicle is stuck. All other protections are enforced.
    """
    zero = Twist()

    # --- staleness of recovery suggestion itself ---
    if recovery_suggestion_age > timeout:
        return zero, 'STALE_RECOVERY_SUGGESTION'

    # --- localization must be healthy ---
    if localization_age > timeout:
        return zero, 'STALE_LOCALIZATION'

    if (localization_confidence is None
            or not finite(localization_confidence)
            or localization_confidence < localization_confidence_threshold):
        return zero, 'LOW_LOCALIZATION_CONFIDENCE'

    # --- obstacle check still enforced ---
    if obstacle_score is None:
        return zero, 'WAITING_FOR_OBSTACLE_SCORE'

    if obstacle_age > timeout:
        return zero, 'STALE_OBSTACLE_SCORE'

    if not finite(obstacle_score):
        return zero, 'INVALID_OBSTACLE_SCORE'

    if obstacle_score <= obstacle_stop_threshold:
        return zero, 'OBSTACLE_STOP'

    # --- sanitize velocity (clamp + finite check) ---
    safe_command, valid = sanitize_command(recovery_suggestion)
    if not valid:
        return zero, 'INVALID_RECOVERY_COMMAND'

    return safe_command, 'RECOVERY_COMMAND'


class SafetySupervisor(Node):
    """Gate advisory planner commands before vehicle actuation."""

    def __init__(self) -> None:
        """Initialize the safety supervisor with subscriptions, publishers, and timer."""
        super().__init__('safety_supervisor')

        self.command: Optional[Twist] = None
        self.navigation_state: Optional[str] = None
        self.obstacle_score: Optional[float] = None
        self.localization_confidence: Optional[float] = None

        self.command_time = None
        self.state_time = None
        self.obstacle_time = None
        self.localization_pose_time = None

        # Recovery integration
        self.recovery_suggestion: Optional[Twist] = None
        self.recovery_state: Optional[str] = None
        self.recovery_suggestion_time: Optional[float] = None
        self.recovery_state_time: Optional[float] = None

        # Autonomy mode and dynamic obstacle risk integration
        self.autonomy_mode: str = 'FULL_AUTONOMY'
        self.autonomy_mode_time: Optional[float] = None
        self.dynamic_risk: float = 0.0
        self.dynamic_risk_time: Optional[float] = None

        self.declare_parameter('timeout_sec', 0.75)
        self.declare_parameter('obstacle_stop_threshold', 0.01)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 0.60)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('localization_confidence_threshold', 0.2)

        self.command_sub = self.create_subscription(
            Twist,
            '/planner/local_recommended_cmd_vel',
            self.command_callback,
            10,
        )

        self.state_sub = self.create_subscription(
            String,
            '/planner/local_navigation_state',
            self.state_callback,
            10,
        )

        self.obstacle_sub = self.create_subscription(
            Float32,
            '/planner/local_obstacle_score',
            self.obstacle_callback,
            10,
        )

        self.localization_sub = self.create_subscription(
            Float32,
            '/localization/confidence',
            self.localization_callback,
            10,
        )

        self.localization_pose_sub = self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.localization_pose_callback,
            10,
        )

        # Recovery subscriptions
        self.recovery_suggestion_sub = self.create_subscription(
            Twist,
            '/recovery/cmd_vel_suggestion',
            self.recovery_suggestion_callback,
            10,
        )

        self.recovery_state_sub = self.create_subscription(
            String,
            '/recovery/state',
            self.recovery_state_callback,
            10,
        )

        # Autonomy mode and dynamic obstacle risk subscriptions
        self.autonomy_mode_sub = self.create_subscription(
            AutonomyMode,
            '/autonomy/mode',
            self.autonomy_mode_callback,
            10,
        )

        self.dynamic_risk_sub = self.create_subscription(
            Float32,
            '/obstacles/dynamic_risk',
            self.dynamic_risk_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            '/safety/state',
            10,
        )

        self.reason_pub = self.create_publisher(
            String,
            '/safety/reason',
            10,
        )

        self.timer = self.create_timer(
            1.0 / max(
                1.0,
                float(self.get_parameter('publish_hz').value),
            ),
            self.process,
        )

    def command_callback(self, msg: Twist) -> None:
        """Store the latest advisory command."""
        self.command = msg
        self.command_time = time.monotonic()

    def state_callback(self, msg: String) -> None:
        """Store the latest local navigation state."""
        self.navigation_state = msg.data
        self.state_time = time.monotonic()

    def obstacle_callback(self, msg: Float32) -> None:
        """Store the latest obstacle score."""
        self.obstacle_score = float(msg.data)
        self.obstacle_time = time.monotonic()

    def localization_callback(self, msg: Float32) -> None:
        """Store the latest confidence value; pose freshness is tracked directly."""
        self.localization_confidence = float(msg.data)

    def localization_pose_callback(self, _msg: PoseStamped) -> None:
        """Track freshness of the actual localization pose used by navigation."""
        self.localization_pose_time = time.monotonic()

    def recovery_suggestion_callback(self, msg: Twist) -> None:
        """Store the latest recovery cmd_vel suggestion from stuck_recovery_node."""
        self.recovery_suggestion = msg
        self.recovery_suggestion_time = time.monotonic()

    def recovery_state_callback(self, msg: String) -> None:
        """Store the latest recovery FSM state from stuck_recovery_node."""
        self.recovery_state = msg.data
        self.recovery_state_time = time.monotonic()

    def autonomy_mode_callback(self, msg: AutonomyMode) -> None:
        """Store the latest autonomy mode from DegradedAutonomyManager."""
        self.autonomy_mode = msg.mode
        self.autonomy_mode_time = time.monotonic()

    def dynamic_risk_callback(self, msg: Float32) -> None:
        """Store the latest dynamic obstacle risk score."""
        self.dynamic_risk = max(0.0, min(1.0, float(msg.data)))
        self.dynamic_risk_time = time.monotonic()

    def age_seconds(self, timestamp) -> float:
        """Return age of a received message using monotonic time."""
        if timestamp is None:
            return float('inf')
        return max(0.0, time.monotonic() - timestamp)

    def publish_safety_status(self, state: str, reason: str) -> None:
        """Publish current safety state and reason."""
        state_msg = String()
        state_msg.data = state
        self.state_pub.publish(state_msg)

        reason_msg = String()
        reason_msg.data = reason
        self.reason_pub.publish(reason_msg)

    def _is_recovery_active(self) -> bool:
        """Return True only when the recovery FSM explicitly reports RECOVERY_ACTIVE."""
        if self.recovery_state != 'RECOVERY_ACTIVE':
            return False
        # Stale recovery state is treated as inactive — don't act on old data
        if self.age_seconds(self.recovery_state_time) > float(
            self.get_parameter('timeout_sec').value
        ):
            return False
        return True

    def process(self) -> None:
        """Evaluate and publish the current safe command."""
        timeout = float(
            self.get_parameter('timeout_sec').value
        )
        obstacle_threshold = float(
            self.get_parameter('obstacle_stop_threshold').value
        )
        max_linear = float(
            self.get_parameter('max_linear_speed').value
        )
        max_angular = float(
            self.get_parameter('max_angular_speed').value
        )
        confidence_threshold = float(
            self.get_parameter('localization_confidence_threshold').value
        )

        obstacle_age = self.age_seconds(self.obstacle_time)
        localization_age = self.age_seconds(self.localization_pose_time)

        # ----------------------------------------------------------------
        # Recovery path: only when recovery FSM is explicitly RECOVERY_ACTIVE
        # ----------------------------------------------------------------
        if self._is_recovery_active():
            recovery_cmd = self.recovery_suggestion if self.recovery_suggestion else Twist()
            recovery_suggestion_age = self.age_seconds(self.recovery_suggestion_time)

            safe_command, reason = recovery_safety_decision(
                recovery_suggestion=recovery_cmd,
                recovery_suggestion_age=recovery_suggestion_age,
                obstacle_score=self.obstacle_score,
                obstacle_age=obstacle_age,
                localization_age=localization_age,
                localization_confidence=self.localization_confidence,
                timeout=timeout,
                obstacle_stop_threshold=obstacle_threshold,
                localization_confidence_threshold=confidence_threshold,
            )

            if reason == 'RECOVERY_COMMAND':
                safe_command.linear.x = clamp(
                    safe_command.linear.x, -max_linear, max_linear
                )
                safe_command.angular.z = clamp(
                    safe_command.angular.z, -max_angular, max_angular
                )
                safety_state = 'RECOVERY'
            else:
                safe_command = Twist()
                safety_state = 'STOPPED'

            self.cmd_pub.publish(safe_command)
            self.publish_safety_status(safety_state, reason)
            self.get_logger().info(
                f'safety={safety_state} '
                f'reason={reason} '
                f'linear={safe_command.linear.x:.3f} '
                f'angular={safe_command.angular.z:.3f}',
                throttle_duration_sec=1.0,
            )
            return

        # ----------------------------------------------------------------
        # Normal path: planner command
        # ----------------------------------------------------------------
        if self.command is None:
            command = Twist()
        else:
            command = self.command

        command_age = self.age_seconds(self.command_time)
        state_age = self.age_seconds(self.state_time)

        # SAFE_HOLD: autonomy manager requests the vehicle to stop and hold
        if getattr(self, 'autonomy_mode', 'FULL_AUTONOMY') == 'SAFE_HOLD':
            self.cmd_pub.publish(Twist())
            self.publish_safety_status('STOPPED', 'SAFE_HOLD')
            self.get_logger().info(
                'safety=STOPPED reason=SAFE_HOLD (autonomy mode)',
                throttle_duration_sec=1.0,
            )
            return

        safe_command, reason = safety_decision(
            self.navigation_state,
            self.obstacle_score,
            command,
            command_age,
            state_age,
            obstacle_age,
            localization_age=localization_age,
            localization_confidence=self.localization_confidence,
            timeout=timeout,
            obstacle_stop_threshold=obstacle_threshold,
            localization_confidence_threshold=confidence_threshold,
        )

        # Apply configured limits one more time at the node boundary.
        safe_command.linear.x = clamp(
            safe_command.linear.x,
            -max_linear,
            max_linear,
        )
        safe_command.angular.z = clamp(
            safe_command.angular.z,
            -max_angular,
            max_angular,
        )

        if reason == 'SAFE_COMMAND':
            safety_state = 'SAFE'
            # Apply dynamic obstacle risk speed scaling
            dynamic_risk = getattr(self, 'dynamic_risk', 0.0)
            dynamic_risk_time = getattr(self, 'dynamic_risk_time', None)
            dynamic_risk_age = self.age_seconds(dynamic_risk_time)
            if dynamic_risk_age <= timeout and dynamic_risk > 0.0:
                if dynamic_risk > 0.9:
                    # High risk: stop completely
                    safe_command = Twist()
                    safety_state = 'STOPPED'
                    reason = 'BLOCKED_DYNAMIC_OBSTACLE'
                elif dynamic_risk > 0.7:
                    # Moderate-high risk: slow down proportionally
                    scale = max(0.0, 1.0 - dynamic_risk)
                    safe_command.linear.x *= scale
        else:
            safe_command = Twist()
            safety_state = 'STOPPED'

        self.cmd_pub.publish(safe_command)
        self.publish_safety_status(
            safety_state,
            reason,
        )

        self.get_logger().info(
            f'safety={safety_state} '
            f'reason={reason} '
            f'linear={safe_command.linear.x:.3f} '
            f'angular={safe_command.angular.z:.3f} '
            f'dyn_risk={getattr(self, "dynamic_risk", 0.0):.2f}',
            throttle_duration_sec=10.0,
        )


def main(args=None) -> None:
    """Run the safety supervisor."""
    rclpy.init(args=args)
    node = SafetySupervisor()

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
