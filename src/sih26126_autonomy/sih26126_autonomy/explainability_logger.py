#!/usr/bin/env python3
"""
Explainability Enhancements - Rejected path logging and recovery outcomes.

Extends existing mission_intelligence and dashboard with:
- Rejected global path reason tracking
- Recovery attempt outcome logging
- Detailed decision rationale
"""

from typing import Optional, List
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import String
from sih26126_msgs.msg import MissionEvent


class RejectionReason(Enum):
    """Reasons for rejecting a planned path."""
    UNREACHABLE = "UNREACHABLE"
    HIGH_COST = "HIGH_COST"
    OBSTACLE_BLOCKED = "OBSTACLE_BLOCKED"
    TERRAIN_UNSAFE = "TERRAIN_UNSAFE"
    LOCALIZATION_POOR = "LOCALIZATION_POOR"
    TIMEOUT = "TIMEOUT"
    NONE = "NONE"


class RecoveryOutcome(Enum):
    """Outcomes of recovery attempts."""
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"
    IN_PROGRESS = "IN_PROGRESS"
    BLOCKED = "BLOCKED"
    TIMEOUT = "TIMEOUT"


@dataclass
class RejectedPath:
    """Record of a rejected path with reason."""
    timestamp: float
    path_length: float
    rejection_reason: RejectionReason
    cost_value: Optional[float] = None
    alternative_found: bool = False
    description: str = ""


@dataclass
class RecoveryAttempt:
    """Record of a recovery attempt with outcome."""
    timestamp: float
    recovery_phase: str
    outcome: RecoveryOutcome
    duration_sec: float
    displacement_m: float
    attempts_count: int
    final_state: str
    description: str = ""


class ExplainabilityLogger(Node):
    """Enhanced logging for navigation decisions and recovery outcomes."""
    
    def __init__(self):
        super().__init__('explainability_logger')
        
        # State
        self.rejected_paths: List[RejectedPath] = []
        self.recovery_attempts: List[RecoveryAttempt] = []
        self.last_path: Optional[NavPath] = None
        self.last_recovery_state: Optional[str] = None
        self.recovery_start_time: Optional[float] = None
        
        # Subscribers
        self.create_subscription(
            NavPath,
            '/planner/global_path',
            self.global_path_callback,
            10
        )
        self.create_subscription(
            String,
            '/planner/state',
            self.planner_state_callback,
            10
        )
        self.create_subscription(
            String,
            '/recovery/state',
            self.recovery_state_callback,
            10
        )
        self.create_subscription(
            String,
            '/safety/stop_reason',
            self.stop_reason_callback,
            10
        )
        
        # Publishers
        # Single publisher for /mission/events — both path rejections and
        # recovery outcomes route through this one publisher to avoid two
        # publishers on the same topic from the same node.
        self.mission_event_pub = self.create_publisher(
            MissionEvent,
            '/mission/events',
            10
        )
        self.decision_rationale_pub = self.create_publisher(
            String,
            '/mission/decision_rationale',
            10
        )
        
        # Timer
        self.create_timer(1.0, self.process)
        
        self.get_logger().info('Explainability logger started')
    
    def global_path_callback(self, msg: NavPath):
        """Track global path updates."""
        self.last_path = msg
    
    def planner_state_callback(self, msg: String):
        """Detect path rejections from planner state."""
        state = msg.data
        
        # Detect replanning due to rejection
        if "REPLAN" in state.upper() or "BLOCKED" in state.upper():
            self.log_path_rejection(
                reason=RejectionReason.OBSTACLE_BLOCKED,
                description=f"Planner state: {state}"
            )
    
    def stop_reason_callback(self, msg: String):
        """Detect path rejections from safety stops."""
        reason = msg.data
        
        if "OBSTACLE" in reason.upper():
            self.log_path_rejection(
                reason=RejectionReason.OBSTACLE_BLOCKED,
                description=f"Safety stop: {reason}"
            )
        elif "TERRAIN" in reason.upper():
            self.log_path_rejection(
                reason=RejectionReason.TERRAIN_UNSAFE,
                description=f"Terrain issue: {reason}"
            )
        elif "LOCALIZATION" in reason.upper():
            self.log_path_rejection(
                reason=RejectionReason.LOCALIZATION_POOR,
                description=f"Localization issue: {reason}"
            )
    
    def recovery_state_callback(self, msg: String):
        """Track recovery state transitions and outcomes."""
        state = msg.data
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Detect recovery start
        if state == "RECOVERY_ACTIVE" and self.last_recovery_state != "RECOVERY_ACTIVE":
            self.recovery_start_time = current_time
            self.get_logger().info('Recovery started')
        
        # Detect recovery end
        elif self.last_recovery_state == "RECOVERY_ACTIVE" and state != "RECOVERY_ACTIVE":
            if self.recovery_start_time is not None:
                duration = current_time - self.recovery_start_time
                
                # Determine outcome
                if state == "NORMAL":
                    outcome = RecoveryOutcome.SUCCESS
                elif state == "STOPPED":
                    outcome = RecoveryOutcome.FAILED
                else:
                    outcome = RecoveryOutcome.BLOCKED
                
                self.log_recovery_outcome(
                    outcome=outcome,
                    duration_sec=duration,
                    final_state=state,
                    description=f"Recovery transitioned to {state}"
                )
                
                self.recovery_start_time = None
        
        self.last_recovery_state = state
    
    def log_path_rejection(
        self,
        reason: RejectionReason,
        description: str = "",
        cost_value: Optional[float] = None
    ):
        """Log a rejected path with reason."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        path_length = 0.0
        if self.last_path and len(self.last_path.poses) > 0:
            # Compute approximate path length
            path_length = len(self.last_path.poses) * 0.1  # Rough estimate
        
        rejection = RejectedPath(
            timestamp=current_time,
            path_length=path_length,
            rejection_reason=reason,
            cost_value=cost_value,
            description=description
        )
        
        self.rejected_paths.append(rejection)
        
        # Publish event
        event = MissionEvent()
        event.stamp = self.get_clock().now().to_msg()
        event.severity = "WARNING"
        event.subsystem = "PLANNER"
        event.description = f"Path rejected: {reason.value}"
        event.cause = description
        event.action_taken = "Searching for alternative path"
        
        self.mission_event_pub.publish(event)
        
        # Publish rationale
        rationale_msg = String()
        rationale_msg.data = (
            f"REJECTED: {reason.value} | "
            f"Path length: {path_length:.1f}m | "
            f"Reason: {description}"
        )
        self.decision_rationale_pub.publish(rationale_msg)
        
        self.get_logger().warn(f'Path rejected: {reason.value} - {description}')
    
    def log_recovery_outcome(
        self,
        outcome: RecoveryOutcome,
        duration_sec: float,
        final_state: str,
        description: str = "",
        displacement_m: float = 0.0,
        attempts_count: int = 1
    ):
        """Log a recovery attempt outcome."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        recovery = RecoveryAttempt(
            timestamp=current_time,
            recovery_phase="ROTATE_REVERSE",  # Could be tracked more precisely
            outcome=outcome,
            duration_sec=duration_sec,
            displacement_m=displacement_m,
            attempts_count=attempts_count,
            final_state=final_state,
            description=description
        )
        
        self.recovery_attempts.append(recovery)
        
        # Publish event
        event = MissionEvent()
        event.stamp = self.get_clock().now().to_msg()
        
        if outcome == RecoveryOutcome.SUCCESS:
            event.severity = "RECOVERY"
            event.action_taken = "Resumed normal navigation"
        elif outcome == RecoveryOutcome.FAILED:
            event.severity = "CRITICAL"
            event.action_taken = "Recovery failed, system stopped"
        else:
            event.severity = "WARNING"
            event.action_taken = "Recovery blocked, trying alternative"
        
        event.subsystem = "RECOVERY"
        event.description = f"Recovery outcome: {outcome.value}"
        event.cause = f"Duration: {duration_sec:.1f}s | Final state: {final_state}"
        
        self.mission_event_pub.publish(event)
        
        # Publish rationale
        rationale_msg = String()
        rationale_msg.data = (
            f"RECOVERY: {outcome.value} | "
            f"Duration: {duration_sec:.1f}s | "
            f"Attempts: {attempts_count} | "
            f"Final: {final_state}"
        )
        self.decision_rationale_pub.publish(rationale_msg)
        
        self.get_logger().info(
            f'Recovery outcome: {outcome.value} after {duration_sec:.1f}s → {final_state}'
        )
    
    def process(self):
        """Periodic processing and statistics."""
        # Could publish summary statistics here
        pass
    
    def get_rejection_statistics(self) -> dict:
        """Get statistics on rejected paths."""
        if not self.rejected_paths:
            return {}
        
        total = len(self.rejected_paths)
        by_reason = {}
        
        for rejection in self.rejected_paths:
            reason = rejection.rejection_reason.value
            by_reason[reason] = by_reason.get(reason, 0) + 1
        
        return {
            'total_rejections': total,
            'by_reason': by_reason,
            'recent_rejections': self.rejected_paths[-10:]
        }
    
    def get_recovery_statistics(self) -> dict:
        """Get statistics on recovery attempts."""
        if not self.recovery_attempts:
            return {}
        
        total = len(self.recovery_attempts)
        successes = sum(1 for r in self.recovery_attempts if r.outcome == RecoveryOutcome.SUCCESS)
        failures = sum(1 for r in self.recovery_attempts if r.outcome == RecoveryOutcome.FAILED)
        
        avg_duration = sum(r.duration_sec for r in self.recovery_attempts) / total
        
        return {
            'total_attempts': total,
            'successes': successes,
            'failures': failures,
            'success_rate': successes / total if total > 0 else 0.0,
            'average_duration_sec': avg_duration,
            'recent_attempts': self.recovery_attempts[-10:]
        }


def main(args=None):
    rclpy.init(args=args)
    node = ExplainabilityLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final statistics
        rejection_stats = node.get_rejection_statistics()
        recovery_stats = node.get_recovery_statistics()
        
        node.get_logger().info('=== Final Explainability Statistics ===')
        node.get_logger().info(f'Rejections: {rejection_stats}')
        node.get_logger().info(f'Recovery: {recovery_stats}')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
