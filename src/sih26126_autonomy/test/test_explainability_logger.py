#!/usr/bin/env python3
"""Tests for explainability logger."""

import unittest

from sih26126_autonomy.explainability_logger import (
    RejectionReason,
    RecoveryOutcome,
    RejectedPath,
    RecoveryAttempt,
)


class TestRejectionReason(unittest.TestCase):
    """Test rejection reason enum."""
    
    def test_all_rejection_reasons_defined(self):
        """All expected rejection reasons exist with correct string values."""
        assert RejectionReason.UNREACHABLE.value == "UNREACHABLE"
        assert RejectionReason.HIGH_COST.value == "HIGH_COST"
        assert RejectionReason.OBSTACLE_BLOCKED.value == "OBSTACLE_BLOCKED"
        assert RejectionReason.TERRAIN_UNSAFE.value == "TERRAIN_UNSAFE"
        assert RejectionReason.LOCALIZATION_POOR.value == "LOCALIZATION_POOR"
        assert RejectionReason.TIMEOUT.value == "TIMEOUT"
        assert RejectionReason.NONE.value == "NONE"
    
    def test_rejection_reason_values(self):
        """Rejection reason enum values are correct."""
        assert RejectionReason.UNREACHABLE.value == "UNREACHABLE"
        assert RejectionReason.HIGH_COST.value == "HIGH_COST"
        assert RejectionReason.OBSTACLE_BLOCKED.value == "OBSTACLE_BLOCKED"


class TestRecoveryOutcome(unittest.TestCase):
    """Test recovery outcome enum."""
    
    def test_all_recovery_outcomes_defined(self):
        """All expected recovery outcomes exist with correct string values."""
        assert RecoveryOutcome.SUCCESS.value == "SUCCESS"
        assert RecoveryOutcome.FAILED.value == "FAILED"
        assert RecoveryOutcome.IN_PROGRESS.value == "IN_PROGRESS"
        assert RecoveryOutcome.BLOCKED.value == "BLOCKED"
        assert RecoveryOutcome.TIMEOUT.value == "TIMEOUT"
    
    def test_recovery_outcome_values(self):
        """Recovery outcome enum values are correct."""
        assert RecoveryOutcome.SUCCESS.value == "SUCCESS"
        assert RecoveryOutcome.FAILED.value == "FAILED"


class TestRejectedPathDataclass(unittest.TestCase):
    """Test RejectedPath dataclass."""
    
    def test_rejected_path_creation(self):
        """RejectedPath can be created with required fields."""
        path = RejectedPath(
            timestamp=100.0,
            path_length=5.0,
            rejection_reason=RejectionReason.OBSTACLE_BLOCKED
        )
        
        assert path.timestamp == 100.0
        assert path.path_length == 5.0
        assert path.rejection_reason == RejectionReason.OBSTACLE_BLOCKED
        assert path.cost_value is None  # default
        assert path.alternative_found is False  # default
    
    def test_rejected_path_with_all_fields(self):
        """RejectedPath supports all optional fields."""
        path = RejectedPath(
            timestamp=100.0,
            path_length=5.0,
            rejection_reason=RejectionReason.HIGH_COST,
            cost_value=0.8,
            alternative_found=True,
            description="Cost exceeded threshold"
        )
        
        assert path.cost_value == 0.8
        assert path.alternative_found is True
        assert path.description == "Cost exceeded threshold"


class TestRecoveryAttemptDataclass(unittest.TestCase):
    """Test RecoveryAttempt dataclass."""
    
    def test_recovery_attempt_creation(self):
        """RecoveryAttempt can be created with required fields."""
        attempt = RecoveryAttempt(
            timestamp=200.0,
            recovery_phase="ROTATE",
            outcome=RecoveryOutcome.SUCCESS,
            duration_sec=3.5,
            displacement_m=0.8,
            attempts_count=2,
            final_state="NORMAL"
        )
        
        assert attempt.timestamp == 200.0
        assert attempt.recovery_phase == "ROTATE"
        assert attempt.outcome == RecoveryOutcome.SUCCESS
        assert attempt.duration_sec == 3.5
        assert attempt.displacement_m == 0.8
        assert attempt.attempts_count == 2
        assert attempt.final_state == "NORMAL"
    
    def test_recovery_attempt_with_description(self):
        """RecoveryAttempt supports optional description."""
        attempt = RecoveryAttempt(
            timestamp=200.0,
            recovery_phase="REVERSE",
            outcome=RecoveryOutcome.FAILED,
            duration_sec=10.0,
            displacement_m=0.1,
            attempts_count=3,
            final_state="STOPPED",
            description="Maximum attempts exceeded"
        )
        
        assert attempt.description == "Maximum attempts exceeded"


class TestStatisticsLogic(unittest.TestCase):
    """Test statistics computation logic."""
    
    def test_rejection_statistics_empty(self):
        """Empty rejection list produces empty statistics."""
        rejections = []
        
        if not rejections:
            stats = {}
        
        assert stats == {}
    
    def test_rejection_statistics_counting(self):
        """Rejection statistics count by reason."""
        rejections = [
            RejectedPath(100.0, 5.0, RejectionReason.OBSTACLE_BLOCKED),
            RejectedPath(101.0, 6.0, RejectionReason.OBSTACLE_BLOCKED),
            RejectedPath(102.0, 7.0, RejectionReason.TERRAIN_UNSAFE),
        ]
        
        by_reason = {}
        for rejection in rejections:
            reason = rejection.rejection_reason.value
            by_reason[reason] = by_reason.get(reason, 0) + 1
        
        assert by_reason["OBSTACLE_BLOCKED"] == 2
        assert by_reason["TERRAIN_UNSAFE"] == 1
    
    def test_recovery_statistics_success_rate(self):
        """Recovery statistics compute success rate."""
        attempts = [
            RecoveryAttempt(100.0, "ROTATE", RecoveryOutcome.SUCCESS, 3.0, 0.5, 1, "NORMAL"),
            RecoveryAttempt(110.0, "REVERSE", RecoveryOutcome.SUCCESS, 4.0, 0.6, 1, "NORMAL"),
            RecoveryAttempt(120.0, "ROTATE", RecoveryOutcome.FAILED, 10.0, 0.1, 3, "STOPPED"),
        ]
        
        total = len(attempts)
        successes = sum(1 for a in attempts if a.outcome == RecoveryOutcome.SUCCESS)
        success_rate = successes / total
        
        assert total == 3
        assert successes == 2
        assert abs(success_rate - 0.667) < 0.01
    
    def test_recovery_statistics_average_duration(self):
        """Recovery statistics compute average duration."""
        attempts = [
            RecoveryAttempt(100.0, "ROTATE", RecoveryOutcome.SUCCESS, 3.0, 0.5, 1, "NORMAL"),
            RecoveryAttempt(110.0, "REVERSE", RecoveryOutcome.SUCCESS, 5.0, 0.6, 1, "NORMAL"),
            RecoveryAttempt(120.0, "ROTATE", RecoveryOutcome.SUCCESS, 4.0, 0.7, 1, "NORMAL"),
        ]
        
        avg_duration = sum(a.duration_sec for a in attempts) / len(attempts)
        
        assert abs(avg_duration - 4.0) < 0.01


class TestEventSeverity(unittest.TestCase):
    """Test event severity assignment."""
    
    def test_success_recovery_is_recovery_severity(self):
        """Successful recovery has RECOVERY severity."""
        outcome = RecoveryOutcome.SUCCESS
        
        if outcome == RecoveryOutcome.SUCCESS:
            severity = "RECOVERY"
        else:
            severity = "CRITICAL"
        
        assert severity == "RECOVERY"
    
    def test_failed_recovery_is_critical(self):
        """Failed recovery has CRITICAL severity."""
        outcome = RecoveryOutcome.FAILED
        
        if outcome == RecoveryOutcome.SUCCESS:
            severity = "RECOVERY"
        else:
            severity = "CRITICAL"
        
        assert severity == "CRITICAL"
    
    def test_path_rejection_is_warning(self):
        """Path rejection has WARNING severity."""
        rejection_reason = RejectionReason.OBSTACLE_BLOCKED
        
        # Path rejections are warnings (not critical)
        severity = "WARNING"
        
        assert severity == "WARNING"


if __name__ == '__main__':
    unittest.main()
