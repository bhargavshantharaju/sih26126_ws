"""Tests for the RGB-D localization startup grace period in mission_manager.pose_cb.

All tests are pure Python — no rclpy.init() required.  The harness below
replicates the state machine in MissionManager.pose_cb exactly as written
after the Blocker-C fix.
"""

import math

from sih26126_autonomy.mission_manager import pose_jump_exceeded


# ---------------------------------------------------------------------------
# Minimal harness replicating MissionManager.pose_cb with grace period
# ---------------------------------------------------------------------------

class _GraceHarness:
    """Replicate the pose_cb startup-grace-period state machine without ROS.

    Parameters
    ----------
    grace_sec : float
        Length of the grace period in seconds (mirrors pose_startup_grace_sec).
    max_speed : float
        max_pose_speed_mps parameter.
    margin : float
        pose_jump_margin_m parameter.
    threshold : int
        pose_discontinuity_safe_stop_count parameter.
    """

    _STABLE_RESET_THRESHOLD = 3

    def __init__(
        self,
        grace_sec: float = 15.0,
        max_speed: float = 5.0,
        margin: float = 0.50,
        threshold: int = 3,
    ) -> None:
        self.grace_sec = grace_sec
        self.max_speed = max_speed
        self.margin = margin
        self.threshold = threshold

        # Replicated state
        self.consecutive_pose_discontinuities = 0
        self.consecutive_stable_poses = 0
        self.pose_discontinuity_count = 0
        self.persistent_discontinuity_fault = False
        self.state = 'EXPLORING'
        self.distance_travelled = 0.0

        # Grace-period tracking (wall-clock simulation via explicit time param)
        self._pose_grace_end: float | None = None
        self._pose_grace_logged = False
        self.grace_expired_logged = False  # set True when INFO logged

        # Internal xy for delta computation
        self._last_pose_xy: tuple[float, float] | None = None
        self._last_raw_time: float | None = None
        self._xy = (0.0, 0.0)

    # ------------------------------------------------------------------
    # Simulate one incoming pose message.
    #
    # Parameters:
    #   delta_m  – planar distance moved since last pose
    #   elapsed_s – time elapsed since last pose (for speed check)
    #   frame    – frame_id of the incoming message
    #   now      – simulated wall-clock time (time.monotonic() equivalent)
    #   frame_changed_force – if True, override frame comparison to True
    #                         (shorthand so callers don't need to track
    #                          last_frame separately)
    # Returns True if accepted, False if rejected as a discontinuity.
    # ------------------------------------------------------------------

    def feed(
        self,
        delta_m: float = 0.05,
        elapsed_s: float = 0.1,
        frame: str = 'map',
        now: float = 0.0,
        last_frame: str | None = None,
    ) -> bool:
        """Simulate one incoming PoseStamped message.

        Returns True if the pose was accepted, False if rejected.
        """
        # Grace-period initialisation (first pose ever received)
        if self._pose_grace_end is None:
            self._pose_grace_end = now + self.grace_sec

        in_grace = now < self._pose_grace_end

        # Log grace expiry once
        if not in_grace and not self._pose_grace_logged:
            self._pose_grace_logged = True
            self.grace_expired_logged = True

        if self._last_pose_xy is not None:
            frame_changed = bool(
                last_frame is not None
                and frame is not None
                and frame != last_frame
            )
            implausible = pose_jump_exceeded(
                delta_m,
                elapsed_s,
                max_speed_mps=self.max_speed,
                margin_m=self.margin,
            )

            is_discontinuity = (not in_grace and frame_changed) or implausible

            if is_discontinuity:
                self.pose_discontinuity_count += 1
                self.consecutive_pose_discontinuities += 1
                if (
                    self.state not in ('MISSION_COMPLETE', 'FAILED', 'SAFE_STOP')
                    and self.consecutive_pose_discontinuities >= self.threshold
                ):
                    self.persistent_discontinuity_fault = True
                    self.state = 'SAFE_STOP'
                # Reset stable streak
                self.consecutive_stable_poses = 0
                return False

            elif in_grace and frame_changed:
                # Suppress frame fault during grace — reset consecutive counter
                self.consecutive_pose_discontinuities = 0

            self.distance_travelled += delta_m

        # Accepted
        new_xy = (self._xy[0] + delta_m, self._xy[1])
        self._last_pose_xy = new_xy
        self._xy = new_xy

        self.consecutive_stable_poses += 1
        if self.consecutive_stable_poses >= self._STABLE_RESET_THRESHOLD:
            self.consecutive_pose_discontinuities = 0
            self.consecutive_stable_poses = self._STABLE_RESET_THRESHOLD

        return True


# ===========================================================================
# Test 1 — Frame change during grace period is NOT counted as a discontinuity
# ===========================================================================

def test_frame_change_during_grace_not_counted_as_discontinuity():
    """Frame changes from visual_odom -> map during grace must be silently accepted."""
    h = _GraceHarness(grace_sec=15.0)

    # First pose at t=0 — establishes grace end = 15.0
    h.feed(delta_m=0.0, elapsed_s=0.0, frame='visual_odom', now=0.0,
           last_frame=None)

    # Several alternating frame changes during grace (t=1..10, well within 15 s)
    frames = ['visual_odom', 'map', 'visual_odom', 'map', 'visual_odom', 'map']
    for i, (cur, prev) in enumerate(zip(frames[1:], frames[:-1]), start=1):
        accepted = h.feed(
            delta_m=0.05,
            elapsed_s=0.1,
            frame=cur,
            now=float(i),
            last_frame=prev,
        )
        assert accepted, (
            f'Frame change {prev}->{cur} at t={i}s (within grace) must be accepted; '
            f'consecutive={h.consecutive_pose_discontinuities}'
        )

    assert h.pose_discontinuity_count == 0, (
        'No discontinuities should have been counted during grace period; '
        f'got {h.pose_discontinuity_count}'
    )
    assert h.consecutive_pose_discontinuities == 0
    assert h.state == 'EXPLORING'


# ===========================================================================
# Test 2 — Frame change AFTER grace period IS counted as a discontinuity
# ===========================================================================

def test_frame_change_after_grace_is_counted():
    """Frame changes after grace expiry must be flagged as FRAME_CHANGED."""
    h = _GraceHarness(grace_sec=5.0)

    # Establish last_pose_xy at t=0 (same frame)
    h.feed(delta_m=0.0, elapsed_s=0.0, frame='map', now=0.0, last_frame=None)
    # Stable same-frame pose at t=4 (still in grace)
    h.feed(delta_m=0.05, elapsed_s=4.0, frame='map', now=4.0, last_frame='map')

    # t=6 — grace ended at t=5; now a frame change must be flagged
    accepted = h.feed(
        delta_m=0.05,
        elapsed_s=2.0,
        frame='visual_odom',
        now=6.0,
        last_frame='map',
    )
    assert not accepted, 'Frame change after grace must be flagged as discontinuity'
    assert h.consecutive_pose_discontinuities == 1
    assert h.pose_discontinuity_count == 1


# ===========================================================================
# Test 3 — Large position jump during grace IS still a discontinuity
# ===========================================================================

def test_large_jump_during_grace_is_still_discontinuity():
    """Implausible position deltas must always be flagged, even within grace."""
    h = _GraceHarness(grace_sec=15.0, max_speed=5.0, margin=0.50)

    # First pose establishes tracking
    h.feed(delta_m=0.0, elapsed_s=0.0, frame='visual_odom', now=0.0,
           last_frame=None)

    # t=1 — 50 m jump in 0.01 s — clearly implausible regardless of frame
    accepted = h.feed(
        delta_m=50.0,
        elapsed_s=0.01,
        frame='visual_odom',
        now=1.0,
        last_frame='visual_odom',
    )
    assert not accepted, (
        'Implausible delta must be rejected even during grace period'
    )
    assert h.pose_discontinuity_count == 1
    assert h.consecutive_pose_discontinuities == 1


# ===========================================================================
# Test 4 — Grace period expires at the configured time
# ===========================================================================

def test_grace_period_expires_at_configured_time():
    """The grace period boundary is exactly pose_startup_grace_sec seconds
    after the first pose.  Poses just inside must be accepted; poses just
    outside must apply strict checks."""
    h = _GraceHarness(grace_sec=10.0)

    # First pose at t=0
    h.feed(delta_m=0.0, elapsed_s=0.0, frame='map', now=0.0, last_frame=None)

    # At t=9.99 (just inside grace) a frame change must be accepted
    accepted_inside = h.feed(
        delta_m=0.05,
        elapsed_s=9.99,
        frame='visual_odom',
        now=9.99,
        last_frame='map',
    )
    assert accepted_inside, 'Frame change just inside grace must be accepted'

    # Reset to same frame so the next check is a clean frame change
    # (re-feed accepted pose as 'visual_odom' from 'visual_odom' — no change)
    h.feed(delta_m=0.01, elapsed_s=0.01, frame='visual_odom', now=9.999,
           last_frame='visual_odom')

    # At t=10.01 (just outside grace) same frame change must be flagged
    accepted_outside = h.feed(
        delta_m=0.05,
        elapsed_s=0.011,
        frame='map',
        now=10.01,
        last_frame='visual_odom',
    )
    assert not accepted_outside, 'Frame change just after grace must be flagged'
    assert not h.grace_expired_logged is False  # the INFO log was emitted


def test_grace_expired_log_is_emitted_exactly_once():
    """The 'grace period ended' INFO message must fire exactly once."""
    h = _GraceHarness(grace_sec=5.0)

    # First pose at t=0
    h.feed(delta_m=0.0, elapsed_s=0.0, frame='map', now=0.0, last_frame=None)

    # Pre-expiry: no log yet
    h.feed(delta_m=0.02, elapsed_s=1.0, frame='map', now=1.0, last_frame='map')
    assert not h.grace_expired_logged

    # First pose after expiry: log fires
    h.feed(delta_m=0.02, elapsed_s=4.5, frame='map', now=5.5, last_frame='map')
    assert h.grace_expired_logged, (
        'grace_expired_logged must be True after first post-grace pose'
    )

    # Additional poses: flag stays True (not toggled back)
    logged_before = h.grace_expired_logged
    h.feed(delta_m=0.02, elapsed_s=0.1, frame='map', now=5.6, last_frame='map')
    assert h.grace_expired_logged == logged_before, (
        'grace_expired_logged must not be reset on subsequent poses'
    )


# ===========================================================================
# Test 5 — After grace ends, three consecutive frame changes trigger SAFE_STOP
# ===========================================================================

def test_three_consecutive_frame_changes_after_grace_trigger_safe_stop():
    """Three consecutive FRAME_CHANGED faults after grace must trigger SAFE_STOP."""
    h = _GraceHarness(grace_sec=5.0, threshold=3)

    # First pose at t=0 to establish tracking
    h.feed(delta_m=0.0, elapsed_s=0.0, frame='map', now=0.0, last_frame=None)

    # One stable same-frame pose to set last_frame = 'map' at t=1
    h.feed(delta_m=0.05, elapsed_s=1.0, frame='map', now=1.0, last_frame='map')

    # t=6,7,8 — all after grace (ended at t=5), alternating frames
    frame_sequence = [
        ('visual_odom', 'map'),
        ('map', 'visual_odom'),
        ('visual_odom', 'map'),
    ]
    for i, (cur, prev) in enumerate(frame_sequence, start=6):
        accepted = h.feed(
            delta_m=0.05,
            elapsed_s=1.0,
            frame=cur,
            now=float(i),
            last_frame=prev,
        )
        assert not accepted, f'Frame change at t={i} after grace must be rejected'

    assert h.state == 'SAFE_STOP', (
        f'Three consecutive post-grace frame changes must trigger SAFE_STOP; '
        f'state={h.state} consecutive={h.consecutive_pose_discontinuities}'
    )
    assert h.persistent_discontinuity_fault


# ===========================================================================
# Test 6 — Recovery: three consecutive good poses after SAFE_STOP reset counter
# ===========================================================================

def test_recovery_after_safe_stop_resets_consecutive_discontinuities():
    """After entering SAFE_STOP, if localization stabilizes (3 consecutive good
    poses), consecutive_pose_discontinuities resets to zero.

    Note: this tests the counter reset logic directly. The harness does not
    re-open SAFE_STOP (the real manager keeps that terminal), but the counter
    arithmetic must work regardless of state so that any future recovery path
    would have a clean baseline.
    """
    h = _GraceHarness(grace_sec=5.0, threshold=3)

    # Establish tracking at t=0
    h.feed(delta_m=0.0, elapsed_s=0.0, frame='map', now=0.0, last_frame=None)
    h.feed(delta_m=0.05, elapsed_s=1.0, frame='map', now=1.0, last_frame='map')

    # Drive three consecutive frame-change faults after grace to hit SAFE_STOP
    for i in range(3):
        cur = 'visual_odom' if i % 2 == 0 else 'map'
        prev = 'map' if i % 2 == 0 else 'visual_odom'
        h.feed(
            delta_m=0.05,
            elapsed_s=1.0,
            frame=cur,
            now=float(6 + i),
            last_frame=prev,
        )

    assert h.state == 'SAFE_STOP'
    pre_recovery_consecutive = h.consecutive_pose_discontinuities

    # Simulate three stable, same-frame, plausible poses
    for i in range(3):
        h.feed(
            delta_m=0.05,
            elapsed_s=0.5,
            frame='map',
            now=float(9 + i),
            last_frame='map',
        )

    assert h.consecutive_pose_discontinuities == 0, (
        f'consecutive_pose_discontinuities must reset to 0 after 3 stable poses; '
        f'was {pre_recovery_consecutive} before recovery, '
        f'now {h.consecutive_pose_discontinuities}'
    )
    assert h.consecutive_stable_poses == _STABLE_RESET_THRESHOLD(), 'stable streak capped'


def _STABLE_RESET_THRESHOLD():
    return 3


# ===========================================================================
# Test 7 — Mixed scenario: grace absorbs early races, then strict mode works
# ===========================================================================

def test_full_startup_sequence_grace_then_strict():
    """End-to-end scenario: grace absorbs visual_odom/map alternation at startup,
    strict mode activates after grace, and three post-grace frame changes fire
    SAFE_STOP while in EXPLORING state."""
    h = _GraceHarness(grace_sec=15.0, threshold=3)

    # t=0 — first pose: visual_odom (RGB-D comes up first)
    h.feed(delta_m=0.0, elapsed_s=0.0, frame='visual_odom', now=0.0,
           last_frame=None)

    # t=1..8 — alternating visual_odom / map frames during grace
    frames = ['visual_odom', 'map', 'visual_odom', 'map',
              'visual_odom', 'map', 'visual_odom', 'map']
    prev = 'visual_odom'
    for i, cur in enumerate(frames, start=1):
        accepted = h.feed(
            delta_m=0.05,
            elapsed_s=1.0,
            frame=cur,
            now=float(i),
            last_frame=prev,
        )
        assert accepted, (
            f'Pose at t={i} (frame={cur}) during grace must be accepted'
        )
        prev = cur

    # No discontinuities accumulated during grace
    assert h.pose_discontinuity_count == 0
    assert h.state == 'EXPLORING'

    # t=16,17,18 — after grace (ended at t=15): three frame changes
    post_frames = [
        ('visual_odom', 'map'),
        ('map', 'visual_odom'),
        ('visual_odom', 'map'),
    ]
    for i, (cur, prv) in enumerate(post_frames, start=16):
        h.feed(
            delta_m=0.05,
            elapsed_s=1.0,
            frame=cur,
            now=float(i),
            last_frame=prv,
        )

    assert h.state == 'SAFE_STOP', (
        f'Three post-grace frame changes must trigger SAFE_STOP; '
        f'state={h.state}'
    )
