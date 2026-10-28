# Test #2 Actual Analysis - 2026-10-26T12:12:00Z

## Executive Summary

**Previous Claim**: Test #2 was "successful 90s autonomous operation"  
**Actual Result**: Test #2 showed mostly STOPPED robot, not continuous navigation

## Evidence from runtime_evidence_no_visual_20261026T105734/

### Safety Supervisor States
- **SAFE_COMMAND count**: 2 messages only
- **STOPPED count**: 8 messages
- **Ratio**: 80% stopped, 20% safe command

### STOPPED Reasons
1. WAITING_FOR_NAVIGATION_STATE (startup)
2. STALE_NAVIGATION_STATE (timeout)
3. NAVIGATION_NOT_ACTIVE (majority - 5 occurrences)

### Timeline Analysis
```
T+2s:  STOPPED - WAITING_FOR_NAVIGATION_STATE
T+12s: STOPPED - STALE_NAVIGATION_STATE  
T+23s: STOPPED - NAVIGATION_NOT_ACTIVE
T+33s: STOPPED - NAVIGATION_NOT_ACTIVE
T+43s: SAFE_COMMAND linear=0.067 (FIRST movement command)
T+53s: STOPPED - NAVIGATION_NOT_ACTIVE
T+63s: SAFE_COMMAND linear=0.074 (SECOND movement command)
T+74s: STOPPED - STALE_NAVIGATION_STATE
T+84s: STOPPED - NAVIGATION_NOT_ACTIVE
T+94s: STOPPED - NAVIGATION_NOT_ACTIVE
```

### Mission State Reality
- mission_manager states: MAPPING → EXPLORING → REPLANNING cycles
- mission_manager NEVER entered SAFE_STOP
- mission_intelligence reported SAFE_STOP (dashboard status, not actual state)

### SLAM Queue Drops
- Count: 17 drops during test
- Timespan: 2.785s to 44.980s sim-time (42.2s duration)
- Pattern: Roughly every 2-3 sim-seconds
- Status: UNRESOLVED

### Single Pose Discontinuity
```
delta=0.349 m in 0.025 s
velocity = 13.96 m/s
threshold = 2.5*0.025 + 0.10 = 0.1625 m
Result: Correctly flagged, not consecutive, no SAFE_STOP triggered
```

## Root Cause Analysis

### Why Robot Was Stopped

The safety_supervisor requires:
1. navigation_state == "LOCAL_NAVIGATE"
2. Fresh navigation command (< 0.75s old)
3. Fresh navigation state (< 0.75s old)
4. Valid obstacle score
5. Valid localization

The robot was STOPPED because **the local planner was not consistently publishing navigation commands or navigation state**.

### Why NAVIGATION_NOT_ACTIVE

Possible causes:
1. Local planner waiting for global path
2. Global planner not finding valid paths
3. Frontier explorer not issuing goals
4. Map too small/fragmented for path planning
5. Localization quality too low
6. Obstacle scores preventing navigation
7. SLAM queue drops affecting map updates

## Comparison with Test #1

### Test #1 (visual odom enabled)
- Result: 3+ consecutive discontinuities → SAFE_STOP at T+85s
- Safety: SAFE_COMMAND for first 85s, then permanent SAFE_STOP
- Mission: Reached EXPLORING, then SAFE_STOP

### Test #2 (visual odom disabled)
- Result: 1 isolated discontinuity, no SAFE_STOP from discontinuities
- Safety: Intermittent SAFE_COMMAND (2 messages), mostly STOPPED (8 messages)
- Mission: Cycled EXPLORING/REPLANNING, never SAFE_STOP

**Improvement**: Eliminating visual odom prevented consecutive discontinuities
**Problem**: Robot now fails to navigate due to navigation pipeline issues

## What Was Actually Verified

✅ Visual odometry was causing pose discontinuities (confirmed)
✅ Disabling visual odom prevents consecutive discontinuities (confirmed)
✅ Monotonic timestamps working (confirmed)
✅ Mission manager not entering SAFE_STOP (confirmed)
✅ No process contamination after test (confirmed via cleanup)

## What Was NOT Verified

❌ Continuous autonomous navigation (robot was mostly stopped)
❌ Sustained movement (only 2 SAFE_COMMAND samples)
❌ Goal reaching (no ground truth motion samples)
❌ Map growth and frontier completion (not measured)
❌ 60+ seconds healthy navigation (< 20 seconds at best)

## Required Fixes

### Immediate
1. Diagnose why local planner not publishing navigation state consistently
2. Check global path planning and frontier goal validity
3. Verify map quality after SLAM queue drops
4. Ensure obstacle scores being published
5. Fix test script to capture ground truth motion

### Investigation Needed
1. Why is navigation_state becoming stale?
2. Is global planner receiving valid goals?
3. Are frontier goals reachable with current map?
4. What happens during REPLANNING transitions?
5. Why only 2 SAFE_COMMAND messages in 90+ seconds?

## Conclusion

Test #2 did NOT demonstrate successful autonomous operation. The robot was stopped 80% of the time due to navigation pipeline failures, not safety issues.

The claim of "90 seconds successful operation" was based on:
- Mission not entering SAFE_STOP (correct but insufficient)
- No consecutive pose discontinuities (correct but insufficient)  
- Misinterpreting mission_intelligence status as mission_manager state (incorrect)
- Not measuring actual cmd_vel publishing or robot motion (oversight)

**Status**: Robot is NOT operational. Navigation pipeline broken.
