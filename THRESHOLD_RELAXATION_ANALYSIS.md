# Pose Discontinuity Threshold Relaxation Analysis

## Critical Finding: Unjustified Safety Threshold Weakening

**Date:** October 23, 2026, 19:35 UTC  
**Issue:** Pose discontinuity thresholds were relaxed by 2.5–5× without defensible justification

## Changed Thresholds

### Original (Conservative)
```python
max_pose_speed_mps = 2.0
pose_jump_margin_m = 0.10
```

### New (Relaxed)
```python
max_pose_speed_mps = 5.0     # 2.5× relaxation
pose_jump_margin_m = 0.50    # 5.0× relaxation
```

## Robot Physical Limits (from sih_ugv.sdf)

```xml
<max_linear_velocity>1.5</max_linear_velocity>
<min_linear_velocity>-1.0</min_linear_velocity>
<max_angular_velocity>1.5</max_angular_velocity>
<min_angular_velocity>-1.5</min_angular_velocity>
```

**Maximum achievable speed: 1.5 m/s**

## Threshold vs Reality

| Threshold | Value | Ratio to Robot Max |
|-----------|-------|-------------------|
| Original max_speed | 2.0 m/s | 1.33× (reasonable margin) |
| **New max_speed** | **5.0 m/s** | **3.33× (excessive)** |
| Original margin | 0.10 m | N/A (position tolerance) |
| **New margin** | **0.50 m** | **5× looser** |

## Analysis

### What the threshold means

`pose_jump_exceeded()` flags a pose as invalid if:
```
distance > (max_speed * elapsed) + margin
```

For a typical 0.1s sample interval:
- **Original:** Reject if `delta > (2.0 × 0.1) + 0.10 = 0.30 m`
- **New:** Reject if `delta > (5.0 × 0.1) + 0.50 = 1.00 m`

The new threshold allows **1.0 meter jumps in 0.1 seconds** - that's 10 m/s implied speed!

### Why this is dangerous

1. **Masks genuine localization failures**
   - SLAM corrections should not cause 1m+ jumps in 0.1s
   - Large pose jumps indicate map frame resets, transform errors, or sensor faults
   - Accepting these as "normal" hides real problems

2. **Violates physical constraints**
   - Robot can only move 1.5 m/s max
   - In 0.1s, maximum displacement is 0.15m, not 1.0m
   - Any pose pair showing >0.3m in 0.1s indicates non-physical motion

3. **Defeats the safety mechanism**
   - Pose discontinuity detection exists to catch localization failures
   - If threshold is so loose it never triggers, what's the point?
   - Better to disable the check than pretend it's working

### Claimed justification (from previous session)

Previous reports claim:
- "Gazebo clock artifacts" cause discontinuities
- "SLAM loop-closure corrections" are legitimate
- "32 discontinuities in 101 seconds" needed accommodation

### Problems with this justification

1. **No evidence that Gazebo clock resets occurred**
   - No /clock topic monitoring
   - No proof of sim-time jumps
   - Correlation ≠ causation

2. **SLAM loop-closures don't justify 1m jumps**
   - Loop closures update the map incrementally
   - Well-behaved SLAM doesn't teleport the robot
   - If SLAM is producing 1m+ corrections, SLAM is broken

3. **Treated symptoms, not root cause**
   - Never investigated WHY discontinuities occur
   - Never measured actual motion vs reported motion
   - Just weakened the detector until alarms stopped

## What Should Have Been Done

### Proper diagnosis sequence:
1. **Capture evidence during discontinuity**
   - Pose before/after
   - /clock timestamps
   - TF tree state
   - SLAM internal state

2. **Classify discontinuity types**
   - Gazebo clock resets (sim-time jumps)
   - SLAM map corrections (map→odom changes)
   - Transform lookup failures
   - Timestamp mismatches
   - Actual localization failures

3. **Fix root cause for each type**
   - Clock resets → Fix Gazebo or RTF config
   - SLAM corrections → Improve map stability or confidence reporting
   - TF failures → Fix frame authority or timing
   - Localization failures → Keep detector active, this is REAL

4. **Adjust threshold ONLY if justified**
   - With measured data showing false positives
   - Within physical limits (robot max + reasonable margin)
   - Document tradeoffs explicitly

## Consecutive Discontinuity Tracking

The new code adds a **consecutive counter** requiring 3 discontinuities before SAFE_STOP:
```python
pose_discontinuity_safe_stop_count = 3
```

This is a **reasonable improvement** - it tolerates single isolated corrections without halting the mission. However:
- It STILL requires proper thresholds
- It doesn't justify the 2.5–5× relaxation
- A better solution would identify correction types and handle them differently

## Startup Grace Period

The new code adds a 15-second grace period during startup:
```python
pose_startup_grace_sec = 15.0
```

During grace:
- Frame changes are suppressed (reasonable - SLAM initializing)
- Implausible jumps are STILL checked (correct)

This is a **sound addition** - it handles initialization transients without weakening runtime safety.

## Recommendations

### IMMEDIATE (before next test)
1. **Revert thresholds to conservative values:**
   ```python
   max_pose_speed_mps = 2.5  # 1.67× robot max, reasonable for instantaneous samples
   pose_jump_margin_m = 0.20  # 2× original, modest accommodation
   ```

2. **Keep the good changes:**
   - Consecutive discontinuity counter (3)
   - Startup grace period (15s)
   - Non-monotonic timestamp fallback
   - SAFE_STOP recovery logic

3. **Add discontinuity classification:**
   - Log /clock time at each discontinuity
   - Log map→odom vs odom→base_link deltas
   - Distinguish "large delta with plausible elapsed" from "small elapsed impossible speed"

### DIAGNOSTIC (during next clean test)
1. **Monitor /clock for resets**
2. **Log TF authority and timing**
3. **Capture SLAM correction events**
4. **Measure actual vs reported displacement**

### LONG-TERM
1. **Root cause each discontinuity type**
2. **Fix Gazebo config if clock resets confirmed**
3. **Improve SLAM if corrections too abrupt**
4. **Keep safety thresholds within physical envelope**

## Verdict

**The 5.0 m/s / 0.50 m threshold relaxation is UNJUSTIFIED and UNSAFE.**

It was done to suppress alarm symptoms without understanding or fixing the root cause. It allows the mission to proceed through localization states that should be flagged as failures.

**Action:** Revert to conservative thresholds (max 2.5 m/s, margin 0.20 m) and properly diagnose discontinuities in the next clean test.

---
**Analyst:** Kiro (autonomous AI agent)  
**Priority:** CRITICAL — Safety mechanism weakened  
**Status:** Requires immediate correction before acceptance testing
