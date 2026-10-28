# DRISTHI Process Contamination Discovery — October 23, 2026

## Critical Finding: Previous Acceptance Test Was Invalid

**Discovery Time:** October 23, 2026, 19:27 UTC  
**Issue:** Multiple complete DRISTHI autonomy stacks were running simultaneously during the acceptance test, contaminating results.

## Process Archaeology

### Stack 1 (Orphaned from earlier test, started 18:46)
- **Age:** ~41 minutes at discovery
- **PID range:** 6934-7379
- **Key processes:**
  - perception_baseline (6934)
  - frontier_explorer (7375)
  - mission_manager (7376)
  - dashboard (7089)
  - rgbd_odometry_baseline (6942)
  - All owned by PPID 367 (orphaned)

### Stack 2 (Acceptance test, started 19:20)
- **Age:** ~7 minutes at discovery
- **Launch PID:** 11120 (exited, but left children orphaned)
- **PID range:** 11214-11486
- **Key processes:**
  - perception_baseline (11214) - DUPLICATE
  - frontier_explorer (11480) - DUPLICATE
  - mission_manager (11481) - DUPLICATE
  - All owned by PPID 367 (orphaned when launch died)

### Shared Gazebo
- **PID:** 11125
- **Started:** 19:19
- **Also orphaned** (PPID 367)

### Total Contamination
- **63 orphaned ROS2 nodes** from failed launches
- **2 complete autonomy stacks** fighting for control
- **2 mission_manager nodes** publishing /mission/state
- **2 frontier_explorer nodes** publishing goals
- **2 perception pipelines** processing same sensor data
- **2 safety_supervisor nodes** potentially publishing /cmd_vel

## Why This Invalidates the Test

**Topic conflicts:**
- `/mission/state` — two publishers, interleaved messages
- `/exploration/goal` — two frontier explorers with different internal state
- `/mission/goal` — two mission managers with different transitions
- Potentially `/cmd_vel` — could have two safety supervisors (CRITICAL SAFETY ISSUE)

**Observed symptoms explained:**
- Rapid FAILED → EXPLORING → REPLANNING cycles (two state machines)
- "fusion state=FAILED imu=False odom=False ekf=False" at startup (stack 1's monitor)
- Conflicting goal publications (bootstrap from one stack, frontier from another)
- Suspicious distance_travelled metric (which mission_manager calculated it?)

**Acceptance report claims are unverifiable:**
- "SAFE_STOP recovery worked" — which mission_manager?
- "32 discontinuities detected" — from which pose_cb?
- "Bootstrap activated 3 times" — from which frontier_explorer?
- All state transitions and timing suspect

## Root Cause: Shutdown Script Failure

Inspected `run_acceptance_test.sh`:
- Launch PID stored to file
- Script sends SIGINT to launch PID
- Then SIGTERM
- Then SIGKILL

**Why it failed:**
- Launch process exits but doesn't propagate signal to children
- All spawned nodes become orphaned (reparented to PID 1/367)
- Script never tracked or terminated children
- No verification that children actually exited

**Evidence from process table:**
ALL nodes had PPID 367, not the launch PID. Launch exited but left 60+ children running.

## Cleanup Actions Taken

Created `emergency_cleanup.sh` to:
1. Identify orphaned nodes by parent PID 367 + path matching
2. Send SIGINT to all (graceful)
3. Wait 5s, send SIGTERM to survivors
4. Wait 3s, send SIGKILL to survivors (last resort)
5. Handle Gazebo and bridges separately
6. Verify final clean state

**Termination summary:**
- 62/63 nodes responded to SIGINT
- 1 node (terrain_aware_traversability) required SIGTERM then SIGKILL
- Gazebo required SIGTERM (SIGINT didn't work)
- 2 bridge processes terminated
- Final state: 0 orphaned processes (except 1 EKF which was manually terminated)

## Implications

**Previous acceptance test report is UNRELIABLE:**
- Cannot determine which stack's logs are in the captured file
- Cannot trust any state transition timing
- Cannot trust metrics (distance, discontinuities, etc.)
- Cannot verify safety claims

**Distance metric of 306m is ESPECIALLY SUSPECT:**
- Two mission_managers accumulating distance
- Unclear which one published the final metric
- Pose discontinuities may have been caused by two SLAM instances competing

**Frontier exhaustion may be misdiagnosed:**
- Two frontier_explorers with different blacklists, completed sets, etc.
- Goals published by one may have been rejected by the other's planner
- Bootstrap from one stack, normal frontiers from another

## Required Actions

1. ✅ **Terminate all orphaned processes** — DONE
2. ⏳ **Fix run_acceptance_test.sh shutdown** — track and terminate children properly
3. ⏳ **Run CLEAN acceptance test** — single stack, verified isolation
4. ⏳ **Re-analyze logs** — only from verified single-stack run
5. ⏳ **Diagnose real failure** — frontier exhaustion, fusion health, or something else

## Lessons

**Process management in test scripts:**
- Store launch PID AND enumerate children before shutdown
- Use process groups or job control for clean termination
- Verify children exit, don't assume
- Add timeout and forced kill for stuck processes
- Log PID tracking for forensics

**Test environment hygiene:**
- Check for orphaned processes BEFORE launching test
- Verify clean state at start and end
- Fail test if pre-existing processes detected
- Never trust results from contaminated environment

---
**Documented by:** Kiro (autonomous AI agent)  
**Date:** October 23, 2026, 19:28 UTC  
**Status:** Environment now clean, ready for proper diagnosis and testing
