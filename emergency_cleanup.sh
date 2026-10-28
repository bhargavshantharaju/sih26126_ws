#!/bin/bash
# Emergency cleanup for orphaned DRISTHI processes.
#
# Identifies orphaned ROS 2 / Gazebo / SLAM processes whose parent has exited
# (PPID=1 or parent PID no longer running).  Uses graceful SIGINT → SIGTERM
# → SIGKILL escalation.  Does NOT use hardcoded parent PIDs.
#
# Safe to run when no test is active.  When a test IS running, run this only
# if the launch process has already died and left orphans.

set -uo pipefail

echo "==================================================================="
echo "DRISTHI Emergency Cleanup"
echo "==================================================================="
echo ""

ORPHAN_PATTERNS="install/sih26126|gz sim|ros_gz_bridge|ekf_node|slam_toolbox|async_slam"

echo "Scanning for DRISTHI-related processes..."
echo ""

# Find all matching processes with their PIDs and PPIDs
ALL_MATCHING=$(ps -eo pid,ppid,cmd | grep -E "(${ORPHAN_PATTERNS})" | grep -v grep | grep -v "emergency_cleanup" | grep -v "run_clean" || true)

if [ -z "$ALL_MATCHING" ]; then
    echo "✓ No DRISTHI-related processes found. System is clean."
    exit 0
fi

echo "Found processes:"
echo "$ALL_MATCHING"
echo ""

# Identify orphaned ones: parent PID is 1 (init) or parent PID no longer exists.
# A process whose parent has exited gets reparented to PID 1.
ORPHANED_PIDS=""
NON_ORPHANED=""
while IFS= read -r line; do
    [ -z "$line" ] && continue
    pid=$(echo "$line" | awk '{print $1}')
    ppid=$(echo "$line" | awk '{print $2}')
    # Check if parent is still alive
    if [ "$ppid" -eq 1 ] 2>/dev/null; then
        # Reparented to init — orphaned
        ORPHANED_PIDS="$ORPHANED_PIDS $pid"
    elif ! kill -0 "$ppid" 2>/dev/null; then
        # Parent is dead — orphaned
        ORPHANED_PIDS="$ORPHANED_PIDS $pid"
    else
        # Parent is alive — may be a legitimate running test
        NON_ORPHANED="$NON_ORPHANED $pid (parent $ppid)"
    fi
done <<< "$ALL_MATCHING"

ORPHAN_COUNT=$(echo "$ORPHANED_PIDS" | wc -w)

if [ -n "$NON_ORPHANED" ]; then
    echo "Processes with live parents (NOT terminating — may be an active test):"
    echo "$NON_ORPHANED"
    echo ""
fi

if [ "$ORPHAN_COUNT" -eq 0 ]; then
    echo "✓ No orphaned processes (all matching processes have live parents)."
    exit 0
fi

echo "Found $ORPHAN_COUNT orphaned process(es)."
echo ""

# --- Stage 1: SIGINT (graceful) ---
echo "--- Sending SIGINT to $ORPHAN_COUNT orphaned process(es) ---"
for pid in $ORPHANED_PIDS; do
    if kill -0 "$pid" 2>/dev/null; then
        echo "  SIGINT → PID $pid"
        kill -SIGINT "$pid" 2>/dev/null || true
    fi
done
sleep 5

# --- Stage 2: SIGTERM ---
STILL_ALIVE=""
for pid in $ORPHANED_PIDS; do
    if kill -0 "$pid" 2>/dev/null; then
        STILL_ALIVE="$STILL_ALIVE $pid"
    fi
done

if [ -n "$STILL_ALIVE" ]; then
    ALIVE_COUNT=$(echo "$STILL_ALIVE" | wc -w)
    echo ""
    echo "--- $ALIVE_COUNT process(es) survived SIGINT, sending SIGTERM ---"
    for pid in $STILL_ALIVE; do
        echo "  SIGTERM → PID $pid"
        kill -SIGTERM "$pid" 2>/dev/null || true
    done
    sleep 3
fi

# --- Stage 3: SIGKILL (last resort) ---
STILL_ALIVE=""
for pid in $ORPHANED_PIDS; do
    if kill -0 "$pid" 2>/dev/null; then
        STILL_ALIVE="$STILL_ALIVE $pid"
    fi
done

if [ -n "$STILL_ALIVE" ]; then
    ALIVE_COUNT=$(echo "$STILL_ALIVE" | wc -w)
    echo ""
    echo "--- WARNING: $ALIVE_COUNT process(es) survived SIGTERM, using SIGKILL ---"
    for pid in $STILL_ALIVE; do
        echo "  SIGKILL → PID $pid"
        kill -SIGKILL "$pid" 2>/dev/null || true
    done
    sleep 1
fi

# --- Final verification ---
REMAINING=$(ps -eo pid,ppid,cmd | grep -E "(${ORPHAN_PATTERNS})" | grep -v grep | grep -v "emergency_cleanup" | grep -v "run_clean" || true)

echo ""
echo "==================================================================="
echo "Cleanup verification"
echo "==================================================================="
if [ -z "$REMAINING" ]; then
    echo "✓ All orphaned processes terminated. System is clean."
    exit 0
else
    echo "✗ Processes still running:"
    echo "$REMAINING"
    echo ""
    echo "(These may have live parents — an active test may be running.)"
    exit 1
fi
