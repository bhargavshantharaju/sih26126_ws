#!/bin/bash
# Clean acceptance test with comprehensive, properly-instrumented evidence collection.
#
# Fixes from the previous version:
#   - Uses a Python evidence collector (evidence_collector.py) that subscribes to
#     all critical topics atomically, replacing unreliable ``ros2 topic echo --once``
#     polling that suffered from independent timeouts and garbled interleaving.
#   - Captures full /mission/metrics JSON (not truncated by ros2 topic echo).
#   - Records timestamped Gazebo ground-truth and EKF odometry samples and
#     computes cumulative path length and net displacement separately.
#   - Adds slam_toolbox to the shutdown cleanup list (was missing — caused
#     PID 13374 to survive in the previous run).
#   - Uses setsid for process-group tracking so all children are identified.
#   - Preflight checks for slam_toolbox orphans in addition to python/gz/bridge/ekf.
#   - Verifies /cmd_vel publisher count, sensor rates, and TF chain health.
#
# Usage:
#   ./run_clean_acceptance_test.sh [duration_seconds] [world_path]
#
set -eo pipefail

DURATION="${1:-90}"
WORLD="${2:-}"
TIMESTAMP=$(date +%Y%m%dT%H%M%S)
LOG_DIR="clean_acceptance_${TIMESTAMP}"

mkdir -p "${LOG_DIR}"

# ── Pre-flight: detect contamination ──────────────────────────────
echo "==================================================================="
echo "DRISTHI Clean Acceptance Test — ${TIMESTAMP}"
echo "==================================================================="
echo "Target: ${DURATION}s observation (≥60s continuous healthy nav required)"
echo "Log directory: ${LOG_DIR}"
echo ""

echo "Pre-flight: checking for orphaned processes..."
ORPHAN_PATTERNS="install/sih26126|gz sim|ros_gz_bridge|ekf_node|slam_toolbox|async_slam"
EXISTING=$(ps -eo pid,cmd | grep -E "(${ORPHAN_PATTERNS})" | grep -v grep | grep -v "run_clean_acceptance" || true)

if [ -n "$EXISTING" ]; then
    echo "ERROR: Found existing DRISTHI/Gazebo/SLAM processes:"
    echo "$EXISTING"
    echo ""
    echo "Run ./emergency_cleanup.sh first, or investigate manually."
    exit 1
fi
echo "✓ Environment is clean"
echo ""

# ── Launch the stack in a new process group ──────────────────────
echo "[$(date +%H:%M:%S)] Launching DRISTHI stack..."

# Source ROS environment
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

# Determine world
if [ -n "$WORLD" ]; then
    WORLD_ARG="world:=${WORLD}"
else
    WORLD_ARG=""
fi

# Record the exact launch command and world
echo "Launch command: ros2 launch sih26126_bringup autonomous_mission.launch.py gui:=false start_dashboard:=false ${WORLD_ARG}" > "${LOG_DIR}/launch_command.txt"

# Use setsid to create a new process group so we can track all descendants
setsid bash -c "
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    source install/setup.bash 2>/dev/null || true
    exec ros2 launch sih26126_bringup autonomous_mission.launch.py \
        gui:=false \
        start_dashboard:=false \
        ${WORLD_ARG}
" > "${LOG_DIR}/ros_output.log" 2>&1 &

LAUNCH_PGID=$!
echo "Launch process group leader PID: ${LAUNCH_PGID}"
echo "${LAUNCH_PGID}" > "${LOG_DIR}/launch.pid"

# ── Wait for initialization ───────────────────────────────────────
INIT_WAIT=20
echo "[$(date +%H:%M:%S)] Waiting ${INIT_WAIT}s for stack initialization..."
sleep ${INIT_WAIT}

if ! kill -0 ${LAUNCH_PGID} 2>/dev/null; then
    echo "ERROR: Launch process died during initialization"
    tail -50 "${LOG_DIR}/ros_output.log"
    exit 1
fi
echo "[$(date +%H:%M:%S)] Stack initialized. Starting evidence collection..."

# ── Record the process tree before collection ─────────────────────
ps -eo pid,pgid,cmd | grep -E "(${ORPHAN_PATTERNS})" | grep -v grep | grep -v "run_clean_acceptance" > "${LOG_DIR}/process_tree_start.txt" 2>&1 || true

# ── Run the evidence collector ────────────────────────────────────
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

COLLECTION_DURATION=$((DURATION))
python3 evidence_collector.py "${LOG_DIR}" "${COLLECTION_DURATION}" 2>&1 | tee "${LOG_DIR}/collector_output.txt"

echo ""
echo "[$(date +%H:%M:%S)] Evidence collection complete."

# ── Capture final topic list ───────────────────────────────────────
timeout 5 ros2 topic list > "${LOG_DIR}/topics.txt" 2>&1 || echo "timeout" > "${LOG_DIR}/topics.txt"

# Capture individual final states (best-effort, with timeout)
for topic_file in \
    "/mission/state:final_mission_state.txt" \
    "/safety/state:final_safety_state.txt" \
    "/exploration/state:final_exploration_state.txt" \
    "/exploration/frontier_count:final_frontier_count.txt" \
    "/exploration/goal:final_exploration_goal.txt" \
    "/clock:clock_end.txt"; do
    topic="${topic_file%%:*}"
    fname="${topic_file##*:}"
    timeout 5 ros2 topic echo "${topic}" --once > "${LOG_DIR}/${fname}" 2>&1 || echo "timeout" > "${LOG_DIR}/${fname}"
done

# ── Shutdown ──────────────────────────────────────────────────────
echo ""
echo "[$(date +%H:%M:%S)] Shutting down..."

# Record the process tree before shutdown
ps -eo pid,pgid,cmd | grep -E "(${ORPHAN_PATTERNS})" | grep -v grep | grep -v "run_clean_acceptance" > "${LOG_DIR}/process_tree_pre_shutdown.txt" 2>&1 || true

# Graceful: SIGINT to the process group (sends to all members)
echo "Sending SIGINT to process group ${LAUNCH_PGID}..."
kill -SIGINT -${LAUNCH_PGID} 2>/dev/null || true
sleep 7

# Check for survivors — scan the entire process group
SURVIVORS=$(ps -eo pid,pgid,cmd | awk -v pgid="${LAUNCH_PGID}" '$2 == pgid {print $0}' | grep -v "run_clean_acceptance" || true)

if [ -n "$SURVIVORS" ]; then
    echo "After SIGINT, survivors in process group:"
    echo "$SURVIVORS"
    echo ""
    echo "Sending SIGTERM to process group..."
    kill -SIGTERM -${LAUNCH_PGID} 2>/dev/null || true
    sleep 3

    SURVIVORS=$(ps -eo pid,pgid,cmd | awk -v pgid="${LAUNCH_PGID}" '$2 == pgid {print $0}' | grep -v "run_clean_acceptance" || true)
    if [ -n "$SURVIVORS" ]; then
        echo "After SIGTERM, survivors in process group:"
        echo "$SURVIVORS"
        echo ""
        echo "WARNING: Sending SIGKILL to process group (last resort)..."
        kill -SIGKILL -${LAUNCH_PGID} 2>/dev/null || true
        sleep 2
    fi
fi

# Final sweep: check for any stragglers by pattern (only those in our pgid)
# This catches processes that may have escaped the process group
REMAINING_BY_PATTERN=$(ps -eo pid,pgid,cmd | grep -E "(${ORPHAN_PATTERNS})" | grep -v grep | grep -v "run_clean_acceptance" | grep -v "evidence_collector" || true)

if [ -n "$REMAINING_BY_PATTERN" ]; then
    echo ""
    echo "WARNING: Processes still running after shutdown:"
    echo "$REMAINING_BY_PATTERN"
    echo "(These may be from a different session — verify before terminating.)"
    # Record but do NOT auto-kill — let the user decide
    echo "$REMAINING_BY_PATTERN" > "${LOG_DIR}/shutdown_survivors.txt"
else
    echo "✓ All test processes terminated"
fi

# Record the process tree after shutdown
ps -eo pid,pgid,cmd | grep -E "(${ORPHAN_PATTERNS})" | grep -v grep | grep -v "run_clean_acceptance" > "${LOG_DIR}/process_tree_post_shutdown.txt" 2>&1 || echo "none" > "${LOG_DIR}/process_tree_post_shutdown.txt"

echo ""
echo "[$(date +%H:%M:%S)] Shutdown complete"
echo ""

# ── Summary ───────────────────────────────────────────────────────
echo "==================================================================="
echo "Evidence collected in: ${LOG_DIR}"
echo "==================================================================="
ls -lh "${LOG_DIR}"
echo ""

# Print evidence summary if available
if [ -f "${LOG_DIR}/evidence_summary.json" ]; then
    echo "=== Evidence Summary ==="
    cat "${LOG_DIR}/evidence_summary.json"
    echo ""
fi

# Print odometry summary if available
if [ -f "${LOG_DIR}/odometry_summary.json" ]; then
    echo ""
    echo "=== Odometry Summary ==="
    cat "${LOG_DIR}/odometry_summary.json"
    echo ""
fi

if [ -f "${LOG_DIR}/shutdown_survivors.txt" ]; then
    echo ""
    echo "⚠  SHUTDOWN SURVIVORS RECORDED:"
    cat "${LOG_DIR}/shutdown_survivors.txt"
fi
