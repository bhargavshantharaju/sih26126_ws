#!/usr/bin/env bash
# Runtime validation script for DRISTHI autonomous UGV
# Purpose: Launch full stack and capture evidence for 60+ seconds

set -eo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EVIDENCE_DIR="${WORKSPACE_ROOT}/runtime_evidence_$(date +%Y%m%dT%H%M%S)"
DURATION_SEC=75

echo "=== DRISTHI Runtime Validation ==="
echo "Evidence directory: ${EVIDENCE_DIR}"
echo "Planned duration: ${DURATION_SEC}s"
echo ""

mkdir -p "${EVIDENCE_DIR}"

# Source ROS environment (disable strict mode temporarily for sourcing)
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "${WORKSPACE_ROOT}/install/setup.bash" 2>/dev/null || true
set -u

# Check for existing DRISTHI processes
echo "Checking for existing DRISTHI processes..."
EXISTING_PROCS=$(ps aux | grep -E 'gz sim|slam_toolbox|safety_supervisor|mission_manager' | grep -v grep | wc -l)
if [ "${EXISTING_PROCS}" -gt 0 ]; then
    echo "WARNING: Found ${EXISTING_PROCS} existing DRISTHI-related processes"
    ps aux | grep -E 'gz sim|slam_toolbox|safety_supervisor|mission_manager' | grep -v grep | head -10
    echo "Continue anyway? (y/N)"
    read -r response
    if [[ ! "${response}" =~ ^[Yy]$ ]]; then
        echo "Aborted"
        exit 1
    fi
fi

echo "Starting full stack launch..."
echo "Launch command: ros2 launch sih26126_bringup full_stack.launch.py slam:=true fusion:=true gui:=false start_dashboard:=false"
echo ""

# Launch in background and capture PID
ros2 launch sih26126_bringup full_stack.launch.py \
    slam:=true \
    fusion:=true \
    gui:=false \
    start_dashboard:=false \
    > "${EVIDENCE_DIR}/launch_output.log" 2>&1 &

LAUNCH_PID=$!
echo "Launch PID: ${LAUNCH_PID}"
echo "${LAUNCH_PID}" > "${EVIDENCE_DIR}/launch.pid"

# Wait for startup (20 seconds)
echo "Waiting 20s for startup..."
sleep 20

if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "ERROR: Launch process died during startup"
    cat "${EVIDENCE_DIR}/launch_output.log" | tail -50
    exit 1
fi

echo "Capturing runtime evidence..."

# Capture topic list
ros2 topic list > "${EVIDENCE_DIR}/topics.txt" 2>&1 || true

# Capture node list
ros2 node list > "${EVIDENCE_DIR}/nodes.txt" 2>&1 || true

# Capture /cmd_vel publishers
ros2 topic info /cmd_vel -v > "${EVIDENCE_DIR}/cmd_vel_publishers.txt" 2>&1 || true

# Capture TF tree
ros2 run tf2_tools view_frames --ros-args -p target_frame:=map > "${EVIDENCE_DIR}/tf_check.log" 2>&1 || true

# Sample key topics for 60 seconds
echo "Sampling key topics for ${DURATION_SEC}s..."
SAMPLE_START=$(date +%s)

timeout "${DURATION_SEC}" ros2 topic echo /mission/state --once > "${EVIDENCE_DIR}/mission_state_samples.txt" 2>&1 || true &
timeout "${DURATION_SEC}" ros2 topic echo /safety/state --once > "${EVIDENCE_DIR}/safety_state_sample.txt" 2>&1 || true &
timeout "${DURATION_SEC}" ros2 topic echo /safety/reason --once > "${EVIDENCE_DIR}/safety_reason_sample.txt" 2>&1 || true &
timeout "${DURATION_SEC}" ros2 topic echo /localization/confidence --once > "${EVIDENCE_DIR}/localization_confidence.txt" 2>&1 || true &
timeout "${DURATION_SEC}" ros2 topic hz /clock > "${EVIDENCE_DIR}/clock_hz.txt" 2>&1 || true &
timeout "${DURATION_SEC}" ros2 topic hz /lidar/scan_2d > "${EVIDENCE_DIR}/scan_hz.txt" 2>&1 || true &
timeout "${DURATION_SEC}" ros2 topic hz /cmd_vel > "${EVIDENCE_DIR}/cmd_vel_hz.txt" 2>&1 || true &
timeout "${DURATION_SEC}" ros2 topic hz /odometry/filtered > "${EVIDENCE_DIR}/odometry_hz.txt" 2>&1 || true &

# Monitor odometry for motion
timeout "${DURATION_SEC}" ros2 topic echo /model/sih_ugv/odometry --field pose.pose.position > "${EVIDENCE_DIR}/position_samples.txt" 2>&1 || true &

# Wait for sampling to complete
wait

SAMPLE_END=$(date +%s)
ACTUAL_DURATION=$((SAMPLE_END - SAMPLE_START))

echo ""
echo "=== Runtime Evidence Captured ==="
echo "Actual sampling duration: ${ACTUAL_DURATION}s"
echo "Evidence directory: ${EVIDENCE_DIR}"

# Graceful shutdown
echo "Shutting down launch..."
if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -SIGINT "${LAUNCH_PID}"
    sleep 5
    if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "WARNING: Launch did not terminate gracefully, forcing..."
        kill -9 "${LAUNCH_PID}" || true
    fi
else
    echo "Launch already terminated"
fi

# Check for orphaned processes
echo "Checking for orphaned DRISTHI processes..."
REMAINING=$(ps aux | grep -E 'gz sim|slam_toolbox|safety_supervisor|mission_manager|robot_localization' | grep -v grep | wc -l)
if [ "${REMAINING}" -gt 0 ]; then
    echo "WARNING: Found ${REMAINING} remaining DRISTHI processes after shutdown"
    ps aux | grep -E 'gz sim|slam_toolbox|safety_supervisor|mission_manager|robot_localization' | grep -v grep > "${EVIDENCE_DIR}/orphaned_processes.txt" || true
fi

echo ""
echo "=== Runtime Validation Complete ==="
echo "Review evidence in: ${EVIDENCE_DIR}"
