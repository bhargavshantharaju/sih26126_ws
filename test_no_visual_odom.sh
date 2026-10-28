#!/bin/bash
# DRISTHI Runtime Test #2 - Visual Odometry Disabled
# Testing hypothesis: visual odom causes EKF drift → SLAM corrections → pose jumps

set -e

TIMESTAMP=$(date -u +%Y%m%dT%H%M%S)
EVIDENCE_DIR="runtime_evidence_no_visual_${TIMESTAMP}"

echo "=================================================="
echo "DRISTHI Test #2 - Visual Odom DISABLED - ${TIMESTAMP}"
echo "=================================================="
echo ""

mkdir -p "${EVIDENCE_DIR}"

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting full_stack launch..."
echo "Modification: Visual odometry DISABLED in EKF config"
echo "Evidence directory: ${EVIDENCE_DIR}"
echo ""

ros2 launch sih26126_bringup full_stack.launch.py \
    slam:=true \
    fusion:=true \
    gui:=false \
    start_dashboard:=false \
    scenario:=showcase \
    > "${EVIDENCE_DIR}/launch.log" 2>&1 &

LAUNCH_PID=$!
echo "Launch PID: ${LAUNCH_PID}" | tee "${EVIDENCE_DIR}/launch_pid.txt"

echo "Waiting 30s for startup..."
sleep 30

if ! kill -0 ${LAUNCH_PID} 2>/dev/null; then
    echo "ERROR: Launch died"
    exit 1
fi

echo "Running 90-second test..."
sleep 90

echo "Collecting evidence..."
grep -c "Pose discontinuity" "${EVIDENCE_DIR}/launch.log" > "${EVIDENCE_DIR}/discontinuity_count.txt" || echo "0" > "${EVIDENCE_DIR}/discontinuity_count.txt"
grep "Pose discontinuity" "${EVIDENCE_DIR}/launch.log" | head -10 > "${EVIDENCE_DIR}/discontinuities.txt" 2>&1 || true
grep -E "SAFE_STOP|safety=SAFE reason=SAFE_COMMAND" "${EVIDENCE_DIR}/launch.log" | tail -20 > "${EVIDENCE_DIR}/safety_timeline.txt" 2>&1 || true

echo "Shutting down..."
kill ${LAUNCH_PID} 2>/dev/null || true
sleep 5
kill -9 ${LAUNCH_PID} 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 2

DISC_COUNT=$(cat "${EVIDENCE_DIR}/discontinuity_count.txt")
echo ""
echo "=================================================="
echo "Test #2 complete"
echo "Discontinuities: ${DISC_COUNT}"
echo "Evidence: ${EVIDENCE_DIR}"
echo "=================================================="
