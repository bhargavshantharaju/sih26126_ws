#!/bin/bash
# DRISTHI Proper Acceptance Test with Comprehensive Evidence Collection
set -e

TIMESTAMP=$(date -u +%Y%m%dT%H%M%S)
EVIDENCE_DIR="acceptance_test_${TIMESTAMP}"

echo "=========================================="
echo "DRISTHI Acceptance Test - ${TIMESTAMP}"
echo "=========================================="

mkdir -p "${EVIDENCE_DIR}"

# Source environments
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Start launch in background
echo "Starting full_stack launch..."
ros2 launch sih26126_bringup full_stack.launch.py \
    slam:=true \
    fusion:=true \
    gui:=false \
    start_dashboard:=false \
    scenario:=showcase \
    > "${EVIDENCE_DIR}/launch.log" 2>&1 &

LAUNCH_PID=$!
echo "Launch PID: ${LAUNCH_PID}" | tee "${EVIDENCE_DIR}/launch_pid.txt"

# Wait for startup
echo "Waiting 30s for startup..."
sleep 30

# Verify launch still running
if ! kill -0 ${LAUNCH_PID} 2>/dev/null; then
    echo "ERROR: Launch died during startup"
    tail -50 "${EVIDENCE_DIR}/launch.log"
    exit 1
fi

echo "Collecting baseline evidence..."

# Node list
timeout 3 ros2 node list > "${EVIDENCE_DIR}/nodes.txt" 2>&1 || echo "timeout" > "${EVIDENCE_DIR}/nodes.txt"

# Topic list
timeout 3 ros2 topic list > "${EVIDENCE_DIR}/topics.txt" 2>&1 || echo "timeout" > "${EVIDENCE_DIR}/topics.txt"

# cmd_vel publisher info
timeout 3 ros2 topic info -v /cmd_vel > "${EVIDENCE_DIR}/cmd_vel_publishers.txt" 2>&1 || echo "timeout" > "${EVIDENCE_DIR}/cmd_vel_publishers.txt"

echo "Starting 90-second observation..."
START_TIME=$(date +%s)

# Continuous evidence collection in background
{
    while kill -0 ${LAUNCH_PID} 2>/dev/null; do
        ELAPSED=$(($(date +%s) - START_TIME))
        if [ $ELAPSED -ge 90 ]; then
            break
        fi
        
        # Sample every 5 seconds
        sleep 5
        T=$ELAPSED
        
        # Ground truth odometry
        timeout 2 ros2 topic echo /model/sih_ugv/odometry --once 2>&1 | grep -A3 "position:" | head -4 >> "${EVIDENCE_DIR}/ground_truth_samples.txt" || echo "T+${T}s: timeout" >> "${EVIDENCE_DIR}/ground_truth_samples.txt"
        echo "---T+${T}s---" >> "${EVIDENCE_DIR}/ground_truth_samples.txt"
        
        # Safety state
        timeout 1 ros2 topic echo /safety/state --once 2>&1 | grep "data:" >> "${EVIDENCE_DIR}/safety_states.txt" || echo "T+${T}s: timeout" >> "${EVIDENCE_DIR}/safety_states.txt"
        
        # Safety reason
        timeout 1 ros2 topic echo /safety/reason --once 2>&1 | grep "data:" >> "${EVIDENCE_DIR}/safety_reasons.txt" || echo "T+${T}s: timeout" >> "${EVIDENCE_DIR}/safety_reasons.txt"
    done
} &
MONITOR_PID=$!

# Wait for test duration
sleep 90

# Stop monitoring
kill ${MONITOR_PID} 2>/dev/null || true
wait ${MONITOR_PID} 2>/dev/null || true

echo "Collecting final evidence..."

# Final samples
timeout 2 ros2 topic echo /mission/state --once > "${EVIDENCE_DIR}/final_mission_state.txt" 2>&1 || echo "timeout" > "${EVIDENCE_DIR}/final_mission_state.txt"

# Count queue drops
grep -c "queue is full" "${EVIDENCE_DIR}/launch.log" > "${EVIDENCE_DIR}/queue_drops.txt" 2>&1 || echo "0" > "${EVIDENCE_DIR}/queue_drops.txt"

# Count pose discontinuities  
grep -c "Pose discontinuity" "${EVIDENCE_DIR}/launch.log" > "${EVIDENCE_DIR}/pose_discontinuities.txt" 2>&1 || echo "0" > "${EVIDENCE_DIR}/pose_discontinuities.txt"

# Count SAFE_COMMAND vs STOPPED
grep -c "safety=SAFE reason=SAFE_COMMAND" "${EVIDENCE_DIR}/launch.log" > "${EVIDENCE_DIR}/safe_command_count.txt" 2>&1 || echo "0" > "${EVIDENCE_DIR}/safe_command_count.txt"
grep -c "safety=STOPPED" "${EVIDENCE_DIR}/launch.log" > "${EVIDENCE_DIR}/stopped_count.txt" 2>&1 || echo "0" > "${EVIDENCE_DIR}/stopped_count.txt"

echo "Shutting down..."

# Graceful shutdown
kill ${LAUNCH_PID} 2>/dev/null || true
sleep 5

# Force kill if needed
if kill -0 ${LAUNCH_PID} 2>/dev/null; then
    echo "Force killing launch..."
    kill -9 ${LAUNCH_PID} 2>/dev/null || true
fi

# Clean up Gazebo
sleep 2
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
sleep 2

# Verify cleanup
REMAINING=$(ps aux | grep -E "(ros2 launch sih26126|gz sim)" | grep -v grep | wc -l)
echo "Remaining processes: ${REMAINING}" | tee "${EVIDENCE_DIR}/cleanup_status.txt"

# Generate summary
{
    echo "Test Timestamp: ${TIMESTAMP}"
    echo "Duration: 90 seconds"
    echo "Queue drops: $(cat ${EVIDENCE_DIR}/queue_drops.txt)"
    echo "Pose discontinuities: $(cat ${EVIDENCE_DIR}/pose_discontinuities.txt)"
    echo "SAFE_COMMAND count: $(cat ${EVIDENCE_DIR}/safe_command_count.txt)"
    echo "STOPPED count: $(cat ${EVIDENCE_DIR}/stopped_count.txt)"
    echo "Cleanup status: ${REMAINING} remaining processes"
} | tee "${EVIDENCE_DIR}/summary.txt"

echo "=========================================="
echo "Test complete. Evidence in: ${EVIDENCE_DIR}"
echo "=========================================="
