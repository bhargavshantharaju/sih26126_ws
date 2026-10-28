#!/bin/bash
# DRISTHI Runtime Test - 2026-10-26
# Test the full autonomous stack after slam_pose_adapter monotonic timestamp recovery

set -e

TIMESTAMP=$(date -u +%Y%m%dT%H%M%S)
EVIDENCE_DIR="runtime_evidence_${TIMESTAMP}"

echo "=================================================="
echo "DRISTHI Runtime Test - ${TIMESTAMP}"
echo "=================================================="
echo ""

# Create evidence directory
mkdir -p "${EVIDENCE_DIR}"

# Source ROS and workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting full_stack launch..."
echo "Configuration: slam=true, fusion=true, gui=false"
echo "Evidence directory: ${EVIDENCE_DIR}"
echo ""

# Launch in background and capture PID
ros2 launch sih26126_bringup full_stack.launch.py \
    slam:=true \
    fusion:=true \
    gui:=false \
    start_dashboard:=false \
    scenario:=showcase \
    > "${EVIDENCE_DIR}/launch.log" 2>&1 &

LAUNCH_PID=$!
echo "Launch PID: ${LAUNCH_PID}" | tee "${EVIDENCE_DIR}/launch_pid.txt"

# Give nodes time to start
echo "Waiting 30s for node startup..."
sleep 30

# Check if launch is still running
if ! kill -0 ${LAUNCH_PID} 2>/dev/null; then
    echo "ERROR: Launch process died during startup"
    cat "${EVIDENCE_DIR}/launch.log" | tail -50
    exit 1
fi

echo ""
echo "Collecting baseline evidence..."

# Capture initial state
ros2 node list > "${EVIDENCE_DIR}/nodes_running.txt" 2>&1
ros2 topic list > "${EVIDENCE_DIR}/topics.txt" 2>&1
ros2 topic info -v /cmd_vel > "${EVIDENCE_DIR}/cmd_vel_info.txt" 2>&1
timeout 2 ros2 topic echo /model/sih_ugv/odometry --once 2>&1 | head -20 > "${EVIDENCE_DIR}/start_odom.txt"

echo "Running 90-second observation window..."
echo "Start time: $(date -u +%H:%M:%S)"

# Continuous monitoring during the test
for i in {1..18}; do
    sleep 5
    if ! kill -0 ${LAUNCH_PID} 2>/dev/null; then
        echo "ERROR: Launch process died at T+$((i*5))s"
        break
    fi
    
    # Sample key topics every 15s
    if [ $((i % 3)) -eq 0 ]; then
        T=$((i*5))
        echo "  T+${T}s: Sampling state..."
        timeout 1 ros2 topic echo /safety/state --once 2>&1 | grep "data:" > "${EVIDENCE_DIR}/safety_state_t${T}.txt" || echo "timeout" > "${EVIDENCE_DIR}/safety_state_t${T}.txt"
        timeout 1 ros2 topic echo /localization/pose --once 2>&1 | head -15 > "${EVIDENCE_DIR}/localization_pose_t${T}.txt" || true
        timeout 1 ros2 topic echo /model/sih_ugv/odometry --once 2>&1 | head -15 > "${EVIDENCE_DIR}/gazebo_odom_t${T}.txt" || true
    fi
done

echo ""
echo "End time: $(date -u +%H:%M:%S)"
echo "Test duration: 90 seconds"

# Final evidence collection
echo ""
echo "Collecting final evidence..."

timeout 2 ros2 topic echo /model/sih_ugv/odometry --once 2>&1 | head -20 > "${EVIDENCE_DIR}/end_odom.txt"
timeout 2 ros2 topic echo /safety/state --once > "${EVIDENCE_DIR}/final_safety_state.txt" 2>&1
timeout 2 ros2 topic echo /safety/reason --once > "${EVIDENCE_DIR}/final_safety_reason.txt" 2>&1
timeout 2 ros2 topic echo /mission/state --once > "${EVIDENCE_DIR}/final_mission_state.txt" 2>&1

# Check /cmd_vel rate
echo "Measuring /cmd_vel rate..."
timeout 5 ros2 topic hz /cmd_vel > "${EVIDENCE_DIR}/cmd_vel_hz.txt" 2>&1 &
CMD_VEL_PID=$!
sleep 3
kill ${CMD_VEL_PID} 2>/dev/null || true

# Check clock rate
echo "Measuring /clock rate..."
timeout 5 ros2 topic hz /clock > "${EVIDENCE_DIR}/clock_hz.txt" 2>&1 &
CLOCK_PID=$!
sleep 3
kill ${CLOCK_PID} 2>/dev/null || true

# Check for pose discontinuities in the log
echo "Checking for pose discontinuities..."
grep -i "pose discontinuity\|IMPLAUSIBLE_POSE_DELTA" "${EVIDENCE_DIR}/launch.log" | wc -l > "${EVIDENCE_DIR}/discontinuity_count.txt"
grep -i "pose discontinuity\|IMPLAUSIBLE_POSE_DELTA" "${EVIDENCE_DIR}/launch.log" | tail -20 > "${EVIDENCE_DIR}/discontinuities_sample.txt" 2>&1 || echo "None found" > "${EVIDENCE_DIR}/discontinuities_sample.txt"

# Shutdown
echo ""
echo "Shutting down launch..."
kill ${LAUNCH_PID} 2>/dev/null || true
sleep 3

# Force kill if still running
if kill -0 ${LAUNCH_PID} 2>/dev/null; then
    echo "Force killing launch process..."
    kill -9 ${LAUNCH_PID} 2>/dev/null || true
    sleep 2
fi

# Verify clean shutdown
REMAINING=$(ps aux | grep -E "(ros2|gazebo|gz)" | grep -v grep | wc -l)
echo "Remaining ROS/Gazebo processes: ${REMAINING}" | tee "${EVIDENCE_DIR}/remaining_processes.txt"

echo ""
echo "=================================================="
echo "Test complete. Evidence in: ${EVIDENCE_DIR}"
echo "=================================================="
echo ""
echo "Quick summary:"
echo "  Nodes: $(cat ${EVIDENCE_DIR}/nodes_running.txt 2>/dev/null | wc -l) at startup"
echo "  Discontinuities: $(cat ${EVIDENCE_DIR}/discontinuity_count.txt 2>/dev/null)"
echo "  Final safety state: $(cat ${EVIDENCE_DIR}/final_safety_state.txt 2>/dev/null | grep 'data:' || echo 'unknown')"
echo ""
