#!/usr/bin/env bash
# Simplified 90-second acceptance test for DRISTHI
# Captures odometry to verify robot motion

set -eo pipefail

WORKSPACE="/home/venom/sih26126_ws"
EVIDENCE_DIR="${WORKSPACE}/acceptance_run_$(date +%Y%m%dT%H%M%S)"
DURATION=90

mkdir -p "${EVIDENCE_DIR}"
echo "=== DRISTHI 90s Acceptance Run ===" | tee "${EVIDENCE_DIR}/summary.txt"
echo "Started: $(date)" | tee -a "${EVIDENCE_DIR}/summary.txt"

# Source environment
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null
source "${WORKSPACE}/install/setup.bash" 2>/dev/null
set -u

# Start launch in background
echo "Launching DRISTHI..." | tee -a "${EVIDENCE_DIR}/summary.txt"
ros2 launch sih26126_bringup full_stack.launch.py \
    slam:=true fusion:=true gui:=false start_dashboard:=false \
    > "${EVIDENCE_DIR}/launch.log" 2>&1 &

LAUNCH_PID=$!
echo "Launch PID: ${LAUNCH_PID}" | tee -a "${EVIDENCE_DIR}/summary.txt"

# Wait 25s for startup
echo "Waiting 25s for startup..." | tee -a "${EVIDENCE_DIR}/summary.txt"
sleep 25

if ! kill -0 ${LAUNCH_PID} 2>/dev/null; then
    echo "ERROR: Launch died" | tee -a "${EVIDENCE_DIR}/summary.txt"
    exit 1
fi

# Capture start position
echo "Capturing start position..." | tee -a "${EVIDENCE_DIR}/summary.txt"
timeout 5 ros2 topic echo /model/sih_ugv/odometry --field pose.pose.position --once > "${EVIDENCE_DIR}/start_position.txt" 2>&1 || echo "timeout" > "${EVIDENCE_DIR}/start_position.txt"

# Sample topics for 60 seconds
echo "Sampling for ${DURATION}s..." | tee -a "${EVIDENCE_DIR}/summary.txt"
START_TIME=$(date +%s)

# Sample odometry every 5s for motion tracking
for i in $(seq 1 12); do
    timeout 5 ros2 topic echo /model/sih_ugv/odometry --field pose.pose.position --once >> "${EVIDENCE_DIR}/position_log.txt" 2>&1 || echo "sample_${i}_timeout" >> "${EVIDENCE_DIR}/position_log.txt"
    sleep 5
done &

# Sample key status topics
timeout ${DURATION} ros2 topic echo /mission/state --once > "${EVIDENCE_DIR}/mission_state.txt" 2>&1 &
timeout ${DURATION} ros2 topic echo /safety/state --once > "${EVIDENCE_DIR}/safety_state.txt" 2>&1 &
timeout ${DURATION} ros2 topic echo /safety/reason --once > "${EVIDENCE_DIR}/safety_reason.txt" 2>&1 &
timeout ${DURATION} ros2 topic info /cmd_vel -v > "${EVIDENCE_DIR}/cmd_vel_info.txt" 2>&1 &
timeout ${DURATION} ros2 topic hz /cmd_vel --window 50 > "${EVIDENCE_DIR}/cmd_vel_hz.txt" 2>&1 &
timeout ${DURATION} ros2 topic hz /clock --window 100 > "${EVIDENCE_DIR}/clock_hz.txt" 2>&1 &

# Wait for sampling
wait

END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

# Capture end position
timeout 5 ros2 topic echo /model/sih_ugv/odometry --field pose.pose.position --once > "${EVIDENCE_DIR}/end_position.txt" 2>&1 || echo "timeout" > "${EVIDENCE_DIR}/end_position.txt"

# Compute distance traveled
python3 - <<'PYEOF' > "${EVIDENCE_DIR}/motion_analysis.txt" 2>&1
import sys
import math

try:
    with open("${EVIDENCE_DIR}/start_position.txt", 'r') as f:
        start_lines = [l.strip() for l in f if l.strip() and not l.startswith('-')]
    with open("${EVIDENCE_DIR}/end_position.txt", 'r') as f:
        end_lines = [l.strip() for l in f if l.strip() and not l.startswith('-')]
    
    # Parse x, y, z from lines like "x: 0.123"
    def parse_pos(lines):
        pos = {}
        for line in lines:
            if ': ' in line:
                key, val = line.split(': ', 1)
                pos[key.strip()] = float(val.strip())
        return pos
    
    start_pos = parse_pos(start_lines)
    end_pos = parse_pos(end_lines)
    
    if 'x' in start_pos and 'x' in end_pos:
        dx = end_pos['x'] - start_pos['x']
        dy = end_pos['y'] - start_pos['y']
        distance = math.sqrt(dx*dx + dy*dy)
        
        print(f"Start position: ({start_pos['x']:.3f}, {start_pos['y']:.3f})")
        print(f"End position: ({end_pos['x']:.3f}, {end_pos['y']:.3f})")
        print(f"Distance traveled: {distance:.3f} m")
        print(f"Displacement: dx={dx:.3f} m, dy={dy:.3f} m")
        
        if distance > 0.5:
            print("MOTION VERIFIED: Robot moved > 0.5m")
        else:
            print("WARNING: Robot displacement < 0.5m")
    else:
        print("ERROR: Could not parse positions")
        print(f"Start lines: {start_lines}")
        print(f"End lines: {end_lines}")
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
PYEOF

# Shutdown
echo "" | tee -a "${EVIDENCE_DIR}/summary.txt"
echo "Shutting down..." | tee -a "${EVIDENCE_DIR}/summary.txt"
if kill -0 ${LAUNCH_PID} 2>/dev/null; then
    kill -SIGINT ${LAUNCH_PID}
    sleep 5
    kill -9 ${LAUNCH_PID} 2>/dev/null || true
fi

# Summarize
echo "Finished: $(date)" | tee -a "${EVIDENCE_DIR}/summary.txt"
echo "Elapsed: ${ELAPSED}s" | tee -a "${EVIDENCE_DIR}/summary.txt"
echo "" | tee -a "${EVIDENCE_DIR}/summary.txt"
echo "=== Motion Analysis ===" | tee -a "${EVIDENCE_DIR}/summary.txt"
cat "${EVIDENCE_DIR}/motion_analysis.txt" | tee -a "${EVIDENCE_DIR}/summary.txt"
echo "" | tee -a "${EVIDENCE_DIR}/summary.txt"
echo "Evidence: ${EVIDENCE_DIR}"
