#!/bin/bash
# Test SLAM-enabled system with fixed parameters
set -euo pipefail

WORKSPACE=/home/venom/sih26126_ws
TEST_DURATION=90
TIMESTAMP=$(date +%Y%m%dT%H%M%S)
EVIDENCE_DIR="$WORKSPACE/slam_fixed_test_$TIMESTAMP"

mkdir -p "$EVIDENCE_DIR"

echo "=== DRISTHI SLAM Fixed Test ==="
echo "Duration: ${TEST_DURATION}s"
echo "Evidence: $EVIDENCE_DIR"
echo ""

# Source ROS environment
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true

# Launch system WITH SLAM (slam:=true, fusion:=true)
echo "Launching with slam:=true fusion:=true gui:=false..."
ros2 launch sih26126_bringup full_stack.launch.py \
  slam:=true \
  fusion:=true \
  gui:=false \
  start_dashboard:=false \
  scenario:=showcase \
  > "$EVIDENCE_DIR/launch.log" 2>&1 &

LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

# Wait for system to initialize
echo "Waiting 30s for system initialization (including SLAM)..."
sleep 30

# Check if launch is still running
if ! kill -0 $LAUNCH_PID 2>/dev/null; then
  echo "ERROR: Launch process died during startup"
  cat "$EVIDENCE_DIR/launch.log" | tail -50
  exit 1
fi

echo "System initialized. Running for ${TEST_DURATION}s..."

# Capture start position
timeout 5 ros2 topic echo /model/sih_ugv/odometry --once 2>/dev/null | grep -A3 "position:" > "$EVIDENCE_DIR/start_position.txt" || echo "Failed to capture start position" > "$EVIDENCE_DIR/start_position.txt"

# Sample position every 5 seconds
echo "=== Position samples (every 5s) ===" > "$EVIDENCE_DIR/position_log.txt"
for i in {1..12}; do
  sleep 5
  echo "Sample $i at $(date +%s):" >> "$EVIDENCE_DIR/position_log.txt"
  timeout 2 ros2 topic echo /model/sih_ugv/odometry --once 2>/dev/null | grep "x:" | head -1 >> "$EVIDENCE_DIR/position_log.txt" || echo "  x: <timeout>" >> "$EVIDENCE_DIR/position_log.txt"
done

echo ""
echo "Test complete. Capturing final state..."

# Capture final states
timeout 5 ros2 topic echo /safety/state --once > "$EVIDENCE_DIR/safety_state.txt" 2>&1 || echo "timeout" > "$EVIDENCE_DIR/safety_state.txt"
timeout 5 ros2 topic echo /safety/reason --once > "$EVIDENCE_DIR/safety_reason.txt" 2>&1 || echo "timeout" > "$EVIDENCE_DIR/safety_reason.txt"
timeout 5 ros2 topic echo /mission/state --once > "$EVIDENCE_DIR/mission_state.txt" 2>&1 || echo "timeout" > "$EVIDENCE_DIR/mission_state.txt"

# Check /cmd_vel publisher
timeout 5 ros2 topic info -v /cmd_vel > "$EVIDENCE_DIR/cmd_vel_info.txt" 2>&1 || echo "timeout" > "$EVIDENCE_DIR/cmd_vel_info.txt"

# Check /cmd_vel rate
timeout 10 ros2 topic hz /cmd_vel > "$EVIDENCE_DIR/cmd_vel_hz.txt" 2>&1 || echo "timeout" > "$EVIDENCE_DIR/cmd_vel_hz.txt"

# Check for pose discontinuities
echo "=== Checking for pose discontinuities ===" > "$EVIDENCE_DIR/discontinuities.txt"
DISC_COUNT=$(grep -c "discontinuity\|IMPLAUSIBLE" "$EVIDENCE_DIR/launch.log" 2>/dev/null || echo "0")
echo "Total discontinuities: $DISC_COUNT" >> "$EVIDENCE_DIR/discontinuities.txt"
if [ "$DISC_COUNT" -gt 0 ]; then
  echo "" >> "$EVIDENCE_DIR/discontinuities.txt"
  echo "First 20 discontinuities:" >> "$EVIDENCE_DIR/discontinuities.txt"
  grep -i "discontinuity\|IMPLAUSIBLE" "$EVIDENCE_DIR/launch.log" | head -20 >> "$EVIDENCE_DIR/discontinuities.txt"
fi

# Check for SLAM issues
echo "=== Checking for SLAM issues ===" > "$EVIDENCE_DIR/slam_issues.txt"
SLAM_DROPS=$(grep -c "queue is full\|dropping message" "$EVIDENCE_DIR/launch.log" 2>/dev/null || echo "0")
echo "SLAM message drops: $SLAM_DROPS" >> "$EVIDENCE_DIR/slam_issues.txt"
if [ "$SLAM_DROPS" -gt 0 ]; then
  echo "" >> "$EVIDENCE_DIR/slam_issues.txt"
  echo "Sample drops:" >> "$EVIDENCE_DIR/slam_issues.txt"
  grep -i "queue is full\|dropping message" "$EVIDENCE_DIR/launch.log" | head -10 >> "$EVIDENCE_DIR/slam_issues.txt"
fi

# Motion analysis
echo "=== Motion Analysis ===" > "$EVIDENCE_DIR/motion_analysis.txt"
echo "Start position:" >> "$EVIDENCE_DIR/motion_analysis.txt"
cat "$EVIDENCE_DIR/start_position.txt" >> "$EVIDENCE_DIR/motion_analysis.txt"
echo "" >> "$EVIDENCE_DIR/motion_analysis.txt"
echo "Position samples:" >> "$EVIDENCE_DIR/motion_analysis.txt"
cat "$EVIDENCE_DIR/position_log.txt" >> "$EVIDENCE_DIR/motion_analysis.txt"

# Shutdown
echo "Shutting down..."
kill -INT $LAUNCH_PID 2>/dev/null || true
sleep 5
kill -KILL $LAUNCH_PID 2>/dev/null || true

# Summary
echo ""
echo "=== TEST SUMMARY ==="
echo "Evidence directory: $EVIDENCE_DIR"
echo ""
echo "Safety state:"
grep "data:" "$EVIDENCE_DIR/safety_state.txt" 2>/dev/null || echo "(not captured)"
echo ""
echo "Mission state:"
grep "data:" "$EVIDENCE_DIR/mission_state.txt" 2>/dev/null || echo "(not captured)"
echo ""
echo "Pose discontinuities: $DISC_COUNT"
echo "SLAM message drops: $SLAM_DROPS"
echo ""
echo "Motion check:"
echo "First position:"
grep "x:" "$EVIDENCE_DIR/position_log.txt" 2>/dev/null | head -1 || echo "(not captured)"
echo "Last position:"
grep "x:" "$EVIDENCE_DIR/position_log.txt" 2>/dev/null | tail -1 || echo "(not captured)"

echo ""
echo "Full logs in: $EVIDENCE_DIR/launch.log"
echo "Test complete!"
