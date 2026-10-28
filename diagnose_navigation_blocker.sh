#!/usr/bin/env bash
set -euo pipefail

# Diagnose navigation blocker: monitor state transitions and sensor rates

TEST_DIR="nav_diagnostic_$(date +%Y%m%dT%H%M%S)"
mkdir -p "$TEST_DIR"

echo "=== Navigation Blocker Diagnostic ==="
echo "Test directory: $TEST_DIR"
echo "Duration: 60 seconds"
echo ""

# Source ROS
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

# Launch stack in background
echo "Launching full stack..."
timeout 70 ros2 launch sih26126_bringup full_stack.launch.py \
  slam:=true \
  fusion:=true \
  gui:=false \
  start_dashboard:=false \
  scenario:=showcase \
  > "$TEST_DIR/launch.log" 2>&1 &

LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

# Wait for startup
echo "Waiting 40s for stack startup..."
sleep 40

echo "Monitoring topics for 20s..."

# Monitor local navigation state
echo "=== Capturing local_navigation_state ===" | tee "$TEST_DIR/state_sample.txt"
timeout 20 ros2 topic echo /planner/local_navigation_state --once >> "$TEST_DIR/state_sample.txt" 2>&1 || echo "No state received" >> "$TEST_DIR/state_sample.txt"

# Check sensor rates
echo "=== Sensor Rates ===" | tee "$TEST_DIR/sensor_rates.txt"
timeout 10 ros2 topic hz /depth_camera/points --window 10 >> "$TEST_DIR/sensor_rates.txt" 2>&1 || echo "depth_camera/points: not publishing" >> "$TEST_DIR/sensor_rates.txt"
timeout 10 ros2 topic hz /lidar/scan_2d --window 10 >> "$TEST_DIR/sensor_rates.txt" 2>&1 || echo "lidar/scan_2d: not publishing" >> "$TEST_DIR/sensor_rates.txt"
timeout 10 ros2 topic hz /traversability/terrain_aware_mean_score --window 10 >> "$TEST_DIR/sensor_rates.txt" 2>&1 || echo "traversability: not publishing" >> "$TEST_DIR/sensor_rates.txt"
timeout 10 ros2 topic hz /odometry/filtered --window 10 >> "$TEST_DIR/sensor_rates.txt" 2>&1 || echo "odometry/filtered: not publishing" >> "$TEST_DIR/sensor_rates.txt"

# Sample safety state
echo "=== Safety State ===" | tee "$TEST_DIR/safety_state.txt"
timeout 5 ros2 topic echo /safety/state --once >> "$TEST_DIR/safety_state.txt" 2>&1 || echo "No safety state" >> "$TEST_DIR/safety_state.txt"
timeout 5 ros2 topic echo /safety/reason --once >> "$TEST_DIR/safety_state.txt" 2>&1 || echo "No safety reason" >> "$TEST_DIR/safety_state.txt"

# Sample cmd_vel
echo "=== cmd_vel ===" | tee "$TEST_DIR/cmd_vel_sample.txt"
timeout 5 ros2 topic echo /cmd_vel --once >> "$TEST_DIR/cmd_vel_sample.txt" 2>&1 || echo "No cmd_vel" >> "$TEST_DIR/cmd_vel_sample.txt"

echo ""
echo "Shutting down..."
kill -SIGINT $LAUNCH_PID 2>/dev/null || true
sleep 5
kill -SIGTERM $LAUNCH_PID 2>/dev/null || true
sleep 2
kill -SIGKILL $LAUNCH_PID 2>/dev/null || true

echo "=== Diagnostic Complete ==="
echo "Results in: $TEST_DIR/"
echo ""
echo "=== State Sample ==="
cat "$TEST_DIR/state_sample.txt"
echo ""
echo "=== Sensor Rates ==="
cat "$TEST_DIR/sensor_rates.txt"
echo ""
echo "=== Safety State ==="
cat "$TEST_DIR/safety_state.txt"
echo ""
echo "=== cmd_vel Sample ==="
cat "$TEST_DIR/cmd_vel_sample.txt"
