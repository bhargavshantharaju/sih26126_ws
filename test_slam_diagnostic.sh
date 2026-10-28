#!/bin/bash
# Focused SLAM diagnostic test
# Duration: 60 seconds
# Purpose: Verify scans reach SLAM and map is created

set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_DIR="$WORKSPACE_ROOT/slam_diagnostic_$(date +%Y%m%dT%H%M%S)"
mkdir -p "$TEST_DIR"

echo "=== SLAM Diagnostic Test ==="
echo "Workspace: $WORKSPACE_ROOT"
echo "Test directory: $TEST_DIR"
echo "Start time: $(date)"
echo ""

# Source ROS (disable set -u temporarily because ROS setup scripts
# reference unbound variables)
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$WORKSPACE_ROOT/install/setup.bash" 2>/dev/null || true
set -u

# Helper function: source ROS and run a command with timeout
# Usage: ros_cmd <timeout_sec> <output_file> <command...>
ros_cmd() {
    local _timeout=$1
    local _outfile=$2
    shift 2
    set +u
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    source "$WORKSPACE_ROOT/install/setup.bash" 2>/dev/null || true
    timeout "$_timeout" "$@" > "$_outfile" 2>&1 || echo "TIMEOUT" > "$_outfile"
    set -u
}

# Launch the stack in background
echo "Launching DRISTHI stack (slam=true, fusion=true, gui=false)..."
timeout 120 ros2 launch sih26126_bringup full_stack.launch.py \
    slam:=true \
    fusion:=true \
    gui:=false \
    start_dashboard:=false \
    scenario:=showcase \
    > "$TEST_DIR/launch.log" 2>&1 &

LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

# Wait for system to start (40s for WSL2 overhead)
echo "Waiting 40s for system startup..."
sleep 40

echo ""
echo "=== Collecting Diagnostics (T+40s to T+80s) ==="

# Check if processes still running
if ! ps -p $LAUNCH_PID > /dev/null 2>&1; then
    echo "ERROR: Launch process died"
    cat "$TEST_DIR/launch.log" | tail -50
    exit 1
fi

# Sample /clock
echo "Sampling /clock..."
ros_cmd 5 "$TEST_DIR/clock_sample.txt" ros2 topic echo /clock --once
head -5 "$TEST_DIR/clock_sample.txt" > "$TEST_DIR/clock_sample_tmp.txt" 2>/dev/null && mv "$TEST_DIR/clock_sample_tmp.txt" "$TEST_DIR/clock_sample.txt" || true

# Sample /lidar/scan_2d
echo "Sampling /lidar/scan_2d..."
ros_cmd 5 "$TEST_DIR/scan_sample.txt" ros2 topic echo /lidar/scan_2d --once
head -20 "$TEST_DIR/scan_sample.txt" > "$TEST_DIR/scan_sample_tmp.txt" 2>/dev/null && mv "$TEST_DIR/scan_sample_tmp.txt" "$TEST_DIR/scan_sample.txt" || true

# Check scan rate
echo "Measuring scan rate..."
ros_cmd 8 "$TEST_DIR/scan_hz.txt" ros2 topic hz /lidar/scan_2d
head -10 "$TEST_DIR/scan_hz.txt" > "$TEST_DIR/scan_hz_tmp.txt" 2>/dev/null && mv "$TEST_DIR/scan_hz_tmp.txt" "$TEST_DIR/scan_hz.txt" || true

# Sample /map
echo "Sampling /map..."
ros_cmd 5 "$TEST_DIR/map_sample.txt" ros2 topic echo /map --once
head -30 "$TEST_DIR/map_sample.txt" > "$TEST_DIR/map_sample_tmp.txt" 2>/dev/null && mv "$TEST_DIR/map_sample_tmp.txt" "$TEST_DIR/map_sample.txt" || true

# Check map dimensions and data
echo "Checking map data..."
timeout 5 ros2 topic echo /map --once 2>&1 | python3 -c "
import sys
data = sys.stdin.read()
print('=== MAP DATA ANALYSIS ===')
# Check for occupancy data
if 'data:' in data:
    # Extract the data array
    lines = data.split('data:')
    if len(lines) > 1:
        data_str = lines[1].strip()
        # Count occupied/free/unknown cells
        cells = [int(x.strip().rstrip(',')) for x in data_str.split('\n') if x.strip().rstrip('-').rstrip(',').isdigit()]
        if cells:
            total = len(cells)
            occupied = sum(1 for c in cells if c > 50)
            free_cells = sum(1 for c in cells if c >= 0 and c < 50)
            unknown = sum(1 for c in cells if c < 0 or c == 100)
            print(f'Total cells: {total}')
            print(f'Occupied (>50): {occupied}')
            print(f'Free (0-49): {free_cells}')
            print(f'Unknown (-1): {unknown}')
            print(f'Has real data: {occupied > 0 or free_cells > 0}')
        else:
            print('No numeric cell data found')
    else:
        print('No data field found')
else:
    print('No map data found - may be empty')
" > "$TEST_DIR/map_data_analysis.txt" 2>&1 || echo "MAP ANALYSIS FAILED" > "$TEST_DIR/map_data_analysis.txt"

# Check TF tree
echo "Checking TF tree..."
ros_cmd 5 "$TEST_DIR/tf_map_odom.txt" ros2 run tf2_ros tf2_echo map odom
head -15 "$TEST_DIR/tf_map_odom.txt" > "$TEST_DIR/tf_map_odom_tmp.txt" 2>/dev/null && mv "$TEST_DIR/tf_map_odom_tmp.txt" "$TEST_DIR/tf_map_odom.txt" || true
ros_cmd 5 "$TEST_DIR/tf_odom_base.txt" ros2 run tf2_ros tf2_echo odom base_link
head -15 "$TEST_DIR/tf_odom_base.txt" > "$TEST_DIR/tf_odom_base_tmp.txt" 2>/dev/null && mv "$TEST_DIR/tf_odom_base_tmp.txt" "$TEST_DIR/tf_odom_base.txt" || true

# Check /cmd_vel publisher
echo "Checking /cmd_vel publisher..."
ros_cmd 5 "$TEST_DIR/cmd_vel_info.txt" ros2 topic info /cmd_vel -v
head -20 "$TEST_DIR/cmd_vel_info.txt" > "$TEST_DIR/cmd_vel_info_tmp.txt" 2>/dev/null && mv "$TEST_DIR/cmd_vel_info_tmp.txt" "$TEST_DIR/cmd_vel_info.txt" || true

# Check localization/pose
echo "Sampling localization pose..."
ros_cmd 5 "$TEST_DIR/pose_sample.txt" ros2 topic echo /localization/pose --once
head -20 "$TEST_DIR/pose_sample.txt" > "$TEST_DIR/pose_sample_tmp.txt" 2>/dev/null && mv "$TEST_DIR/pose_sample_tmp.txt" "$TEST_DIR/pose_sample.txt" || true
ros_cmd 5 "$TEST_DIR/confidence_sample.txt" ros2 topic echo /localization/confidence --once

# Extract key log events
echo "Extracting SLAM events from launch log..."
grep -E "slam_toolbox.*Registering sensor|slam_toolbox.*Message Filter dropping|slam_activator.*ACTIVE|lidar_processor.*TF available|lidar_processor.*Discarding|slam_toolbox.*Configuring|slam_toolbox.*Activating|ekf.*odom|map.*publish" "$TEST_DIR/launch.log" > "$TEST_DIR/slam_events.txt" || echo "No matching events" > "$TEST_DIR/slam_events.txt"

# Count MessageFilter drops
echo "Counting MessageFilter drops..."
grep -c "Message Filter dropping" "$TEST_DIR/launch.log" > "$TEST_DIR/drop_count.txt" || echo "0" > "$TEST_DIR/drop_count.txt"

# Check safety state
echo "Sampling safety state..."
timeout 2 ros2 topic echo /safety/state --once 2>&1 > "$TEST_DIR/safety_state.txt" || echo "TIMEOUT" > "$TEST_DIR/safety_state.txt"
timeout 2 ros2 topic echo /safety/reason --once 2>&1 > "$TEST_DIR/safety_reason.txt" || echo "TIMEOUT" > "$TEST_DIR/safety_reason.txt"

# Wait a bit more then shutdown
echo ""
echo "Waiting additional 10s for map updates..."
sleep 10

# Final map sample
timeout 3 ros2 topic echo /map --once 2>&1 | head -30 > "$TEST_DIR/map_sample_final.txt" || echo "TIMEOUT" > "$TEST_DIR/map_sample_final.txt"

# Graceful shutdown
echo ""
echo "Shutting down..."
kill -SIGINT $LAUNCH_PID 2>/dev/null || true
sleep 3
kill -SIGTERM $LAUNCH_PID 2>/dev/null || true
sleep 2
kill -9 $LAUNCH_PID 2>/dev/null || true

echo ""
echo "=== Test Complete ==="
echo "End time: $(date)"
echo ""

# Summarize results
echo "=== Summary ==="
echo ""

echo "1. Clock:"
cat "$TEST_DIR/clock_sample.txt" | head -3

echo ""
echo "2. Scan sample:"
cat "$TEST_DIR/scan_sample.txt" | head -5

echo ""
echo "3. Scan rate:"
cat "$TEST_DIR/scan_hz.txt" | head -5

echo ""
echo "4. MessageFilter drops:"
cat "$TEST_DIR/drop_count.txt"

echo ""
echo "5. SLAM events:"
cat "$TEST_DIR/slam_events.txt" | head -10

echo ""
echo "6. Map availability (first check):"
cat "$TEST_DIR/map_sample.txt" | head -10

echo ""
echo "7. Map data analysis:"
cat "$TEST_DIR/map_data_analysis.txt" | head -10

echo ""
echo "8. Map availability (final check):"
cat "$TEST_DIR/map_sample_final.txt" | head -10

echo ""
echo "9. TF map->odom:"
cat "$TEST_DIR/tf_map_odom.txt" | head -8

echo ""
echo "10. TF odom->base_link:"
cat "$TEST_DIR/tf_odom_base.txt" | head -8

echo ""
echo "11. Localization pose:"
cat "$TEST_DIR/pose_sample.txt" | head -10

echo ""
echo "12. Localization confidence:"
cat "$TEST_DIR/confidence_sample.txt"

echo ""
echo "13. Safety state:"
cat "$TEST_DIR/safety_state.txt"

echo ""
echo "Full diagnostic data saved in: $TEST_DIR/"
echo ""
