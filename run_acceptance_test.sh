#!/bin/bash
# Controlled acceptance test for DRISTHI autonomous UGV
# Captures timestamped logs and evidence for 60+ second healthy navigation

set -eo pipefail

# Pre-flight check: detect orphaned processes from previous runs
echo "==================================================================="
echo "DRISTHI Acceptance Test Pre-Flight Check"
echo "==================================================================="
echo "Checking for orphaned processes from previous runs..."

EXISTING_DRISTHI=$(ps -eo pid,cmd | grep "install/sih26126" | grep "python3" | grep -v grep | wc -l || echo "0")
EXISTING_GAZEBO=$(ps -eo pid,cmd | grep "gz sim" | grep -v grep | wc -l || echo "0")
EXISTING_BRIDGES=$(ps -eo pid,cmd | grep "ros_gz_bridge" | grep -v grep | wc -l || echo "0")
EXISTING_EKF=$(ps -eo pid,cmd | grep "ekf_node" | grep -v grep | wc -l || echo "0")

TOTAL_EXISTING=$((EXISTING_DRISTHI + EXISTING_GAZEBO + EXISTING_BRIDGES + EXISTING_EKF))

if [ $TOTAL_EXISTING -gt 0 ]; then
    echo ""
    echo "ERROR: Found existing processes from previous test:"
    echo "  DRISTHI nodes: $EXISTING_DRISTHI"
    echo "  Gazebo: $EXISTING_GAZEBO"
    echo "  Bridges: $EXISTING_BRIDGES"
    echo "  EKF: $EXISTING_EKF"
    echo ""
    echo "Environment is contaminated. Run ./emergency_cleanup.sh first."
    echo ""
    ps -eo pid,cmd | grep -E "(install/sih26126|gz sim|ros_gz_bridge|ekf_node)" | grep -v grep || true
    echo ""
    exit 1
fi

echo "✓ Environment is clean"
echo ""

TIMESTAMP=$(date +%Y%m%dT%H%M%S)
LOG_DIR="acceptance_logs_${TIMESTAMP}"
mkdir -p "${LOG_DIR}"

echo "==================================================================="
echo "DRISTHI Acceptance Test - ${TIMESTAMP}"
echo "==================================================================="
echo "Target: 60+ seconds continuous healthy autonomous navigation"
echo "Log directory: ${LOG_DIR}"
echo ""

# Launch the full stack with logging
echo "[$(date +%H:%M:%S)] Launching DRISTHI stack..."
source install/setup.bash
ros2 launch sih26126_bringup autonomous_mission.launch.py \
    gui:=false \
    start_dashboard:=false \
    > "${LOG_DIR}/ros_output.log" 2>&1 &

LAUNCH_PID=$!
echo "Launch PID: ${LAUNCH_PID}"
echo "${LAUNCH_PID}" > "${LOG_DIR}/launch.pid"

# Wait for stack to initialize
echo "[$(date +%H:%M:%S)] Waiting 15 seconds for stack initialization..."
sleep 15

# Check if launch is still running
if ! kill -0 ${LAUNCH_PID} 2>/dev/null; then
    echo "ERROR: Launch process died during initialization"
    cat "${LOG_DIR}/ros_output.log"
    exit 1
fi

echo "[$(date +%H:%M:%S)] Stack initialized. Beginning 90-second observation period..."
echo ""

# Monitor for 90 seconds
START_TIME=$(date +%s)
DURATION=90

while true; do
    ELAPSED=$(($(date +%s) - START_TIME))
    
    if [ ${ELAPSED} -ge ${DURATION} ]; then
        echo ""
        echo "[$(date +%H:%M:%S)] Observation period complete (${DURATION}s)"
        break
    fi
    
    # Check if launch is still running
    if ! kill -0 ${LAUNCH_PID} 2>/dev/null; then
        echo ""
        echo "ERROR: Launch process died at ${ELAPSED}s"
        break
    fi
    
    # Heartbeat every 10 seconds
    if [ $((ELAPSED % 10)) -eq 0 ]; then
        echo "[$(date +%H:%M:%S)] Running... ${ELAPSED}s elapsed"
    fi
    
    sleep 1
done

echo ""
echo "[$(date +%H:%M:%S)] Capturing final state..."

# Capture topic list
timeout 5 ros2 topic list > "${LOG_DIR}/topics.txt" 2>&1 || echo "timeout" > "${LOG_DIR}/topics.txt"

# Capture mission state
timeout 5 ros2 topic echo /mission/state --once > "${LOG_DIR}/final_mission_state.txt" 2>&1 || echo "timeout" > "${LOG_DIR}/final_mission_state.txt"

# Capture mission metrics
timeout 5 ros2 topic echo /mission/metrics --once > "${LOG_DIR}/final_metrics.txt" 2>&1 || echo "timeout" > "${LOG_DIR}/final_metrics.txt"

# Capture safety state
timeout 5 ros2 topic echo /safety/state --once > "${LOG_DIR}/final_safety_state.txt" 2>&1 || echo "timeout" > "${LOG_DIR}/final_safety_state.txt"

# Capture exploration state
timeout 5 ros2 topic echo /exploration/state --once > "${LOG_DIR}/final_exploration_state.txt" 2>&1 || echo "timeout" > "${LOG_DIR}/final_exploration_state.txt"

echo "[$(date +%H:%M:%S)] Shutting down..."

# Enumerate all children of the launch process BEFORE terminating
# This is critical because launch may exit without killing children
CHILD_PIDS=$(pgrep -P ${LAUNCH_PID} 2>/dev/null || true)
CHILD_COUNT=$(echo "$CHILD_PIDS" | grep -c . || echo "0")
echo "Launch process has $CHILD_COUNT direct children"

# Also find all DRISTHI processes that might be grandchildren
ALL_DRISTHI=$(ps -eo pid,cmd | grep "install/sih26126" | grep "python3" | awk '{print $1}' || true)
DRISTHI_COUNT=$(echo "$ALL_DRISTHI" | grep -c . || echo "0")
echo "Found $DRISTHI_COUNT total DRISTHI processes"

# Send SIGINT to launch process
kill -SIGINT ${LAUNCH_PID} 2>/dev/null || true
echo "Sent SIGINT to launch PID ${LAUNCH_PID}, waiting 7 seconds..."
sleep 7

# Check if launch is still running
if kill -0 ${LAUNCH_PID} 2>/dev/null; then
    echo "Launch still running, sending SIGTERM..."
    kill -SIGTERM ${LAUNCH_PID} 2>/dev/null || true
    sleep 3
    
    if kill -0 ${LAUNCH_PID} 2>/dev/null; then
        echo "WARNING: Using SIGKILL on launch (last resort)"
        kill -SIGKILL ${LAUNCH_PID} 2>/dev/null || true
    fi
else
    echo "Launch process exited"
fi

# Now check for orphaned children
REMAINING=$(ps -eo pid,cmd | grep "install/sih26126" | grep "python3" | awk '{print $1}' || true)
REMAINING_COUNT=$(echo "$REMAINING" | grep -c . || echo "0")

if [ $REMAINING_COUNT -gt 0 ]; then
    echo "WARNING: $REMAINING_COUNT DRISTHI processes still running after launch exit"
    echo "Terminating orphaned children..."
    
    # Try SIGINT first
    for pid in $REMAINING; do
        if kill -0 $pid 2>/dev/null; then
            kill -SIGINT $pid 2>/dev/null || true
        fi
    done
    
    sleep 5
    
    # Check what's still alive
    STILL_ALIVE=$(ps -eo pid,cmd | grep "install/sih26126" | grep "python3" | awk '{print $1}' || true)
    ALIVE_COUNT=$(echo "$STILL_ALIVE" | grep -c . || echo "0")
    
    if [ $ALIVE_COUNT -gt 0 ]; then
        echo "WARNING: $ALIVE_COUNT processes did not respond to SIGINT, using SIGTERM..."
        for pid in $STILL_ALIVE; do
            if kill -0 $pid 2>/dev/null; then
                kill -SIGTERM $pid 2>/dev/null || true
            fi
        done
        
        sleep 3
        
        # Last check
        FINAL=$(ps -eo pid,cmd | grep "install/sih26126" | grep "python3" | awk '{print $1}' || true)
        FINAL_COUNT=$(echo "$FINAL" | grep -c . || echo "0")
        
        if [ $FINAL_COUNT -gt 0 ]; then
            echo "ERROR: $FINAL_COUNT processes still running, using SIGKILL (last resort)"
            for pid in $FINAL; do
                if kill -0 $pid 2>/dev/null; then
                    kill -SIGKILL $pid 2>/dev/null || true
                fi
            done
        fi
    fi
fi

# Also check for orphaned Gazebo and bridge processes
GAZEBO_PID=$(ps -eo pid,cmd | grep "gz sim" | grep -v grep | awk '{print $1}' || true)
if [ -n "$GAZEBO_PID" ]; then
    echo "Terminating Gazebo (PID $GAZEBO_PID)..."
    kill -SIGTERM $GAZEBO_PID 2>/dev/null || true
    sleep 2
    if kill -0 $GAZEBO_PID 2>/dev/null; then
        kill -SIGKILL $GAZEBO_PID 2>/dev/null || true
    fi
fi

BRIDGE_PIDS=$(ps -eo pid,cmd | grep "ros_gz_bridge" | grep -v grep | awk '{print $1}' || true)
if [ -n "$BRIDGE_PIDS" ]; then
    echo "Terminating bridge processes..."
    for pid in $BRIDGE_PIDS; do
        kill -SIGTERM $pid 2>/dev/null || true
    done
    sleep 1
    for pid in $BRIDGE_PIDS; do
        if kill -0 $pid 2>/dev/null; then
            kill -SIGKILL $pid 2>/dev/null || true
        fi
    done
fi

# EKF nodes sometimes persist
EKF_PID=$(ps -eo pid,cmd | grep "ekf_node" | grep -v grep | awk '{print $1}' || true)
if [ -n "$EKF_PID" ]; then
    echo "Terminating EKF node..."
    for pid in $EKF_PID; do
        kill -SIGTERM $pid 2>/dev/null || true
    done
    sleep 1
    for pid in $EKF_PID; do
        if kill -0 $pid 2>/dev/null; then
            kill -SIGKILL $pid 2>/dev/null || true
        fi
    done
fi

echo "[$(date +%H:%M:%S)] Shutdown complete"

# Final verification
FINAL_CHECK=$(ps -eo pid,cmd | grep -E "(install/sih26126|gz sim|ekf_node)" | grep -v grep | wc -l || echo "0")
if [ $FINAL_CHECK -eq 0 ]; then
    echo "✓ All test processes terminated cleanly"
else
    echo "✗ WARNING: $FINAL_CHECK processes still running:"
    ps -eo pid,cmd | grep -E "(install/sih26126|gz sim|ekf_node)" | grep -v grep || true
fi
echo ""
echo "==================================================================="
echo "Evidence collected in: ${LOG_DIR}"
echo "==================================================================="
ls -lh "${LOG_DIR}"
echo ""
