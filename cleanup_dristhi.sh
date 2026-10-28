#!/bin/bash
# DRISTHI Improved Shutdown Script
# Handles graceful and forced shutdown with process tracking

cleanup_dristhi() {
    echo "Shutting down DRISTHI processes..."
    
    # Track launch PID if provided as argument
    LAUNCH_PID=$1
    
    # Step 1: Graceful shutdown of launch process if PID known
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "  Sending SIGTERM to launch PID $LAUNCH_PID..."
        kill "$LAUNCH_PID" 2>/dev/null
        
        # Wait up to 10 seconds for graceful shutdown
        for i in {1..10}; do
            if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
                echo "  Launch process terminated gracefully"
                break
            fi
            sleep 1
        done
        
        # Force kill if still running
        if kill -0 "$LAUNCH_PID" 2>/dev/null; then
            echo "  Force killing launch PID $LAUNCH_PID..."
            kill -9 "$LAUNCH_PID" 2>/dev/null
            sleep 1
        fi
    fi
    
    # Step 2: Find and terminate any remaining ros2 launch processes
    ROS_LAUNCH_PIDS=$(pgrep -f "ros2 launch sih26126_bringup")
    if [ -n "$ROS_LAUNCH_PIDS" ]; then
        echo "  Found additional ros2 launch processes: $ROS_LAUNCH_PIDS"
        for pid in $ROS_LAUNCH_PIDS; do
            echo "    Terminating PID $pid..."
            kill "$pid" 2>/dev/null
        done
        sleep 2
        
        # Force kill remaining
        for pid in $ROS_LAUNCH_PIDS; do
            if kill -0 "$pid" 2>/dev/null; then
                echo "    Force killing PID $pid..."
                kill -9 "$pid" 2>/dev/null
            fi
        done
    fi
    
    # Step 3: Terminate Gazebo simulator (can take time to shutdown)
    GZ_PIDS=$(pgrep -f "gz sim")
    if [ -n "$GZ_PIDS" ]; then
        echo "  Terminating Gazebo simulator..."
        for pid in $GZ_PIDS; do
            kill "$pid" 2>/dev/null
        done
        sleep 3
        
        # Force kill if needed
        for pid in $GZ_PIDS; do
            if kill -0 "$pid" 2>/dev/null; then
                echo "    Force killing Gazebo PID $pid..."
                kill -9 "$pid" 2>/dev/null
            fi
        done
    fi
    
    # Step 4: Terminate ROS bridges
    BRIDGE_PIDS=$(pgrep -f "ros_gz_bridge")
    if [ -n "$BRIDGE_PIDS" ]; then
        echo "  Terminating ROS-Gazebo bridges..."
        for pid in $BRIDGE_PIDS; do
            kill -9 "$pid" 2>/dev/null
        done
    fi
    
    # Step 5: Terminate specific DRISTHI nodes that may linger
    for node_pattern in "slam_toolbox" "slam_activator" "slam_pose_adapter" "ekf_filter_node"; do
        NODE_PIDS=$(pgrep -f "$node_pattern")
        if [ -n "$NODE_PIDS" ]; then
            echo "  Terminating $node_pattern nodes..."
            for pid in $NODE_PIDS; do
                kill -9 "$pid" 2>/dev/null
            done
        fi
    done
    
    # Step 6: Wait and verify cleanup
    sleep 2
    
    # Step 7: Final verification
    REMAINING=$(ps aux | grep -E "(ros2 launch sih26126|gz sim|slam_toolbox)" | grep -v grep | wc -l)
    
    if [ "$REMAINING" -eq 0 ]; then
        echo "✓ All DRISTHI processes terminated successfully"
        return 0
    else
        echo "⚠ Warning: $REMAINING processes may still be running"
        ps aux | grep -E "(ros2 launch sih26126|gz sim|slam_toolbox)" | grep -v grep
        return 1
    fi
}

# If sourced, export the function; if executed, run cleanup
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    # Script is being executed directly
    cleanup_dristhi "$@"
else
    # Script is being sourced
    export -f cleanup_dristhi
    echo "cleanup_dristhi function loaded. Usage: cleanup_dristhi [launch_pid]"
fi
