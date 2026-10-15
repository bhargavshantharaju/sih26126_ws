#!/bin/bash

cd ~/sih26126_ws
source install/setup.bash

echo "=== ROTATION DIRECTION TEST ==="
echo

# Get initial pose
echo "Getting initial orientation..."
INIT=$(ros2 topic echo /model/sih_ugv/odometry --once 2>/dev/null | grep -A4 "orientation:")
echo "$INIT"
echo

# Test positive angular velocity (should rotate LEFT/CCW)
echo "Commanding +0.5 rad/s for 5 seconds..."
timeout 5s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" >/dev/null 2>&1 &
PID=$!
sleep 5.5
kill $PID 2>/dev/null

# Stop
echo "Stopping..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
sleep 1

# Get final pose
echo "Getting final orientation..."
FINAL=$(ros2 topic echo /model/sih_ugv/odometry --once 2>/dev/null | grep -A4 "orientation:")
echo "$FINAL"
echo

echo "ANALYSIS:"
echo "If z component INCREASED (became more positive), rotation is CORRECT (LEFT/CCW)"
echo "If z component DECREASED (became more negative), rotation is WRONG"
