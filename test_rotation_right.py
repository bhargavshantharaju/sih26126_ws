#!/usr/bin/env python3
"""Test RIGHT rotation (-angular.z)"""

import subprocess
import time
import math
import sys

def get_orientation():
    try:
        result = subprocess.run(
            ['bash', '-c', 'source ~/sih26126_ws/install/setup.bash && ros2 topic echo /model/sih_ugv/odometry --once 2>/dev/null | grep -A4 "orientation:"'],
            capture_output=True,
            text=True,
            timeout=5
        )
        lines = result.stdout.strip().split('\n')
        orientation = {}
        for line in lines:
            if 'x:' in line:
                orientation['x'] = float(line.split(':')[1].strip())
            elif 'y:' in line:
                orientation['y'] = float(line.split(':')[1].strip())
            elif 'z:' in line:
                orientation['z'] = float(line.split(':')[1].strip())
            elif 'w:' in line:
                orientation['w'] = float(line.split(':')[1].strip())
        return orientation
    except Exception as e:
        return None

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q['w'] * q['z'] + q['x'] * q['y'])
    cosy_cosp = 1.0 - 2.0 * (q['y'] * q['y'] + q['z'] * q['z'])
    return math.atan2(siny_cosp, cosy_cosp)

def command_velocity(angular_z, duration):
    cmd = f'source ~/sih26126_ws/install/setup.bash && timeout {duration}s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{{angular: {{z: {angular_z}}}}}" >/dev/null 2>&1'
    subprocess.Popen(['bash', '-c', cmd])
    time.sleep(duration + 0.5)
    stop_cmd = 'source ~/sih26126_ws/install/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1'
    subprocess.run(['bash', '-c', stop_cmd])

print("Testing RIGHT rotation (-0.5 rad/s)...")
init_q = get_orientation()
init_yaw = quaternion_to_yaw(init_q)
print(f"Initial yaw: {math.degrees(init_yaw):.2f}°")

command_velocity(-0.5, 5)

final_q = get_orientation()
final_yaw = quaternion_to_yaw(final_q)
yaw_change = final_yaw - init_yaw

while yaw_change > math.pi:
    yaw_change -= 2 * math.pi
while yaw_change < -math.pi:
    yaw_change += 2 * math.pi

print(f"Final yaw: {math.degrees(final_yaw):.2f}°")
print(f"Yaw change: {math.degrees(yaw_change):.2f}°")

if yaw_change < -0.1:
    print("✅ PASS: Robot rotated RIGHT (negative yaw change) - CORRECT")
else:
    print("❌ FAIL: Robot did NOT rotate right")
