#!/usr/bin/env python3
"""Simple rotation direction test"""

import subprocess
import time
import math
import sys

def get_orientation():
    """Get current robot orientation from odometry."""
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
        print(f"Error getting orientation: {e}")
        return None

def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (q['w'] * q['z'] + q['x'] * q['y'])
    cosy_cosp = 1.0 - 2.0 * (q['y'] * q['y'] + q['z'] * q['z'])
    return math.atan2(siny_cosp, cosy_cosp)

def command_velocity(angular_z, duration):
    """Command velocity for specified duration."""
    cmd = f'source ~/sih26126_ws/install/setup.bash && timeout {duration}s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{{angular: {{z: {angular_z}}}}}" >/dev/null 2>&1'
    subprocess.Popen(['bash', '-c', cmd])
    time.sleep(duration + 0.5)
    # Stop
    stop_cmd = 'source ~/sih26126_ws/install/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1'
    subprocess.run(['bash', '-c', stop_cmd])

print("="*60)
print("ROTATION DIRECTION TEST")
print("="*60)

# Get initial orientation
print("\n1. Getting initial orientation...")
init_q = get_orientation()
if not init_q:
    print("ERROR: Could not get initial orientation")
    sys.exit(1)

init_yaw = quaternion_to_yaw(init_q)
print(f"   Initial yaw: {math.degrees(init_yaw):.2f}°")
print(f"   Quaternion z: {init_q['z']:.4f}")

# Test LEFT rotation (+angular.z)
print("\n2. Commanding +0.5 rad/s (expect LEFT/CCW rotation)...")
command_velocity(0.5, 5)

print("\n3. Getting final orientation...")
final_q = get_orientation()
if not final_q:
    print("ERROR: Could not get final orientation")
    sys.exit(1)

final_yaw = quaternion_to_yaw(final_q)
yaw_change = final_yaw - init_yaw

# Normalize
while yaw_change > math.pi:
    yaw_change -= 2 * math.pi
while yaw_change < -math.pi:
    yaw_change += 2 * math.pi

print(f"   Final yaw: {math.degrees(final_yaw):.2f}°")
print(f"   Quaternion z: {final_q['z']:.4f}")
print(f"   Yaw change: {math.degrees(yaw_change):.2f}°")

print("\n" + "="*60)
print("RESULT:")
print("="*60)

if yaw_change > 0.1:
    print("✅ PASS: Robot rotated LEFT (positive yaw change)")
    print("   Direction is CORRECT")
elif yaw_change < -0.1:
    print("❌ FAIL: Robot rotated RIGHT (negative yaw change)")
    print("   Direction is WRONG - sign inversion detected")
else:
    print("⚠️  INCONCLUSIVE: No significant rotation detected")
    print(f"   Only {math.degrees(yaw_change):.2f}° change")

# Calculate efficiency
expected = 0.5 * 5.0  # rad/s * seconds
efficiency = abs(yaw_change) / expected * 100 if expected > 0 else 0
print(f"\nRotation efficiency: {efficiency:.1f}%")
if efficiency < 80:
    print("⚠️  Efficiency below 80% target")
else:
    print("✅ Efficiency acceptable")
