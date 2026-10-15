#!/usr/bin/env python3
"""
DRISTHI Rotation Direction Test

Tests whether the robot rotates in the correct direction in response to cmd_vel commands.

EXPECTED BEHAVIOR (ROS standard, Z-up, right-hand rule):
- angular.z > 0  →  rotate LEFT (counter-clockwise, +yaw)
- angular.z < 0  →  rotate RIGHT (clockwise, -yaw)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import sys


def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class RotationDirectionTest(Node):
    def __init__(self):
        super().__init__('rotation_direction_test')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/model/sih_ugv/odometry', self.odom_callback, 10
        )
        
        self.current_yaw = None
        self.start_yaw = None
        self.test_start_time = None
        self.test_duration = 5.0  # seconds
        self.angular_velocity = 0.5  # rad/s
        
        self.get_logger().info('Rotation Direction Test initialized')
        self.get_logger().info('Waiting for odometry...')
    
    def odom_callback(self, msg):
        """Update current yaw from odometry."""
        q = msg.pose.pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
    
    def test_rotation(self, direction: str):
        """Test rotation in specified direction (left or right)."""
        if direction == 'left':
            angular_z = self.angular_velocity
            expected_direction = 'POSITIVE (LEFT/CCW)'
        else:
            angular_z = -self.angular_velocity
            expected_direction = 'NEGATIVE (RIGHT/CW)'
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'TEST: {direction.upper()} ROTATION')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'Commanding: angular.z = {angular_z:+.2f} rad/s')
        self.get_logger().info(f'Expected: yaw should change {expected_direction}')
        
        # Wait for initial odometry
        while self.current_yaw is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.start_yaw = self.current_yaw
        self.get_logger().info(f'Initial yaw: {math.degrees(self.start_yaw):+.2f}°')
        
        # Command rotation
        cmd = Twist()
        cmd.angular.z = angular_z
        
        rate = self.create_rate(10)  # 10 Hz
        start_time = time.time()
        
        while time.time() - start_time < self.test_duration:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
        
        # Stop
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        
        # Final measurement
        rclpy.spin_once(self, timeout_sec=0.5)
        final_yaw = self.current_yaw
        yaw_change = final_yaw - self.start_yaw
        
        # Normalize to [-pi, pi]
        while yaw_change > math.pi:
            yaw_change -= 2 * math.pi
        while yaw_change < -math.pi:
            yaw_change += 2 * math.pi
        
        self.get_logger().info(f'Final yaw: {math.degrees(final_yaw):+.2f}°')
        self.get_logger().info(f'Yaw change: {math.degrees(yaw_change):+.2f}°')
        
        # Verify direction
        if direction == 'left' and yaw_change > 0.1:
            result = 'PASS ✅'
        elif direction == 'right' and yaw_change < -0.1:
            result = 'PASS ✅'
        else:
            result = 'FAIL ❌'
            self.get_logger().error(f'WRONG DIRECTION! Expected {expected_direction}')
        
        # Calculate efficiency
        expected_change = angular_z * self.test_duration
        # Normalize expected
        while expected_change > math.pi:
            expected_change -= 2 * math.pi
        while expected_change < -math.pi:
            expected_change += 2 * math.pi
        
        if abs(expected_change) > 0.1:
            efficiency = abs(yaw_change) / abs(expected_change) * 100
        else:
            efficiency = 0.0
        
        self.get_logger().info(f'Expected change: {math.degrees(expected_change):+.2f}°')
        self.get_logger().info(f'Rotation efficiency: {efficiency:.1f}%')
        self.get_logger().info(f'Result: {result}')
        
        return result == 'PASS ✅', efficiency


def main():
    rclpy.init()
    
    test = RotationDirectionTest()
    
    try:
        # Wait for system to stabilize
        test.get_logger().info('Waiting 3 seconds for system to stabilize...')
        time.sleep(3.0)
        
        # Test left rotation
        left_pass, left_eff = test.test_rotation('left')
        
        # Wait between tests
        test.get_logger().info('\nWaiting 3 seconds between tests...')
        time.sleep(3.0)
        
        # Test right rotation
        right_pass, right_eff = test.test_rotation('right')
        
        # Final summary
        test.get_logger().info(f'\n{"="*60}')
        test.get_logger().info('FINAL SUMMARY')
        test.get_logger().info(f'{"="*60}')
        test.get_logger().info(f'Left rotation:  {"PASS ✅" if left_pass else "FAIL ❌"} (efficiency: {left_eff:.1f}%)')
        test.get_logger().info(f'Right rotation: {"PASS ✅" if right_pass else "FAIL ❌"} (efficiency: {right_eff:.1f}%)')
        
        overall_pass = left_pass and right_pass
        test.get_logger().info(f'\nOVERALL: {"PASS ✅" if overall_pass else "FAIL ❌"}')
        
        if not overall_pass:
            test.get_logger().error('Rotation direction is INCORRECT')
            test.get_logger().error('Action required: Fix sign convention in robot SDF or control chain')
            sys.exit(1)
        
        avg_eff = (left_eff + right_eff) / 2.0
        if avg_eff < 80.0:
            test.get_logger().warn(f'Rotation efficiency ({avg_eff:.1f}%) is below 80% target')
            test.get_logger().warn('Action recommended: Tune mass/inertia/friction/damping')
        else:
            test.get_logger().info(f'Rotation efficiency ({avg_eff:.1f}%) is acceptable')
        
    except KeyboardInterrupt:
        test.get_logger().info('Test interrupted by user')
    finally:
        test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
