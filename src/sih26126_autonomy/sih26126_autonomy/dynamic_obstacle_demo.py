"""Deterministic simulation-only mover and runtime recorder for Objective 4."""

import json
import math
import os
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage


def obstacle_y(elapsed: float) -> float:
    """Wait, move into route, hold, then clear it at a fixed speed."""
    if elapsed < 8.0:
        return -2.0
    if elapsed < 16.0:
        return -2.0 + 0.25 * (elapsed - 8.0)
    if elapsed < 28.0:
        return 0.0
    return min(2.0, 0.25 * (elapsed - 28.0))


class DynamicObstacleController(Node):
    """Move the physical Gazebo model using only its simulation actuator API."""

    def __init__(self):
        super().__init__('objective4_obstacle_controller')
        self.client = self.create_client(
            SetEntityPose, '/world/dynamic_obstacle_demo/set_pose')
        self.declare_parameter(
            'trajectory_log', '/tmp/objective4_obstacle_trajectory.jsonl')
        self.trajectory_log = str(self.get_parameter('trajectory_log').value)
        with open(self.trajectory_log, 'w', encoding='utf-8'):
            pass
        self.start_time = None
        self.pending = False
        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return
        if self.start_time is None:
            self.start_time = now
        if self.pending or not self.client.service_is_ready():
            return
        elapsed = now - self.start_time
        request = SetEntityPose.Request()
        request.entity = Entity(name='dynamic_obstacle', type=Entity.MODEL)
        request.pose.position.x = 3.2
        request.pose.position.y = obstacle_y(elapsed)
        request.pose.position.z = 0.55
        request.pose.orientation.w = 1.0
        self.pending = True
        y = obstacle_y(elapsed)
        future = self.client.call_async(request)
        future.add_done_callback(lambda result: self.completed(result, elapsed, y))

    def completed(self, future, elapsed, y):
        self.pending = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error('Gazebo rejected dynamic obstacle pose')
            else:
                with open(self.trajectory_log, 'a', encoding='utf-8') as stream:
                    stream.write(json.dumps({
                        'monotonic_s': time.monotonic(),
                        'sim_elapsed_s': elapsed,
                        'x': 3.2, 'y': y, 'z': 0.55,
                        'service_success': True,
                    }) + '\n')
        except Exception as exc:  # surface transport failures in runtime logs
            self.get_logger().error(f'obstacle pose service failed: {exc}')


class DemoRecorder(Node):
    """Collect sensor/planner/safety telemetry; GT is isolated to evaluation."""

    def __init__(self, duration):
        super().__init__('objective4_demo_recorder')
        self.duration = duration
        self.started = time.monotonic()
        self.started_monotonic = self.started
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self.lock = threading.Lock()
        self.samples = []
        self.tf_edges = set()
        self.collision = False
        self.scan_live = False
        self.slam_seen = False
        self.ekf_seen = False
        self.map_live = False
        self.detected_at = None
        self.min_scan = math.inf
        self.latest_front_scan_min = math.inf
        self.latest_global_scan_min = math.inf
        self.latest_front_scan_count = 0
        self.min_gt_distance = math.inf
        self.cmd_vel_publishers = set()
        self.latest_robot_pose = None
        self.robot_pose_samples = []
        self.latest_clearance = None
        self.latest_steering_angle_deg = None
        self.latest_local_state = ''
        self.latest_safety = ''
        self.latest_reason = ''
        self.first_motion_at = None
        self.route_publisher = self.create_publisher(Path, '/planner/global_path', 10)
        self.goal_publisher = self.create_publisher(PointStamped, '/exploration/goal', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 20)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 20)
        self.create_subscription(Float32, '/planner/local_obstacle_clearance',
                                 self.clearance_callback, 20)
        self.create_subscription(Float32, '/planner/local_steering_angle',
                                 self.steering_callback, 20)
        self.create_subscription(String, '/planner/local_navigation_state',
                                 self.local_state_callback, 20)
        self.create_subscription(String, '/safety/state', self.safety_callback, 20)
        self.create_subscription(String, '/safety/reason', self.reason_callback, 20)
        self.create_subscription(Odometry, '/model/sih_ugv/odometry',
                                 self.odom_callback, 20)
        self.create_subscription(Contacts, '/ugv_contacts', self.contacts_callback, 20)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 50)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(String, '/fusion/state',
                                 self.fusion_callback, 10)
        self.route_timer = self.create_timer(0.5, self.publish_route)
        self.sample_timer = self.create_timer(0.2, self.sample)

    def publish_route(self):
        """Request a straight 6 m route through existing planners and safety."""
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = 'odom'
        for i in range(25):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(i) * 0.25
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.route_publisher.publish(path)
        goal = PointStamped()
        goal.header = path.header
        goal.point.x = 6.0
        self.goal_publisher.publish(goal)

    def scan_callback(self, msg):
        values = []
        all_values = []
        for i, raw in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            distance = float(raw)
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                all_values.append(distance)
                if abs(angle) <= 0.42:
                    values.append(distance)
        self.scan_live = True
        self.latest_front_scan_count = len(values)
        self.latest_front_scan_min = min(values, default=math.inf)
        self.latest_global_scan_min = min(all_values, default=math.inf)
        if values:
            nearest = self.latest_front_scan_min
            if nearest <= 3.0:
                self.min_scan = min(self.min_scan, nearest)
            if nearest <= 2.5 and self.detected_at is None:
                self.detected_at = time.monotonic() - self.started

    def command_callback(self, msg):
        now = time.monotonic() - self.started
        if msg.linear.x > 0.03 and self.first_motion_at is None:
            self.first_motion_at = now
        self.latest_command = float(msg.linear.x)

    def clearance_callback(self, msg):
        self.latest_clearance = float(msg.data)

    def steering_callback(self, msg):
        self.latest_steering_angle_deg = float(msg.data)

    def local_state_callback(self, msg):
        self.latest_local_state = msg.data

    def safety_callback(self, msg):
        self.latest_safety = msg.data

    def reason_callback(self, msg):
        self.latest_reason = msg.data

    def fusion_callback(self, msg):
        self.ekf_seen = msg.data == 'HEALTHY'

    def map_callback(self, _msg):
        self.map_live = True
        self.slam_seen = True

    def odom_callback(self, msg):
        # Post-hoc evaluation only: this subscription is not used by autonomy.
        p = msg.pose.pose.position
        now = time.monotonic()
        self.latest_robot_pose = (float(p.x), float(p.y))
        self.robot_pose_samples.append((now, float(p.x), float(p.y)))

    def contacts_callback(self, msg):
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any('dynamic_obstacle' in name for name in names) and any(
                    'sih_ugv' in name for name in names):
                self.collision = True

    def tf_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            self.tf_edges.add((parent, child))

    def sample(self):
        elapsed = time.monotonic() - self.started
        self.cmd_vel_publishers.update(
            endpoint.node_name
            for endpoint in self.get_publishers_info_by_topic('/cmd_vel'))
        self.samples.append({
            'time_s': round(elapsed, 3),
            'scan_clearance_m': (None if not math.isfinite(self.min_scan)
                                 else round(self.min_scan, 3)),
            'scan_front_nearest_m': (None if not math.isfinite(self.latest_front_scan_min)
                                     else round(self.latest_front_scan_min, 3)),
            'scan_global_nearest_m': (None if not math.isfinite(self.latest_global_scan_min)
                                      else round(self.latest_global_scan_min, 3)),
            'scan_front_valid_returns': self.latest_front_scan_count,
            'cmd_vel_linear_x_mps': getattr(self, 'latest_command', 0.0),
            'local_clearance_m': self.latest_clearance,
            'local_steering_angle_deg': self.latest_steering_angle_deg,
            'local_state': self.latest_local_state,
            'safety_state': self.latest_safety,
            'safety_reason': self.latest_reason,
        })

    def artifact(self):
        detected = self.detected_at is not None
        trajectory_path = '/tmp/objective4_obstacle_trajectory.jsonl'
        obstacle_samples = []
        try:
            with open(trajectory_path, encoding='utf-8') as stream:
                obstacle_samples = [json.loads(line) for line in stream if line.strip()]
        except FileNotFoundError:
            pass
        if obstacle_samples:
            actuator_start = obstacle_samples[0]['monotonic_s']
            self.obstacle_cross_start_s = actuator_start + 8.0 - self.started_monotonic
            self.obstacle_clear_time_s = actuator_start + 36.0 - self.started_monotonic
        else:
            self.obstacle_cross_start_s = None
            self.obstacle_clear_time_s = None
        before = [s['cmd_vel_linear_x_mps'] for s in self.samples
                  if self.obstacle_cross_start_s is not None
                  and self.obstacle_cross_start_s - 3 <= s['time_s'] < self.obstacle_cross_start_s]
        encounter = [s['cmd_vel_linear_x_mps'] for s in self.samples
                     if self.obstacle_cross_start_s is not None
                     and self.obstacle_cross_start_s <= s['time_s'] < self.obstacle_clear_time_s]
        after = [s['cmd_vel_linear_x_mps'] for s in self.samples
                 if self.obstacle_clear_time_s is not None
                 and self.obstacle_clear_time_s <= s['time_s'] < self.obstacle_clear_time_s + 8]
        min_gt_distance = math.inf
        for obstacle in obstacle_samples:
            if not obstacle.get('service_success') or not self.robot_pose_samples:
                continue
            robot = min(self.robot_pose_samples,
                        key=lambda sample: abs(sample[0] - obstacle['monotonic_s']))
            min_gt_distance = min(
                min_gt_distance,
                math.hypot(robot[1] - obstacle['x'], robot[2] - obstacle['y']))
        self.min_gt_distance = min_gt_distance
        return {
            'objective': 4,
            'started_utc': self.started_utc,
            'test_duration_s': round(time.monotonic() - self.started, 2),
            'obstacle_detected': detected,
            'obstacle_detection_time_s': self.detected_at,
            'obstacle_route_entry_time_s': self.obstacle_cross_start_s,
            'obstacle_route_clear_time_s': self.obstacle_clear_time_s,
            'minimum_lidar_observed_clearance_m': (None if not math.isfinite(self.min_scan)
                                                   else self.min_scan),
            'commanded_speed_before_mps': max(before, default=0.0),
            'commanded_speed_during_mps': min(encounter, default=0.0),
            'commanded_speed_after_mps': max(after, default=0.0),
            'planner_or_safety_output_changed': any(
                s['local_state'] == 'LOCAL_STOP_OBSTACLE'
                or s['safety_reason'] == 'OBSTACLE_STOP' for s in self.samples),
            'planner_or_safety_changed_during_encounter': any(
                (s['local_state'] == 'LOCAL_STOP_OBSTACLE'
                 or s['safety_reason'] == 'OBSTACLE_STOP')
                and detected
                and self.obstacle_cross_start_s is not None
                and self.obstacle_cross_start_s <= s['time_s'] <= self.obstacle_clear_time_s
                for s in self.samples),
            'vehicle_stopped': any(
                abs(s['cmd_vel_linear_x_mps']) < 0.01
                and self.obstacle_cross_start_s is not None
                and self.obstacle_cross_start_s <= s['time_s'] <= self.obstacle_clear_time_s
                for s in self.samples),
            'vehicle_deviated_around_obstacle': False,
            'vehicle_moved': self.first_motion_at is not None,
            'first_motion_time_s': self.first_motion_at,
            'vehicle_stop_cause_obstacle': detected and any(
                s['safety_reason'] == 'OBSTACLE_STOP'
                and self.obstacle_cross_start_s is not None
                and self.obstacle_cross_start_s <= s['time_s'] <= self.obstacle_clear_time_s
                for s in self.samples),
            'minimum_ugv_obstacle_ground_truth_distance_m_posthoc': (
                None if not math.isfinite(self.min_gt_distance) else self.min_gt_distance),
            'collision': self.collision,
            'scan_live': self.scan_live,
            'slam_tf_edge_seen': ('map', 'odom') in self.tf_edges,
            'ekf_tf_edge_seen': ('odom', 'base_link') in self.tf_edges,
            'slam_status_seen': self.slam_seen,
            'ekf_status_seen': self.ekf_seen,
            'slam_map_live': self.map_live,
            'cmd_vel_publisher_contract': 'safety_supervisor only',
            'cmd_vel_publishers_observed': sorted(self.cmd_vel_publishers),
            'ground_truth_usage': 'post-hoc distance evaluation only',
            'samples': self.samples,
            'obstacle_pose_service_samples': obstacle_samples,
        }


def run_demo(args=None):
    """Launch full stack with slam+fusion and write measured JSON evidence."""
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=50.0)
    parser.add_argument('--gui', choices=('true', 'false'), default='true')
    parser.add_argument('--output', default='artifacts/objective4/dynamic_obstacle.json')
    options = parser.parse_args(args)
    if not rclpy.ok():
        rclpy.init()
    node = DemoRecorder(options.duration)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    bringup = get_package_share_directory('sih26126_bringup')
    sim = get_package_share_directory('sih26126_sim')
    world = os.path.join(sim, 'worlds', 'dynamic_obstacle_demo.sdf')
    command = [
        'ros2', 'launch', 'sih26126_bringup', 'full_stack.launch.py',
        f'world:={world}', 'slam:=true', 'fusion:=true',
        f'gui:={options.gui}', 'start_dashboard:=true',
        'dynamic_obstacle_demo:=true', 'mission_timeout_sec:=120.0',
    ]
    with open('/tmp/objective4_obstacle_trajectory.jsonl', 'w', encoding='utf-8'):
        pass
    launch = subprocess.Popen(command, cwd=bringup, start_new_session=True)
    node.get_logger().info('Running: ' + ' '.join(command))
    try:
        deadline = time.monotonic() + options.duration
        while time.monotonic() < deadline and launch.poll() is None:
            time.sleep(0.2)
        if launch.poll() is not None:
            node.get_logger().error(f'full_stack exited with {launch.returncode}')
    except KeyboardInterrupt:
        pass
    finally:
        if launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                launch.wait(timeout=5)
        artifact = node.artifact()
        artifact['launch_command'] = ' '.join(command)
        os.makedirs(os.path.dirname(options.output), exist_ok=True)
        with open(options.output, 'w', encoding='utf-8') as stream:
            json.dump(artifact, stream, indent=2, sort_keys=True)
            stream.write('\n')
        node.get_logger().info(f'Wrote runtime evidence to {options.output}')
        executor.shutdown(timeout_sec=2)
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2)
    return 0


def controller_main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    return run_demo(args)
