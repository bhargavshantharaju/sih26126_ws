"""Map-criteria autonomous exploration and return-home state machine."""

import json
import math
from datetime import datetime, timezone
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float32, Int32, String


def yaw_from_pose(pose):
    """Return planar yaw for a ROS pose."""
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_jump_exceeded(distance_m, elapsed_s, max_speed_mps=2.0, margin_m=0.10):
    """Flag a pose delta that cannot be explained by the simulated vehicle."""
    if elapsed_s <= 0.0:
        return distance_m > 1e-4
    return distance_m > max(0.0, max_speed_mps) * elapsed_s + max(0.0, margin_m)


def exploration_completion_ready(known_area, minimum_area, completed_frontiers,
                                 minimum_frontiers, frontier_exhausted):
    """Require measurable map coverage, real completed goals, and exhaustion."""
    return (frontier_exhausted and known_area >= minimum_area
            and completed_frontiers >= minimum_frontiers)


def should_enter_return_home(state, completion_ready):
    """Return-home can begin only from exploration states after map criteria pass."""
    return state in ('EXPLORING', 'REPLANNING') and bool(completion_ready)


def final_alignment_ready(exploration_complete, home_distance, home_tolerance,
                          heading_error, heading_tolerance, stationary, zero_command):
    """Require real exploration, home pose, aligned heading, and a stopped vehicle."""
    return (exploration_complete and home_distance <= home_tolerance
            and abs(heading_error) <= heading_tolerance
            and stationary and zero_command)


def zero_velocity_confirmed(command, tolerance=0.01):
    """Check the observed authoritative command is near zero on both axes."""
    return (command is not None and abs(command.linear.x) < tolerance
            and abs(command.angular.z) < tolerance)


def result_json(metrics):
    """Serialize a versioned mission record while retaining explicit nulls."""
    return json.dumps({'schema_version': 1, **json_safe(metrics)}, indent=2,
                      sort_keys=True, allow_nan=False) + '\n'


LIVE_METRIC_FIELDS = (
    'mission', 'mission_result', 'mission_state_final', 'failure_reason',
    'mission_duration_s', 'distance_travelled_m', 'final_position_error_m',
    'final_heading_error_rad', 'explored_area_m2', 'known_area_m2',
    'known_cells', 'free_cells', 'occupied_cells', 'unknown_cells',
    'frontier_count_final', 'number_of_frontiers_selected', 'frontiers_completed',
    'number_of_replans', 'number_of_safety_stops', 'number_of_blocked_goals',
    'minimum_obstacle_clearance_m', 'obstacle_observations', 'collision_detected',
    'contact_sensor_stream_seen', 'localization_source', 'return_home_distance_m',
    'return_home_success', 'exploration_completion_reason',
)


def live_metrics_summary(metrics):
    """Keep the dashboard telemetry small; full histories belong in the result file."""
    return {key: metrics.get(key) for key in LIVE_METRIC_FIELDS}


def json_safe(value):
    """Convert non-finite measurements to JSON null without masking finite data."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


class MissionManager(Node):
    """Capture home, track observed area, return through the live map."""

    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('minimum_explored_area', 24.0)
        self.declare_parameter('frontier_threshold', 0)
        self.declare_parameter('minimum_completed_frontiers', 3)
        self.declare_parameter('home_tolerance_m', 0.35)
        self.declare_parameter('heading_tolerance_rad', 0.20)
        self.declare_parameter('stationary_linear_mps', 0.04)
        self.declare_parameter('stationary_angular_rps', 0.06)
        self.declare_parameter('stationary_dwell_sec', 2.0)
        self.declare_parameter('mission_timeout_sec', 600.0)
        self.declare_parameter('result_directory', 'artifacts/mission_results')
        self.declare_parameter('mission_name', 'autonomous_outdoor_mission')
        self.declare_parameter('max_pose_speed_mps', 2.0)
        self.declare_parameter('pose_jump_margin_m', 0.10)
        self.declare_parameter('frontier_exhaustion_grace_sec', 3.0)
        self.declare_parameter('home_route_unavailable_timeout_sec', 3.0)
        self.pose = None
        self.raw_pose = None
        self.odom = None
        self.localization_source = None
        self.contact_stream_seen = False
        self.collision_detected = None
        self.stationary_since = None
        self.result_path = None
        self.started_at = datetime.now(timezone.utc).isoformat()
        self.finished_at = None
        self.known_cells = None
        self.free_cells = None
        self.occupied_cells = None
        self.unknown_cells = None
        self.initial_known_cells = None
        self.global_path_points = []
        self.local_path_points = []
        self.obstacle_observations = 0
        self.obstacle_near = False
        self.last_goal_reason = ''
        self.blocked_goals = 0
        self.goal_history = []
        self.trajectory = []
        self.pose_discontinuities = []
        self.pose_discontinuity_count = 0
        self.duplicate_pose_samples = 0
        self.last_pose_stamp = None
        self.last_pose_frame = None
        self.last_raw_stamp = None
        self.last_raw_time = None
        self.last_trajectory_time = 0.0
        self.last_raw_pose = None
        self.distance_travelled_raw = 0.0
        self.latest_raw_clearance = None
        self.obstacle_evidence = []
        self.goal_events = []
        self.map = None
        self.home = None
        self.goal = None
        self.frontier_count = 0
        self.frontier_state = 'MAPPING'
        self.path_status = ''
        self.safety_reason = ''
        self.safety_state = 'UNKNOWN'
        self.previous_safety = None
        self.safety_stops = 0
        self.minimum_clearance = None
        self.distance_travelled = 0.0
        self.last_pose_xy = None
        self.previous_goal = None
        self.command = None
        self.frontiers_selected = 0
        self.frontiers_completed = 0
        self.replans = 0
        self.known_area = 0.0
        self.started = time.monotonic()
        self.state = 'INITIALIZING'
        self.failure_reason = ''
        self.exploration_completion_reason = None
        self.frontier_exhausted_since = None
        self.home_route_unavailable_since = None
        self.final_alignment_error = None
        self.state_history = [{'state': self.state, 'time_sec': 0.0}]

        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.home_pub = self.create_publisher(PoseStamped, '/mission/home_pose', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/mission/goal', 10)
        self.area_pub = self.create_publisher(Float32, '/mission/known_area_m2', 10)
        self.reason_pub = self.create_publisher(String, '/mission/reason', 10)
        self.metrics_pub = self.create_publisher(String, '/mission/metrics', 10)
        self.result_path_pub = self.create_publisher(String, '/mission/result_path', 10)
        self.create_subscription(PoseStamped, '/localization/pose', self.pose_cb, 10)
        self.create_subscription(String, '/localization/source', self.source_cb, 10)
        self.create_subscription(Odometry, '/model/sih_ugv/odometry', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.command_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(Int32, '/exploration/frontier_count', self.frontier_cb, 10)
        self.create_subscription(Int32, '/exploration/frontiers_completed',
                                 self.completed_frontiers_cb, 10)
        self.create_subscription(String, '/exploration/state', self.exploration_cb, 10)
        self.create_subscription(PointStamped, '/exploration/goal', self.goal_cb, 10)
        self.create_subscription(String, '/planner/global_path_status', self.path_cb, 10)
        self.create_subscription(String, '/safety/reason', self.safety_reason_cb, 10)
        self.create_subscription(String, '/safety/state', self.safety_state_cb, 10)
        self.create_subscription(Float32, '/planner/local_obstacle_clearance',
                                 self.clearance_cb, 10)
        self.create_subscription(Float32, '/planner/raw_obstacle_clearance',
                                 self.raw_clearance_cb, 10)
        self.create_subscription(String, '/planner/obstacle_evidence',
                                 self.obstacle_evidence_cb, 10)
        self.create_subscription(String, '/exploration/goal_status',
                                 self.goal_status_cb, 10)
        self.create_subscription(String, '/exploration/goal_change_reason', self.goal_reason_cb, 10)
        self.create_subscription(Contacts, '/ugv_contacts', self.contacts_cb, 10)
        self.create_subscription(NavPath, '/planner/global_path', self.global_path_cb, 10)
        self.create_subscription(NavPath, '/planner/local_path', self.local_path_cb, 10)
        self.create_timer(0.5, self.process)

    def pose_cb(self, msg):
        self.raw_pose = msg
        arrival = time.monotonic()
        xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        frame = msg.header.frame_id
        stamp = (float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9)
        stamp_key = stamp if stamp > 0.0 else None
        if self.last_raw_pose is not None:
            self.distance_travelled_raw += math.dist(
                xy, (self.last_raw_pose.pose.position.x,
                     self.last_raw_pose.pose.position.y))
            if (stamp_key is not None and self.last_raw_stamp is not None
                    and stamp_key == self.last_raw_stamp
                    and math.dist(xy, (self.last_raw_pose.pose.position.x,
                                      self.last_raw_pose.pose.position.y)) < 1e-6):
                self.duplicate_pose_samples += 1
                return
        elapsed = (stamp_key - self.last_raw_stamp
                   if stamp_key is not None and self.last_raw_stamp is not None
                   else (arrival - self.last_raw_time if self.last_raw_time is not None else 0.0))
        if self.last_pose_xy is not None:
            delta = math.dist(xy, self.last_pose_xy)
            frame_changed = bool(self.last_pose_frame and frame
                                 and frame != self.last_pose_frame)
            if frame_changed or pose_jump_exceeded(
                    delta, elapsed,
                    float(self.get_parameter('max_pose_speed_mps').value),
                    float(self.get_parameter('pose_jump_margin_m').value)):
                self.pose_discontinuity_count += 1
                event = {
                    'time_sec': round(arrival - self.started, 3),
                    'reason': 'FRAME_CHANGED' if frame_changed else 'IMPLAUSIBLE_POSE_DELTA',
                    'frame_id': frame, 'elapsed_s': elapsed,
                    'delta_m': delta, 'raw_pose': self.pose_dict(msg),
                    'last_accepted_pose': self.pose_dict(self.pose),
                }
                self.pose_discontinuities.append(event)
                self.get_logger().error(
                    f"Pose discontinuity {self.pose_discontinuity_count}: "
                    f"{event['reason']} delta={delta:.3f} m in {elapsed:.4f} s "
                    f"frame={frame!r}; raw pose retained, metric sample rejected")
                if self.state not in ('MISSION_COMPLETE', 'FAILED', 'SAFE_STOP'):
                    self.failure_reason = 'POSE_DISCONTINUITY'
                    self.transition('SAFE_STOP')
                self.last_raw_pose = msg
                self.last_raw_stamp = stamp_key
                self.last_raw_time = arrival
                return
            self.distance_travelled += delta
        self.pose = msg
        self.last_pose_xy = xy
        self.last_pose_stamp = stamp_key
        self.last_pose_frame = frame
        self.last_raw_pose = msg
        self.last_raw_stamp = stamp_key
        self.last_raw_time = arrival

    def map_cb(self, msg):
        self.map = msg
        data = np.asarray(msg.data, dtype=np.int8)
        self.known_cells = int(np.count_nonzero(data >= 0))
        self.free_cells = int(np.count_nonzero(data == 0))
        self.occupied_cells = int(np.count_nonzero(data >= 50))
        self.unknown_cells = int(data.size - self.known_cells)
        if self.initial_known_cells is None:
            self.initial_known_cells = self.known_cells

    def source_cb(self, msg):
        self.localization_source = msg.data

    def odom_cb(self, msg):
        self.odom = msg

    def command_cb(self, msg):
        self.command = msg

    def global_path_cb(self, msg):
        self.global_path_points = [
            {'x': float(p.pose.position.x), 'y': float(p.pose.position.y)}
            for p in msg.poses]

    def local_path_cb(self, msg):
        self.local_path_points = [
            {'x': float(p.pose.position.x), 'y': float(p.pose.position.y)}
            for p in msg.poses]

    def goal_reason_cb(self, msg):
        if (msg.data == 'PROGRESS_TIMEOUT_BLACKLISTED'
                and self.last_goal_reason != msg.data):
            self.blocked_goals += 1
        self.last_goal_reason = msg.data

    def contacts_cb(self, msg):
        self.contact_stream_seen = True
        collision = bool(msg.contacts)
        self.collision_detected = bool(self.collision_detected) or collision
        if collision and self.state not in ('MISSION_COMPLETE', 'FAILED', 'SAFE_STOP'):
            self.failure_reason = 'COLLISION_DETECTED'
            self.transition('FAILED')

    def frontier_cb(self, msg):
        self.frontier_count = int(msg.data)

    def completed_frontiers_cb(self, msg):
        self.frontiers_completed = max(self.frontiers_completed, int(msg.data))

    def exploration_cb(self, msg):
        self.frontier_state = msg.data

    def goal_cb(self, msg):
        here = (float(msg.point.x), float(msg.point.y))
        self.previous_goal = here
        if self.state not in ('RETURNING_HOME', 'FINAL_ALIGNMENT', 'MISSION_COMPLETE'):
            self.goal = msg

    def path_cb(self, msg):
        self.path_status = msg.data

    def safety_reason_cb(self, msg):
        self.safety_reason = msg.data

    def safety_state_cb(self, msg):
        self.safety_state = msg.data
        if msg.data != 'SAFE' and self.previous_safety == 'SAFE':
            self.safety_stops += 1
        self.previous_safety = msg.data

    def clearance_cb(self, msg):
        value = float(msg.data)
        if math.isfinite(value) and value > 0.0:
            self.minimum_clearance = (value if self.minimum_clearance is None
                                      else min(self.minimum_clearance, value))
            near = value < 2.95  # selected-corridor clearance is capped at 3 m
            if near and not self.obstacle_near:
                self.obstacle_observations += 1
            self.obstacle_near = near

    def raw_clearance_cb(self, msg):
        value = float(msg.data)
        if math.isfinite(value) and value > 0.0:
            self.latest_raw_clearance = value

    def obstacle_evidence_cb(self, msg):
        try:
            record = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if len(self.obstacle_evidence) < 5000:
            record['time_sec'] = round(time.monotonic() - self.started, 3)
            self.obstacle_evidence.append(record)

    def goal_status_cb(self, msg):
        try:
            event = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if len(self.goal_events) < 1000:
            self.goal_events.append(event)
        if event.get('event') == 'GOAL_SELECTED':
            self.frontiers_selected = max(self.frontiers_selected,
                                          int(event.get('goal_id', 0)))
            point = event.get('goal') or {}
            if 'x' in point and 'y' in point:
                self.goal_history.append({'x': float(point['x']), 'y': float(point['y'])})
        elif event.get('event') in ('GOAL_BLACKLISTED', 'GOAL_INVALIDATED'):
            self.replans += 1

    def transition(self, state):
        if state == self.state:
            return
        self.state = state
        self.state_history.append({
            'state': state,
            'time_sec': round(time.monotonic() - self.started, 3),
        })
        self.get_logger().info(f'Mission state: {state}')
        self.state_pub.publish(String(data=state))
        if state in ('MISSION_COMPLETE', 'FAILED', 'SAFE_STOP'):
            self.write_result()

    def publish_goal(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.point.x = float(self.home.pose.position.x)
        msg.point.y = float(self.home.pose.position.y)
        self.goal_pub.publish(msg)

    def mission_metrics(self, home_distance):
        final_heading_error = (None if self.pose is None or self.home is None
                               else abs(math.atan2(
                                   math.sin(yaw_from_pose(self.pose.pose)
                                            - yaw_from_pose(self.home.pose)),
                                   math.cos(yaw_from_pose(self.pose.pose)
                                            - yaw_from_pose(self.home.pose)))))
        distance_valid = None if self.pose_discontinuity_count else self.distance_travelled
        return {
            'mission': str(self.get_parameter('mission_name').value),
            'mission_result': self.state,
            'mission_state_final': self.state,
            'mission_state_history': self.state_history,
            'failure_reason': self.failure_reason or None,
            'mission_duration_s': time.monotonic() - self.started,
            'started_at': self.started_at,
            'finished_at': self.finished_at,
            'localization_source': self.localization_source,
            'start_pose': self.pose_dict(self.home),
            'home_pose': self.pose_dict(self.home),
            'final_pose': self.pose_dict(self.pose),
            'final_raw_pose': self.pose_dict(self.raw_pose),
            'final_position_error_m': home_distance,
            'final_heading_error_rad': final_heading_error,
            'final_alignment_error': final_heading_error,
            'zero_velocity_verified': (self.command is not None
                                       and zero_velocity_confirmed(self.command)),
            'distance_travelled_valid': distance_valid,
            'distance_travelled_raw': self.distance_travelled_raw,
            'distance_travelled_m': distance_valid,
            'path_length_m': distance_valid,
            'pose_discontinuity_count': self.pose_discontinuity_count,
            'duplicate_pose_samples': self.duplicate_pose_samples,
            'pose_discontinuities': self.pose_discontinuities,
            'explored_area_m2': self.known_area,
            'known_area_m2': self.known_area,
            'known_cells': self.known_cells,
            'free_cells': self.free_cells,
            'occupied_cells': self.occupied_cells,
            'unknown_cells': self.unknown_cells,
            'initial_known_cells': self.initial_known_cells,
            'frontier_count_final': self.frontier_count,
            'minimum_obstacle_clearance_m': self.minimum_clearance,
            'minimum_planner_clearance': self.minimum_clearance,
            'minimum_planner_clearance_m': self.minimum_clearance,
            'minimum_raw_clearance': self.latest_raw_clearance,
            'minimum_raw_clearance_m': self.latest_raw_clearance,
            'obstacle_evidence': self.obstacle_evidence,
            'collision_evidence': ({
                'contact_sensor_stream_seen': self.contact_stream_seen,
                'collision_detected': self.collision_detected,
            } if self.contact_stream_seen else None),
            'goal_events': self.goal_events,
            'number_of_frontiers_selected': self.frontiers_selected,
            'frontier_goals_selected': self.frontiers_selected,
            'frontiers_completed': self.frontiers_completed,
            'frontier_goals_completed': self.frontiers_completed,
            'number_of_replans': self.replans,
            'replans': self.replans,
            'number_of_safety_stops': self.safety_stops,
            'safety_stops': self.safety_stops,
            'number_of_blocked_goals': self.blocked_goals,
            'blocked_goals': self.blocked_goals,
            'frontier_goal_history': self.goal_history,
            'trajectory': self.trajectory,
            'global_path': self.global_path_points,
            'local_path': self.local_path_points,
            'obstacle_observations': self.obstacle_observations,
            'obstacle_observation_unit': 'selected-corridor near-clearance episodes',
            'collision_detected': self.collision_detected,
            'contact_sensor_stream_seen': self.contact_stream_seen,
            'return_home_distance_m': home_distance,
            'return_home_success': self.state == 'MISSION_COMPLETE',
            'exploration_completion_reason': self.exploration_completion_reason,
            'zero_velocity_confirmed': (self.command is not None
                                        and zero_velocity_confirmed(self.command)),
            'completion_reason': self.failure_reason or (
                'HOME_TOLERANCE_AND_STATIONARY' if self.state == 'MISSION_COMPLETE' else None),
        }

    @staticmethod
    def pose_dict(msg):
        if msg is None:
            return None
        pose = msg.pose
        return {'x': float(pose.position.x), 'y': float(pose.position.y),
                'yaw': float(yaw_from_pose(pose))}

    def write_result(self):
        """Persist a terminal mission record and publish its path."""
        self.finished_at = datetime.now(timezone.utc).isoformat()
        home_distance = None
        if self.pose is not None and self.home is not None:
            home_distance = math.dist(
                (self.pose.pose.position.x, self.pose.pose.position.y),
                (self.home.pose.position.x, self.home.pose.position.y))
        directory = Path(str(self.get_parameter('result_directory').value)).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        if self.result_path is None:
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            self.result_path = str(directory / f'mission_result_{stamp}.json')
        output = Path(self.result_path)
        tmp = output.with_suffix('.json.tmp')
        tmp.write_text(result_json(self.mission_metrics(home_distance)))
        tmp.replace(output)
        self.result_path_pub.publish(String(data=self.result_path))

    def process(self):
        self.state_pub.publish(String(data=self.state))
        self.reason_pub.publish(String(data=self.failure_reason))
        if self.pose is None:
            return
        if self.home is None:
            self.home = PoseStamped()
            self.home.header = self.pose.header
            self.home.pose = self.pose.pose
            self.home_pub.publish(self.home)
            self.started = time.monotonic()
            self.started_at = datetime.now(timezone.utc).isoformat()
            self.transition('MAPPING')
            self.get_logger().info('Home pose captured from startup localization')
        self.home_pub.publish(self.home)

        now_mono = time.monotonic()
        if now_mono - self.last_trajectory_time >= 0.5:
            raw = self.pose_dict(self.raw_pose)
            accepted = self.pose_dict(self.pose)
            self.trajectory.append({
                'time_sec': round(now_mono - self.started, 3),
                'raw_pose': raw,
                'accepted_pose': accepted,
                'x': None if accepted is None else accepted['x'],
                'y': None if accepted is None else accepted['y'],
                'yaw': None if accepted is None else accepted['yaw'],
            })
            self.last_trajectory_time = now_mono

        if self.state == 'MISSION_COMPLETE':
            self.metrics_pub.publish(String(data=json.dumps(
                json_safe(self.mission_metrics(
                    0.0 if self.home is not None else None)),
                sort_keys=True, allow_nan=False)))
            return

        if self.map is not None:
            data = np.asarray(self.map.data, dtype=np.int8)
            known_cells = int(np.count_nonzero(data >= 0))
            free_cells = int(np.count_nonzero(data == 0))
            self.known_area = known_cells * float(self.map.info.resolution) ** 2
            self.area_pub.publish(Float32(data=self.known_area))
            if self.state == 'MAPPING' and free_cells > 20:
                self.transition('EXPLORING')
            min_area = float(self.get_parameter('minimum_explored_area').value)
            threshold = int(self.get_parameter('frontier_threshold').value)
            minimum_completed = int(self.get_parameter(
                'minimum_completed_frontiers').value)
            frontier_ready = self.frontier_state not in ('', 'MAPPING', 'INITIALIZING')
            exhausted_now = (frontier_ready and (
                self.frontier_state in ('NO_FRONTIER', 'NO_REACHABLE_FRONTIER')
                or self.frontier_count <= threshold))
            if exhausted_now:
                if self.frontier_exhausted_since is None:
                    self.frontier_exhausted_since = time.monotonic()
            else:
                self.frontier_exhausted_since = None
            exhaustion_grace = float(
                self.get_parameter('frontier_exhaustion_grace_sec').value)
            exhausted = (exhausted_now and self.frontier_exhausted_since is not None
                         and time.monotonic() - self.frontier_exhausted_since
                         >= exhaustion_grace)
            if (self.state == 'EXPLORING' and self.path_status in
                    ('NO_REACHABLE_FRONTIER', 'NO_PATH')):
                self.transition('REPLANNING')
            elif self.state == 'REPLANNING' and self.path_status == 'AVAILABLE':
                self.transition('EXPLORING')
            mission_searching = self.state in ('EXPLORING', 'REPLANNING')
            completion_ready = exploration_completion_ready(
                self.known_area, min_area, self.frontiers_completed,
                minimum_completed, exhausted)
            if should_enter_return_home(self.state, completion_ready):
                self.exploration_completion_reason = (
                    f"known_area_m2={self.known_area:.2f} >= {min_area:.2f}; "
                    f"completed_frontiers={self.frontiers_completed} >= {minimum_completed}; "
                    f"no_meaningful_frontier ({self.frontier_state})")
                self.transition('EXPLORATION_COMPLETE')
                self.transition('RETURNING_HOME')
                self.publish_goal()
                self.get_logger().info(
                    f'Exploration criteria met: {self.exploration_completion_reason}')
            elif mission_searching and exhausted:
                self.failure_reason = (
                    f'No reachable frontier remains; known area {self.known_area:.1f} m2 '
                    f'and {self.frontiers_completed} frontiers completed; require '
                    f'{min_area:.1f} m2 and {minimum_completed} frontiers')
                self.transition('FAILED')
                self.get_logger().error(self.failure_reason)

        home_distance = None
        if self.home is not None:
            home_distance = math.dist(
                (self.pose.pose.position.x, self.pose.pose.position.y),
                (self.home.pose.position.x, self.home.pose.position.y))
        if self.state == 'RETURNING_HOME':
            if self.path_status == 'HOME_ROUTE_UNAVAILABLE':
                if self.home_route_unavailable_since is None:
                    self.home_route_unavailable_since = time.monotonic()
                elif (time.monotonic() - self.home_route_unavailable_since >= float(
                        self.get_parameter('home_route_unavailable_timeout_sec').value)):
                    self.failure_reason = 'No safe home route exists in the current occupancy map'
                    self.transition('FAILED')
                    self.get_logger().error(self.failure_reason)
            else:
                self.home_route_unavailable_since = None
                self.publish_goal()
                if home_distance <= float(self.get_parameter('home_tolerance_m').value):
                    self.transition('FINAL_ALIGNMENT')
        elif self.state == 'FINAL_ALIGNMENT':
            error = math.atan2(math.sin(yaw_from_pose(self.home.pose)
                                        - yaw_from_pose(self.pose.pose)),
                               math.cos(yaw_from_pose(self.home.pose)
                                        - yaw_from_pose(self.pose.pose)))
            self.final_alignment_error = error
            stationary = self.odom is not None and (
                abs(self.odom.twist.twist.linear.x) <= float(
                    self.get_parameter('stationary_linear_mps').value)
                and abs(self.odom.twist.twist.angular.z) <= float(
                    self.get_parameter('stationary_angular_rps').value))
            command_zero = zero_velocity_confirmed(self.command)
            ready = final_alignment_ready(
                self.exploration_completion_reason is not None,
                home_distance,
                float(self.get_parameter('home_tolerance_m').value),
                error,
                float(self.get_parameter('heading_tolerance_rad').value),
                stationary,
                command_zero)
            if ready:
                if self.stationary_since is None:
                    self.stationary_since = time.monotonic()
                elif time.monotonic() - self.stationary_since >= float(
                        self.get_parameter('stationary_dwell_sec').value):
                    self.transition('MISSION_COMPLETE')
                    self.failure_reason = ''
                    self.get_logger().info(
                        f'MISSION_COMPLETE: home distance={home_distance:.3f} m, '
                        f'heading error={error:.3f} rad, stationary command and odometry '
                        f'held for {self.get_parameter("stationary_dwell_sec").value:.1f} s')
            else:
                self.stationary_since = None
        if self.state not in ('MISSION_COMPLETE', 'FAILED', 'SAFE_STOP') and (
                time.monotonic() - self.started >= float(
                    self.get_parameter('mission_timeout_sec').value)):
            self.failure_reason = 'MISSION_TIMEOUT'
            self.transition('FAILED')
        critical = ('STALE_LOCALIZATION', 'STALE_DEPTH', 'STALE_TRAVERSABILITY',
                    'INVALID_LOCALIZATION', 'INVALID_COMMAND',
                    'LOW_LOCALIZATION_CONFIDENCE')
        active_mission = self.state in (
            'EXPLORING', 'REPLANNING', 'RETURNING_HOME', 'FINAL_ALIGNMENT')
        if (active_mission and self.safety_state == 'STOPPED'
                and any(fault in self.safety_reason for fault in critical)):
            self.failure_reason = f'Critical safety fault: {self.safety_reason}'
            self.transition('SAFE_STOP')
        self.metrics_pub.publish(String(data=json.dumps(
            json_safe(live_metrics_summary(self.mission_metrics(home_distance))),
            sort_keys=True, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
