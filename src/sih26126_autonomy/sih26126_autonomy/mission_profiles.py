#!/usr/bin/env python3
"""
Mission Profile System - Reusable mission behaviors.

Supports multiple mission types using the same navigation infrastructure:
- EXPLORATION: Frontier-based autonomous exploration
- WAYPOINT: Navigate through sequence of waypoints
- COVERAGE: Systematic area coverage
- SEARCH: Pattern-based search
- RETURN_HOME: Return to start position

All modes reuse existing navigation, planning, safety, and localization.
"""

import math
from enum import Enum
from typing import List, Optional, Tuple
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import String, Int32
import numpy as np


class MissionType(Enum):
    """Supported mission types."""
    EXPLORATION = "EXPLORATION"
    WAYPOINT = "WAYPOINT"
    COVERAGE = "COVERAGE"
    SEARCH = "SEARCH"
    RETURN_HOME = "RETURN_HOME"


class MissionState(Enum):
    """Mission execution states."""
    INITIALIZING = "INITIALIZING"
    ACTIVE = "ACTIVE"
    WAYPOINT_TRANSIT = "WAYPOINT_TRANSIT"
    REPLANNING = "REPLANNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


@dataclass
class Waypoint:
    """Single waypoint in a mission."""
    x: float
    y: float
    z: float = 0.0
    tolerance: float = 0.5
    name: str = ""


@dataclass
class MissionProfile:
    """Complete mission profile configuration."""
    mission_type: MissionType
    waypoints: List[Waypoint]
    coverage_bounds: Optional[Tuple[float, float, float, float]] = None  # (x_min, x_max, y_min, y_max)
    coverage_spacing: float = 1.0  # meters between coverage lines
    search_pattern: str = "GRID"  # GRID, SPIRAL, LAWNMOWER
    search_bounds: Optional[Tuple[float, float, float, float]] = None
    search_spacing: float = 2.0
    return_to_start: bool = True


def generate_coverage_waypoints(
    bounds: Tuple[float, float, float, float],
    spacing: float,
    start_x: float,
    start_y: float
) -> List[Waypoint]:
    """
    Generate lawnmower coverage pattern waypoints.
    
    Args:
        bounds: (x_min, x_max, y_min, y_max)
        spacing: Distance between parallel coverage lines
        start_x, start_y: Starting position
    
    Returns:
        List of waypoints forming lawnmower pattern
    """
    x_min, x_max, y_min, y_max = bounds
    waypoints = []
    
    # Start from nearest corner
    y = y_min
    direction = 1  # 1 for left-to-right, -1 for right-to-left
    line_count = 0
    
    while y <= y_max:
        if direction == 1:
            # Left to right
            waypoints.append(Waypoint(x_min, y, name=f"coverage_{line_count}_start"))
            waypoints.append(Waypoint(x_max, y, name=f"coverage_{line_count}_end"))
        else:
            # Right to left
            waypoints.append(Waypoint(x_max, y, name=f"coverage_{line_count}_start"))
            waypoints.append(Waypoint(x_min, y, name=f"coverage_{line_count}_end"))
        
        direction *= -1  # Alternate direction
        y += spacing
        line_count += 1
    
    return waypoints


def generate_grid_search_waypoints(
    bounds: Tuple[float, float, float, float],
    spacing: float
) -> List[Waypoint]:
    """
    Generate grid search pattern waypoints.
    
    Args:
        bounds: (x_min, x_max, y_min, y_max)
        spacing: Distance between search points
    
    Returns:
        List of waypoints forming grid pattern
    """
    x_min, x_max, y_min, y_max = bounds
    waypoints = []
    
    y = y_min
    row = 0
    while y <= y_max:
        x = x_min
        col = 0
        while x <= x_max:
            waypoints.append(Waypoint(x, y, name=f"search_{row}_{col}"))
            x += spacing
            col += 1
        y += spacing
        row += 1
    
    return waypoints


def generate_spiral_search_waypoints(
    center_x: float,
    center_y: float,
    max_radius: float,
    spacing: float
) -> List[Waypoint]:
    """
    Generate spiral search pattern waypoints.
    
    Args:
        center_x, center_y: Center of spiral
        max_radius: Maximum search radius
        spacing: Distance between spiral arms
    
    Returns:
        List of waypoints forming spiral pattern
    """
    waypoints = []
    angle = 0.0
    radius = 0.0
    point_count = 0
    
    while radius < max_radius:
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        waypoints.append(Waypoint(x, y, name=f"spiral_{point_count}"))
        
        # Increment angle and radius
        angle += 0.5  # radians
        radius += spacing / (2 * math.pi)
        point_count += 1
    
    return waypoints


class MissionProfileController(Node):
    """Execute mission profiles using existing navigation infrastructure."""
    
    def __init__(self):
        super().__init__('mission_profile_controller')
        
        # Parameters
        self.declare_parameter('mission_type', 'EXPLORATION')
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('coverage_spacing', 1.0)
        self.declare_parameter('search_spacing', 2.0)
        self.declare_parameter('search_pattern', 'GRID')
        self.declare_parameter('return_to_start', True)
        
        # State
        self.current_profile: Optional[MissionProfile] = None
        self.current_state = MissionState.INITIALIZING
        self.pose: Optional[PoseStamped] = None
        self.home_pose: Optional[PoseStamped] = None
        self.waypoint_index = 0
        self.waypoints: List[Waypoint] = []
        
        # Subscribers
        self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.pose_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            '/mission/home_pose',
            self.home_callback,
            10
        )
        self.create_subscription(
            String,
            '/mission/state',
            self.mission_state_callback,
            10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PointStamped,
            '/mission/goal',
            10
        )
        self.profile_state_pub = self.create_publisher(
            String,
            '/mission/profile_state',
            10
        )
        self.waypoint_index_pub = self.create_publisher(
            Int32,
            '/mission/waypoint_index',
            10
        )
        self.total_waypoints_pub = self.create_publisher(
            Int32,
            '/mission/total_waypoints',
            10
        )
        
        # Timer for periodic processing
        self.create_timer(1.0, self.process)
        
        self.get_logger().info('Mission profile controller started')
        
        # Initialize mission based on parameters
        self.initialize_mission()
    
    def initialize_mission(self):
        """Initialize mission profile from parameters."""
        mission_type_str = self.get_parameter('mission_type').value
        
        try:
            mission_type = MissionType[mission_type_str.upper()]
        except KeyError:
            self.get_logger().error(f'Invalid mission type: {mission_type_str}, defaulting to EXPLORATION')
            mission_type = MissionType.EXPLORATION
        
        self.current_profile = MissionProfile(
            mission_type=mission_type,
            waypoints=[],
            coverage_spacing=self.get_parameter('coverage_spacing').value,
            search_spacing=self.get_parameter('search_spacing').value,
            search_pattern=self.get_parameter('search_pattern').value,
            return_to_start=self.get_parameter('return_to_start').value
        )
        
        self.get_logger().info(f'Mission type: {mission_type.value}')
    
    def pose_callback(self, msg: PoseStamped):
        """Track current robot pose."""
        self.pose = msg
    
    def home_callback(self, msg: PoseStamped):
        """Track home pose."""
        if self.home_pose is None:
            self.home_pose = msg
            self.get_logger().info(f'Home pose captured: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
            
            # Generate waypoints now that we have home pose
            self.generate_mission_waypoints()
    
    def mission_state_callback(self, msg: String):
        """Track mission state from mission manager."""
        pass  # Monitor for completion/failure
    
    def generate_mission_waypoints(self):
        """Generate waypoints based on mission type."""
        if self.current_profile is None or self.home_pose is None:
            return
        
        mission_type = self.current_profile.mission_type
        
        if mission_type == MissionType.EXPLORATION:
            # No fixed waypoints - frontier explorer handles this
            self.waypoints = []
            self.current_state = MissionState.ACTIVE
            self.get_logger().info('Exploration mission: using frontier exploration')
        
        elif mission_type == MissionType.WAYPOINT:
            # Use configured waypoints or generate demo waypoints
            if self.current_profile.waypoints:
                self.waypoints = self.current_profile.waypoints
            else:
                # Demo: 4 corners of a square
                home_x = self.home_pose.pose.position.x
                home_y = self.home_pose.pose.position.y
                side = 5.0
                self.waypoints = [
                    Waypoint(home_x + side, home_y, name="waypoint_1"),
                    Waypoint(home_x + side, home_y + side, name="waypoint_2"),
                    Waypoint(home_x, home_y + side, name="waypoint_3"),
                    Waypoint(home_x, home_y, name="waypoint_4_home"),
                ]
            self.current_state = MissionState.ACTIVE
            self.get_logger().info(f'Waypoint mission: {len(self.waypoints)} waypoints')
        
        elif mission_type == MissionType.COVERAGE:
            # Generate coverage pattern
            home_x = self.home_pose.pose.position.x
            home_y = self.home_pose.pose.position.y
            
            if self.current_profile.coverage_bounds:
                bounds = self.current_profile.coverage_bounds
            else:
                # Default: 10m × 10m area centered on home
                bounds = (home_x - 5, home_x + 5, home_y - 5, home_y + 5)
            
            self.waypoints = generate_coverage_waypoints(
                bounds,
                self.current_profile.coverage_spacing,
                home_x,
                home_y
            )
            self.current_state = MissionState.ACTIVE
            self.get_logger().info(f'Coverage mission: {len(self.waypoints)} waypoints')
        
        elif mission_type == MissionType.SEARCH:
            # Generate search pattern
            home_x = self.home_pose.pose.position.x
            home_y = self.home_pose.pose.position.y
            
            if self.current_profile.search_bounds:
                bounds = self.current_profile.search_bounds
            else:
                # Default: 8m × 8m search area
                bounds = (home_x - 4, home_x + 4, home_y - 4, home_y + 4)
            
            pattern = self.current_profile.search_pattern.upper()
            
            if pattern == "GRID":
                self.waypoints = generate_grid_search_waypoints(
                    bounds,
                    self.current_profile.search_spacing
                )
            elif pattern == "SPIRAL":
                self.waypoints = generate_spiral_search_waypoints(
                    home_x, home_y,
                    max_radius=8.0,
                    spacing=self.current_profile.search_spacing
                )
            else:
                # Default to grid
                self.waypoints = generate_grid_search_waypoints(
                    bounds,
                    self.current_profile.search_spacing
                )
            
            self.current_state = MissionState.ACTIVE
            self.get_logger().info(f'Search mission ({pattern}): {len(self.waypoints)} waypoints')
        
        elif mission_type == MissionType.RETURN_HOME:
            # Single waypoint: home
            if self.home_pose:
                self.waypoints = [
                    Waypoint(
                        self.home_pose.pose.position.x,
                        self.home_pose.pose.position.y,
                        name="home"
                    )
                ]
            self.current_state = MissionState.ACTIVE
            self.get_logger().info('Return home mission')
        
        # Publish total waypoint count
        total_msg = Int32()
        total_msg.data = len(self.waypoints)
        self.total_waypoints_pub.publish(total_msg)
    
    def process(self):
        """Main processing loop."""
        if self.current_state != MissionState.ACTIVE:
            return
        
        if self.current_profile is None or self.pose is None:
            return
        
        # For exploration mode, delegate to frontier explorer
        if self.current_profile.mission_type == MissionType.EXPLORATION:
            state_msg = String()
            state_msg.data = "EXPLORING_FRONTIERS"
            self.profile_state_pub.publish(state_msg)
            return
        
        # For waypoint-based missions
        if self.waypoint_index >= len(self.waypoints):
            # Mission complete
            self.current_state = MissionState.COMPLETED
            state_msg = String()
            state_msg.data = "COMPLETED"
            self.profile_state_pub.publish(state_msg)
            self.get_logger().info('Mission profile completed')
            return
        
        # Check if current waypoint is reached
        current_wp = self.waypoints[self.waypoint_index]
        distance = math.hypot(
            self.pose.pose.position.x - current_wp.x,
            self.pose.pose.position.y - current_wp.y
        )
        
        tolerance = self.get_parameter('waypoint_tolerance').value
        
        if distance <= tolerance:
            # Waypoint reached, move to next
            self.get_logger().info(f'Waypoint {self.waypoint_index} reached: {current_wp.name}')
            self.waypoint_index += 1
            
            # Publish updated index
            index_msg = Int32()
            index_msg.data = self.waypoint_index
            self.waypoint_index_pub.publish(index_msg)
            
            if self.waypoint_index < len(self.waypoints):
                # Publish next goal
                self.publish_current_goal()
        else:
            # Still navigating to current waypoint
            # Ensure goal is published
            self.publish_current_goal()
        
        # Publish state
        state_msg = String()
        state_msg.data = f"WAYPOINT_{self.waypoint_index}_OF_{len(self.waypoints)}"
        self.profile_state_pub.publish(state_msg)
    
    def publish_current_goal(self):
        """Publish current waypoint as goal."""
        if self.waypoint_index >= len(self.waypoints):
            return
        
        waypoint = self.waypoints[self.waypoint_index]
        goal_msg = PointStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.point.x = waypoint.x
        goal_msg.point.y = waypoint.y
        goal_msg.point.z = waypoint.z
        
        self.goal_pub.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionProfileController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
