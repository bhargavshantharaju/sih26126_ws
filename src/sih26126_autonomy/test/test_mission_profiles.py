#!/usr/bin/env python3
"""Tests for mission profile system."""

import math
import unittest

from sih26126_autonomy.mission_profiles import (
    MissionType,
    Waypoint,
    generate_coverage_waypoints,
    generate_grid_search_waypoints,
    generate_spiral_search_waypoints,
)


class TestMissionTypes(unittest.TestCase):
    """Test mission type enum."""
    
    def test_all_mission_types_defined(self):
        """All expected mission types exist with correct string values."""
        assert MissionType.EXPLORATION.value == "EXPLORATION"
        assert MissionType.WAYPOINT.value == "WAYPOINT"
        assert MissionType.COVERAGE.value == "COVERAGE"
        assert MissionType.SEARCH.value == "SEARCH"
        assert MissionType.RETURN_HOME.value == "RETURN_HOME"
    
    def test_mission_type_values(self):
        """Mission type enum values are correct."""
        assert MissionType.EXPLORATION.value == "EXPLORATION"
        assert MissionType.WAYPOINT.value == "WAYPOINT"
        assert MissionType.COVERAGE.value == "COVERAGE"


class TestWaypointDataclass(unittest.TestCase):
    """Test Waypoint dataclass."""
    
    def test_waypoint_creation(self):
        """Waypoint can be created with required fields."""
        wp = Waypoint(x=1.0, y=2.0)
        assert wp.x == 1.0
        assert wp.y == 2.0
        assert wp.z == 0.0  # default
        assert wp.tolerance == 0.5  # default
    
    def test_waypoint_with_all_fields(self):
        """Waypoint supports all optional fields."""
        wp = Waypoint(x=1.0, y=2.0, z=3.0, tolerance=1.0, name="test")
        assert wp.x == 1.0
        assert wp.y == 2.0
        assert wp.z == 3.0
        assert wp.tolerance == 1.0
        assert wp.name == "test"


class TestCoverageWaypoints(unittest.TestCase):
    """Test coverage pattern generation."""
    
    def test_coverage_generates_lawnmower_pattern(self):
        """Coverage generates alternating lawnmower lines."""
        bounds = (0.0, 10.0, 0.0, 10.0)
        spacing = 5.0
        waypoints = generate_coverage_waypoints(bounds, spacing, start_x=0.0, start_y=0.0)
        
        # Should have 3 lines (y=0, y=5, y=10) × 2 waypoints per line = 6 waypoints
        assert len(waypoints) >= 4
        
        # First line: left to right
        assert waypoints[0].x == 0.0
        assert waypoints[0].y == 0.0
        assert waypoints[1].x == 10.0
        assert waypoints[1].y == 0.0
        
        # Second line: right to left (alternating)
        assert waypoints[2].x == 10.0
        assert waypoints[2].y == 5.0
        assert waypoints[3].x == 0.0
        assert waypoints[3].y == 5.0
    
    def test_coverage_respects_spacing(self):
        """Coverage spacing determines line separation."""
        bounds = (0.0, 10.0, 0.0, 10.0)
        
        waypoints_wide = generate_coverage_waypoints(bounds, spacing=10.0, start_x=0.0, start_y=0.0)
        waypoints_narrow = generate_coverage_waypoints(bounds, spacing=2.0, start_x=0.0, start_y=0.0)
        
        # Narrower spacing = more lines = more waypoints
        assert len(waypoints_narrow) > len(waypoints_wide)
    
    def test_coverage_names_waypoints(self):
        """Coverage waypoints have descriptive names."""
        bounds = (0.0, 10.0, 0.0, 10.0)
        spacing = 5.0
        waypoints = generate_coverage_waypoints(bounds, spacing, start_x=0.0, start_y=0.0)
        
        # Check naming pattern
        assert "coverage" in waypoints[0].name
        assert "start" in waypoints[0].name or "end" in waypoints[0].name
    
    def test_coverage_covers_entire_area(self):
        """Coverage pattern spans full bounds."""
        bounds = (5.0, 15.0, 10.0, 20.0)
        spacing = 2.0
        waypoints = generate_coverage_waypoints(bounds, spacing, start_x=5.0, start_y=10.0)
        
        # Should have waypoints at min and max x and y
        x_coords = [wp.x for wp in waypoints]
        y_coords = [wp.y for wp in waypoints]
        
        assert min(x_coords) == 5.0
        assert max(x_coords) == 15.0
        assert min(y_coords) >= 10.0
        assert max(y_coords) <= 20.0 + spacing


class TestGridSearchWaypoints(unittest.TestCase):
    """Test grid search pattern generation."""
    
    def test_grid_creates_regular_points(self):
        """Grid search creates evenly spaced points."""
        bounds = (0.0, 10.0, 0.0, 10.0)
        spacing = 5.0
        waypoints = generate_grid_search_waypoints(bounds, spacing)
        
        # With 5m spacing on 10m × 10m area: 3×3 = 9 points
        # (0,0), (5,0), (10,0), (0,5), (5,5), (10,5), (0,10), (5,10), (10,10)
        assert len(waypoints) == 9
        
        # Check corners
        assert any(wp.x == 0.0 and wp.y == 0.0 for wp in waypoints)
        assert any(wp.x == 10.0 and wp.y == 10.0 for wp in waypoints)
    
    def test_grid_respects_spacing(self):
        """Grid spacing determines point density."""
        bounds = (0.0, 10.0, 0.0, 10.0)
        
        waypoints_wide = generate_grid_search_waypoints(bounds, spacing=10.0)
        waypoints_narrow = generate_grid_search_waypoints(bounds, spacing=2.0)
        
        # Narrower spacing = more points
        assert len(waypoints_narrow) > len(waypoints_wide)
    
    def test_grid_names_waypoints_with_indices(self):
        """Grid waypoints named with row/column indices."""
        bounds = (0.0, 10.0, 0.0, 10.0)
        spacing = 5.0
        waypoints = generate_grid_search_waypoints(bounds, spacing)
        
        # Check naming pattern includes "search"
        assert all("search" in wp.name for wp in waypoints)
    
    def test_grid_empty_for_invalid_bounds(self):
        """Grid handles edge cases gracefully."""
        bounds = (0.0, 0.0, 0.0, 0.0)
        spacing = 5.0
        waypoints = generate_grid_search_waypoints(bounds, spacing)
        
        # Should have at least one point (the single corner)
        assert len(waypoints) >= 1


class TestSpiralSearchWaypoints(unittest.TestCase):
    """Test spiral search pattern generation."""
    
    def test_spiral_starts_at_center(self):
        """Spiral search starts near center point."""
        waypoints = generate_spiral_search_waypoints(
            center_x=5.0,
            center_y=5.0,
            max_radius=10.0,
            spacing=1.0
        )
        
        assert len(waypoints) > 0
        
        # First point should be near center
        first = waypoints[0]
        distance_from_center = math.hypot(first.x - 5.0, first.y - 5.0)
        assert distance_from_center < 1.0
    
    def test_spiral_expands_outward(self):
        """Spiral points increase in distance from center."""
        center_x, center_y = 0.0, 0.0
        waypoints = generate_spiral_search_waypoints(
            center_x=center_x,
            center_y=center_y,
            max_radius=10.0,
            spacing=1.0
        )
        
        # Calculate distances from center
        distances = [
            math.hypot(wp.x - center_x, wp.y - center_y)
            for wp in waypoints
        ]
        
        # Early points should be closer than later points (generally)
        assert distances[0] < distances[-1]
    
    def test_spiral_respects_max_radius(self):
        """Spiral stays within maximum radius."""
        center_x, center_y = 5.0, 5.0
        max_radius = 8.0
        
        waypoints = generate_spiral_search_waypoints(
            center_x=center_x,
            center_y=center_y,
            max_radius=max_radius,
            spacing=1.0
        )
        
        # All points should be within max_radius (plus small tolerance)
        for wp in waypoints:
            distance = math.hypot(wp.x - center_x, wp.y - center_y)
            assert distance <= max_radius + 0.5  # Small tolerance for discretization
    
    def test_spiral_names_waypoints_sequentially(self):
        """Spiral waypoints have sequential names."""
        waypoints = generate_spiral_search_waypoints(
            center_x=0.0,
            center_y=0.0,
            max_radius=5.0,
            spacing=1.0
        )
        
        # Check naming pattern
        assert all("spiral" in wp.name for wp in waypoints)
    
    def test_spiral_density_scales_with_spacing(self):
        """Spiral spacing affects point density."""
        waypoints_fine = generate_spiral_search_waypoints(
            center_x=0.0,
            center_y=0.0,
            max_radius=10.0,
            spacing=0.5
        )
        waypoints_coarse = generate_spiral_search_waypoints(
            center_x=0.0,
            center_y=0.0,
            max_radius=10.0,
            spacing=2.0
        )
        
        # Finer spacing = more points
        assert len(waypoints_fine) > len(waypoints_coarse)


class TestPatternGeometry(unittest.TestCase):
    """Test geometric properties of patterns."""
    
    def test_coverage_alternates_direction(self):
        """Coverage pattern alternates between left-right and right-left."""
        bounds = (0.0, 10.0, 0.0, 10.0)
        spacing = 3.0
        waypoints = generate_coverage_waypoints(bounds, spacing, start_x=0.0, start_y=0.0)
        
        # Group waypoints by pairs (line start, line end)
        lines = [(waypoints[i], waypoints[i+1]) for i in range(0, len(waypoints)-1, 2)]
        
        # Check alternation: line 0 goes left-to-right, line 1 goes right-to-left, etc.
        for i, (start, end) in enumerate(lines):
            if i % 2 == 0:
                # Even lines: left to right
                assert start.x < end.x
            else:
                # Odd lines: right to left
                assert start.x > end.x
    
    def test_grid_row_major_order(self):
        """Grid pattern processes in row-major order."""
        bounds = (0.0, 4.0, 0.0, 4.0)
        spacing = 2.0
        waypoints = generate_grid_search_waypoints(bounds, spacing)
        
        # Should be ordered: (0,0), (2,0), (4,0), (0,2), (2,2), (4,2), (0,4), (2,4), (4,4)
        # Check first row has same y
        y_first_row = waypoints[0].y
        assert waypoints[1].y == y_first_row
        assert waypoints[2].y == y_first_row


if __name__ == '__main__':
    unittest.main()
