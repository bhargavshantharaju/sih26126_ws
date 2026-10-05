"""Unit tests for the DRISTHI obstacle extractor.

Tests cover:
  - PointCloud2 XYZ extraction from raw message bytes
  - Ground removal via per-sector minimum-Z estimation
  - Voxel grid clustering and minimum cluster size
  - Detection message field validation
"""

import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from sih26126_sensor_contract.obstacle_extractor import (
    cluster_to_detection,
    extract_xyz,
    remove_ground,
    voxel_cluster,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_cloud(xyz_list):
    """Build a flat PointCloud2 from a list of (x, y, z) tuples."""
    xyz = np.array(xyz_list, dtype=np.float32) if xyz_list else np.empty(
        (0, 3), dtype=np.float32)
    n = len(xyz)
    msg = PointCloud2()
    msg.height = 1
    msg.width = n
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = n * 12
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = xyz.tobytes()
    return msg


# ---------------------------------------------------------------------------
# extract_xyz
# ---------------------------------------------------------------------------

class TestExtractXyz:
    """Tests for raw XYZ extraction from PointCloud2 messages."""

    def test_empty_cloud(self):
        """An empty cloud returns a (0, 3) array."""
        msg = make_cloud([])
        result = extract_xyz(msg)
        assert result.shape == (0, 3)

    def test_basic_extraction(self):
        """Two valid points are extracted correctly."""
        pts = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        msg = make_cloud(pts)
        result = extract_xyz(msg)
        assert result.shape == (2, 3)
        np.testing.assert_allclose(result[0], [1.0, 2.0, 3.0], atol=1e-5)
        np.testing.assert_allclose(result[1], [4.0, 5.0, 6.0], atol=1e-5)

    def test_nan_filtering(self):
        """NaN-containing points are removed."""
        pts = [(1.0, 2.0, 3.0), (float('nan'), 0.0, 0.0), (4.0, 5.0, 6.0)]
        msg = make_cloud(pts)
        result = extract_xyz(msg)
        assert result.shape == (2, 3)

    def test_inf_filtering(self):
        """Infinite-coordinate points are removed."""
        pts = [(1.0, 2.0, 3.0), (float('inf'), 0.0, 0.0)]
        msg = make_cloud(pts)
        result = extract_xyz(msg)
        assert result.shape == (1, 3)

    def test_missing_fields_returns_empty(self):
        """Cloud without x/y/z fields returns empty array."""
        msg = PointCloud2()
        msg.height = 1
        msg.width = 5
        msg.point_step = 4
        msg.row_step = 20
        msg.fields = [PointField(name='intensity', offset=0,
                                 datatype=PointField.FLOAT32, count=1)]
        msg.data = b'\x00' * 20
        result = extract_xyz(msg)
        assert result.shape == (0, 3)

    def test_all_finite_points_preserved(self):
        """All finite points are kept."""
        pts = [(float(i), float(i), float(i)) for i in range(20)]
        msg = make_cloud(pts)
        result = extract_xyz(msg)
        assert result.shape == (20, 3)


# ---------------------------------------------------------------------------
# remove_ground
# ---------------------------------------------------------------------------

class TestRemoveGround:
    """Tests for sector-based ground removal."""

    def test_empty_input(self):
        """Empty input returns empty output."""
        pts = np.empty((0, 3), dtype=np.float32)
        result = remove_ground(pts)
        assert len(result) == 0

    def test_ground_points_removed(self):
        """Points at ground level are mostly removed."""
        n = 100
        pts = np.zeros((n, 3), dtype=np.float32)
        rng = np.random.default_rng(42)
        pts[:, 0] = rng.uniform(0.5, 10.0, n)
        pts[:, 1] = rng.uniform(-5.0, 5.0, n)
        pts[:, 2] = 0.0
        result = remove_ground(pts, ground_threshold=0.25)
        assert len(result) < n * 0.2

    def test_elevated_points_kept(self):
        """Elevated obstacle points (z=1.0) survive ground removal."""
        rng = np.random.default_rng(0)
        n_ground = 50
        n_obstacle = 30
        ground = np.column_stack([
            rng.uniform(1, 5, n_ground),
            rng.uniform(-1, 1, n_ground),
            np.zeros(n_ground),
        ]).astype(np.float32)
        obstacle = np.column_stack([
            rng.uniform(2, 4, n_obstacle),
            rng.uniform(-0.5, 0.5, n_obstacle),
            np.ones(n_obstacle),
        ]).astype(np.float32)
        pts = np.vstack([ground, obstacle])
        result = remove_ground(pts, ground_threshold=0.25)
        assert len(result) >= n_obstacle * 0.5


# ---------------------------------------------------------------------------
# voxel_cluster
# ---------------------------------------------------------------------------

class TestVoxelCluster:
    """Tests for voxel-grid connected-component clustering."""

    def test_empty_input(self):
        """Empty input returns no clusters."""
        pts = np.empty((0, 3), dtype=np.float32)
        clusters = voxel_cluster(pts)
        assert clusters == []

    def test_single_cluster(self):
        """A dense cube of points forms exactly one cluster."""
        x, y, z = np.meshgrid(
            np.linspace(0, 0.5, 5),
            np.linspace(0, 0.5, 5),
            np.linspace(0, 0.5, 5),
        )
        pts = np.column_stack(
            [x.flatten(), y.flatten(), z.flatten()]
        ).astype(np.float32)
        clusters = voxel_cluster(pts, voxel_size=0.25, min_pts=5)
        assert len(clusters) == 1
        assert len(clusters[0]) >= 5

    def test_two_separated_clusters(self):
        """Two point groups 5 m apart should form two clusters."""
        rng = np.random.default_rng(1)
        n = 30
        cluster_a = np.column_stack([
            rng.uniform(0, 0.4, n),
            rng.uniform(0, 0.4, n),
            rng.uniform(0, 0.4, n),
        ]).astype(np.float32)
        cluster_b = cluster_a + np.array([5.0, 5.0, 0.0], dtype=np.float32)
        pts = np.vstack([cluster_a, cluster_b])
        clusters = voxel_cluster(pts, voxel_size=0.25, min_pts=5)
        assert len(clusters) == 2

    def test_minimum_points_filter(self):
        """Clusters with fewer than min_pts are discarded."""
        pts = np.array([
            [0.0, 0.0, 0.5],
            [0.1, 0.0, 0.5],
            [0.0, 0.1, 0.5],
        ], dtype=np.float32)
        clusters = voxel_cluster(pts, voxel_size=0.25, min_pts=5)
        assert clusters == []

    def test_too_few_total_points(self):
        """Fewer than min_pts total returns no clusters."""
        pts = np.array(
            [[float(i), 0.0, 0.5] for i in range(3)], dtype=np.float32)
        clusters = voxel_cluster(pts, min_pts=5)
        assert clusters == []


# ---------------------------------------------------------------------------
# cluster_to_detection
# ---------------------------------------------------------------------------

class TestClusterToDetection:
    """Tests for converting a point cluster to TrackedObstacle message."""

    def test_basic_detection(self):
        """A cluster produces a valid TrackedObstacle with expected fields."""
        pts = np.array([
            [2.0, 0.0, 0.5],
            [2.1, 0.1, 0.6],
            [2.2, 0.0, 0.5],
            [2.0, 0.1, 0.5],
            [2.1, 0.05, 0.55],
        ], dtype=np.float32)
        det = cluster_to_detection(pts, 'base_link')
        assert det.id.startswith('raw_')
        assert math.isfinite(det.position.x)
        assert math.isfinite(det.position.y)
        assert math.isfinite(det.position.z)
        assert 0.0 < det.confidence <= 1.0
        assert det.dimensions.x > 0
        assert det.dimensions.y > 0
        assert det.dimensions.z > 0
        assert det.is_dynamic is False
        assert det.classification == 'UNKNOWN'
        assert det.frame_id == 'base_link'

    def test_large_cluster_high_confidence(self):
        """A larger cluster should have higher confidence than a small one."""
        rng = np.random.default_rng(2)
        small = rng.random((5, 3)).astype(np.float32)
        large = rng.random((100, 3)).astype(np.float32)
        det_small = cluster_to_detection(small, 'frame')
        det_large = cluster_to_detection(large, 'frame')
        assert det_large.confidence >= det_small.confidence

    def test_centroid_is_mean(self):
        """Detection position is the centroid of the input points."""
        pts = np.array([
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [0.0, 2.0, 0.0],
            [2.0, 2.0, 0.0],
        ], dtype=np.float32)
        det = cluster_to_detection(pts, 'base_link')
        assert abs(det.position.x - 1.0) < 1e-4
        assert abs(det.position.y - 1.0) < 1e-4

    def test_dimensions_positive(self):
        """All bounding box dimensions must be positive."""
        rng = np.random.default_rng(3)
        pts = (rng.random((20, 3)) * 2).astype(np.float32)
        det = cluster_to_detection(pts, 'base_link')
        assert det.dimensions.x > 0
        assert det.dimensions.y > 0
        assert det.dimensions.z > 0
