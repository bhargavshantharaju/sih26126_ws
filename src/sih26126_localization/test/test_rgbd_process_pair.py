import cv2
import numpy as np

from sih26126_localization.rgbd_odometry_baseline import (
    RGBDOdometryBaseline,
)


class FakeOrb:
    def __init__(self, keypoint_sets, descriptors):
        self.keypoint_sets = keypoint_sets
        self.descriptors = descriptors
        self.calls = 0

    def detectAndCompute(self, gray, mask):
        assert gray.shape == (240, 320)
        keypoints = self.keypoint_sets[self.calls]
        self.calls += 1
        return keypoints, self.descriptors.copy()


class FakeMatcher:
    def __init__(self, count):
        self.matches = [
            [
                cv2.DMatch(
                    _queryIdx=i,
                    _trainIdx=i,
                    _distance=10.0,
                ),
                cv2.DMatch(
                    _queryIdx=i,
                    _trainIdx=i,
                    _distance=100.0,
                ),
            ]
            for i in range(count)
        ]

    def knnMatch(self, previous_descriptors, descriptors, k=2):
        assert k == 2
        return self.matches


def make_keypoints(offset_u):
    points = [
        (40, 40),
        (80, 45),
        (120, 50),
        (160, 55),
        (200, 60),
        (240, 70),
        (60, 100),
        (100, 110),
        (140, 120),
        (180, 130),
        (220, 140),
        (260, 150),
    ]

    return [
        cv2.KeyPoint(
            float(u + offset_u),
            float(v),
            8.0,
        )
        for u, v in points
    ]


def make_cloud(points_3d, offset_u):
    cloud = np.full(
        (240, 320, 3),
        np.nan,
        dtype=np.float32,
    )

    pixels = [
        (40, 40),
        (80, 45),
        (120, 50),
        (160, 55),
        (200, 60),
        (240, 70),
        (60, 100),
        (100, 110),
        (140, 120),
        (180, 130),
        (220, 140),
        (260, 150),
    ]

    for point, (u, v) in zip(points_3d, pixels):
        cloud[v, u + offset_u] = point

    return cloud


def configure_node(
    keypoint_sets,
    initial_cloud,
    initial_keypoints,
    descriptors,
):
    node = RGBDOdometryBaseline.__new__(
        RGBDOdometryBaseline
    )

    node.orb = FakeOrb(
        keypoint_sets,
        descriptors,
    )
    node.matcher = FakeMatcher(
        len(initial_keypoints)
    )

    node.previous_gray = np.zeros(
        (240, 320),
        dtype=np.uint8,
    )
    node.previous_cloud = initial_cloud.copy()
    node.previous_keypoints = initial_keypoints
    node.previous_descriptors = descriptors.copy()
    node.previous_stamp = 1.0

    node.pose_rotation = np.eye(
        3,
        dtype=np.float32,
    )
    node.pose_translation = np.zeros(
        3,
        dtype=np.float32,
    )

    node.pair_count = 0
    node.accepted_count = 0
    node.rejected_count = 0
    node.last_log_pair = 0

    node.publish_pose = lambda stamp: None

    return node


def test_process_pair_recovers_and_accumulates_metric_forward_motion():
    rng = np.random.default_rng(26126)

    source_points = np.column_stack(
        [
            rng.uniform(2.0, 5.0, 12),
            rng.uniform(-0.8, 0.8, 12),
            rng.uniform(-0.4, 0.4, 12),
        ]
    ).astype(np.float32)

    motion = np.array(
        [0.20, 0.0, 0.0],
        dtype=np.float32,
    )

    state0 = source_points
    state1 = source_points - motion
    state2 = source_points - 2.0 * motion

    keypoints0 = make_keypoints(0)
    keypoints1 = make_keypoints(2)
    keypoints2 = make_keypoints(4)

    descriptors = np.zeros(
        (12, 32),
        dtype=np.uint8,
    )

    node = configure_node(
        [
            keypoints1,
            keypoints2,
        ],
        make_cloud(state0, 0),
        keypoints0,
        descriptors,
    )

    rgb_frame = np.zeros(
        (480, 640),
        dtype=np.uint8,
    )

    node.process_pair(
        rgb_frame,
        make_cloud(state1, 2),
        1.1,
        0.0,
    )

    assert node.accepted_count == 1
    assert node.rejected_count == 0
    assert np.allclose(
        node.pose_translation,
        np.array([0.20, 0.0, 0.0]),
        atol=1e-4,
    )

    node.process_pair(
        rgb_frame,
        make_cloud(state2, 4),
        1.2,
        0.0,
    )

    assert node.accepted_count == 2
    assert node.rejected_count == 0
    assert np.allclose(
        node.pose_translation,
        np.array([0.40, 0.0, 0.0]),
        atol=1e-4,
    )
