"""Unit tests for the SIH26126 sensor contract."""

import numpy as np

from sensor_msgs.msg import PointCloud2, PointField

from sih26126_sensor_contract.sensor_contract import (
    compute_valid_point_ratio,
)


def make_cloud(
    width,
    height,
    xyz,
):
    """Build an organized FLOAT32 XYZ cloud."""
    values = np.asarray(
        xyz,
        dtype=np.float32,
    ).reshape(height, width, 3)

    msg = PointCloud2()
    msg.height = height
    msg.width = width
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = width * msg.point_step
    msg.fields = [
        PointField(
            name='x',
            offset=0,
            datatype=PointField.FLOAT32,
            count=1,
        ),
        PointField(
            name='y',
            offset=4,
            datatype=PointField.FLOAT32,
            count=1,
        ),
        PointField(
            name='z',
            offset=8,
            datatype=PointField.FLOAT32,
            count=1,
        ),
    ]
    msg.data = values.tobytes()
    return msg


def test_valid_ratio_ignores_upper_invalid_region():
    """Only the navigation ROI should affect the ratio."""
    points = np.ones(
        (8, 4, 3),
        dtype=np.float32,
    )
    points[:4, :, :] = np.inf
    points[:, :, 0] = 2.0
    points[:, :, 1] = 0.0
    points[:, :, 2] = 0.0

    msg = make_cloud(
        4,
        8,
        points,
    )

    ratio = compute_valid_point_ratio(
        msg,
        roi_y_start=0.50,
    )

    assert ratio == 1.0


def test_valid_ratio_is_zero_for_invalid_cloud():
    """An all-invalid XYZ cloud must report zero."""
    points = np.full(
        (8, 4, 3),
        np.inf,
        dtype=np.float32,
    )

    msg = make_cloud(
        4,
        8,
        points,
    )

    ratio = compute_valid_point_ratio(
        msg,
        roi_y_start=0.50,
    )

    assert ratio == 0.0


def test_valid_ratio_rejects_out_of_range_points():
    """Points outside the configured navigation range are invalid."""
    points = np.zeros(
        (8, 4, 3),
        dtype=np.float32,
    )
    points[:, :, 0] = 20.0

    msg = make_cloud(
        4,
        8,
        points,
    )

    ratio = compute_valid_point_ratio(
        msg,
        roi_y_start=0.50,
        max_range_m=15.0,
    )

    assert ratio == 0.0
