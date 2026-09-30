"""Smoke tests for the dashboard package."""

import struct

from sih26126_dashboard.dashboard_node import (
    HTML, HOST, PORT, DashboardNode, cloud_stats, status_payload,
    subsystem_health,
)
from sensor_msgs.msg import PointCloud2, PointField


def test_dashboard_module_exports_runtime_configuration():
    """The dashboard node should expose its runtime host and port."""
    assert isinstance(HOST, str)
    assert isinstance(PORT, int)
    assert issubclass(DashboardNode.__class__, object)


def test_health_marks_partial_sensor_loss_degraded():
    health, system = subsystem_health({
        'camera': 0.1, 'cloud': None, 'localization': 0.1,
        'mapping': 0.1, 'traversability': 0.1, 'exploration': 0.1,
        'global_planner': 0.1, 'local_planner': 0.1, 'safety': 0.1,
    })
    assert health['SENSOR'] == 'DEGRADED'
    assert system == 'DEGRADED'


def test_health_marks_old_messages_stale_and_no_live_stack_offline():
    health, system = subsystem_health({
        'camera': 4.0, 'cloud': 4.0, 'localization': 4.0,
        'mapping': None, 'traversability': None, 'exploration': None,
        'global_planner': None, 'local_planner': None, 'safety': None,
    })
    assert health['SENSOR'] == 'STALE'
    assert health['LOCALIZATION'] == 'STALE'
    assert system == 'OFFLINE'


def test_dashboard_exposes_live_global_recommended_speed():
    """The planner panel and status API expose the received speed value."""
    payload = status_payload()

    assert 'Global recommended speed' in HTML
    assert 'recommendedspeed' in HTML
    assert 'recommended_speed' in payload['planner']


def test_dashboard_renders_sampled_live_depth_returns_and_local_command():
    """Depth visualization and local advisory command use their ROS inputs."""
    msg = PointCloud2()
    msg.height = 1
    msg.width = 9
    msg.point_step = 12
    msg.row_step = 108
    msg.fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in (('x', 0), ('y', 4), ('z', 8))
    ]
    msg.data = b''.join(
        struct.pack('<fff', float(col + 1), float(col) / 10.0, float(col) / 10.0)
        for col in range(9)
    )
    stats = cloud_stats(msg)

    assert [point[0] for point in stats['points']] == [1.0, 9.0]
    assert 'id="pointcloud"' in HTML
    assert 'id="localcommand"' in HTML
    assert 'local_command' in status_payload()['planner']


def test_health_exposes_individual_rgb_depth_imu_and_pose_freshness():
    health, _ = subsystem_health({
        'camera': 0.1, 'cloud': 0.2, 'imu': 0.3, 'localization': 0.4,
        'mapping': 0.1, 'traversability': 0.1, 'exploration': 0.1,
        'global_planner': 0.1, 'local_planner': 0.1, 'safety': 0.1,
    })

    assert [health[name] for name in ('RGB', 'DEPTH', 'IMU', 'POSE')] == [
        'LIVE', 'LIVE', 'LIVE', 'LIVE']
