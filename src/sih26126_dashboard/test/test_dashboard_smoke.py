"""Smoke tests for the dashboard package."""

from sih26126_dashboard.dashboard_node import (
    HTML, HOST, PORT, DashboardNode, status_payload, subsystem_health,
)


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
