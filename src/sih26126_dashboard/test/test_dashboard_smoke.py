"""Smoke tests for the dashboard package."""

from sih26126_dashboard.dashboard_node import HOST, PORT, DashboardNode


def test_dashboard_module_exports_runtime_configuration():
    """The dashboard node should expose its runtime host and port."""
    assert isinstance(HOST, str)
    assert isinstance(PORT, int)
    assert issubclass(DashboardNode.__class__, object)
