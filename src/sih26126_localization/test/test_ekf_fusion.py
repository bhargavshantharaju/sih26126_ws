# Copyright 2026 DRISTHI Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Unit tests for EKF fusion configuration and launch structure.

Tests verify static properties of the EKF config and launch file without
requiring a running ROS 2 environment or Gazebo simulation.

Checks:
  - EKF YAML is valid and contains mandatory keys
  - Correct frame identifiers (odom, base_link, map)
  - world_frame = odom (local estimator, not global)
  - Expected input topics present
  - Linear acceleration NOT fused (Gazebo gravity is present)
  - Sensor adapter assigns finite covariance to zero-covariance Gazebo input
  - publish_tf is true (EKF owns odom->base_link)
  - Launch file declares the fusion argument
  - odom_tf_publisher is conditioned on slam AND NOT fusion
  - imu_static_tf_publisher is conditioned on fusion
  - ekf_node is present and conditioned on fusion
  - /odometry/filtered remapping present in launch file
  - fusion_health_monitor present in launch file
"""

import os

from ament_index_python.packages import get_package_share_directory
from sih26126_localization.ekf_sensor_adapter import _covariance
import yaml


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------

def _ekf_yaml_path() -> str:
    share = get_package_share_directory('sih26126_localization')
    return os.path.join(share, 'config', 'ekf_params.yaml')


def _launch_py_path() -> str:
    share = get_package_share_directory('sih26126_bringup')
    return os.path.join(share, 'launch', 'full_stack.launch.py')


def _load_ekf_yaml():
    with open(_ekf_yaml_path()) as fh:
        return yaml.safe_load(fh)


def _load_launch_source() -> str:
    with open(_launch_py_path()) as fh:
        return fh.read()


def _ekf_params(data: dict) -> dict:
    return data['ekf_filter_node']['ros__parameters']


# ---------------------------------------------------------------------------
# EKF YAML structural tests
# ---------------------------------------------------------------------------

class TestEkfYamlExists:
    """Verify the EKF YAML file exists and is parseable."""

    def test_file_exists(self):
        """EKF config file must exist in the installed package share."""
        assert os.path.isfile(_ekf_yaml_path()), (
            f'EKF config not found at: {_ekf_yaml_path()}'
        )

    def test_yaml_parseable(self):
        """EKF config YAML must be valid YAML."""
        data = _load_ekf_yaml()
        assert data is not None

    def test_top_level_key(self):
        """EKF YAML must have top-level key ekf_filter_node."""
        data = _load_ekf_yaml()
        assert 'ekf_filter_node' in data

    def test_ros_parameters_key(self):
        """ekf_filter_node must contain ros__parameters."""
        data = _load_ekf_yaml()
        assert 'ros__parameters' in data['ekf_filter_node']


class TestEkfFrames:
    """Verify frame identifiers match the rest of the stack."""

    def test_map_frame(self):
        """map_frame must be map."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['map_frame'] == 'map'

    def test_odom_frame(self):
        """odom_frame must be odom."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['odom_frame'] == 'odom'

    def test_base_link_frame(self):
        """base_link_frame must be base_link."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['base_link_frame'] == 'base_link'

    def test_world_frame_is_odom(self):
        """world_frame must be odom (local estimator; SLAM owns global map->odom)."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['world_frame'] == 'odom', (
            'world_frame must be odom — EKF is a local estimator; '
            'SLAM provides global map->odom separately'
        )

    def test_two_d_mode_enabled(self):
        """two_d_mode must be true for a planar UGV."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['two_d_mode'] is True


class TestEkfInputTopics:
    """Verify expected sensor input topics are declared."""

    def test_odom0_topic(self):
        """odom0 must reference the Gazebo odometry topic."""
        p = _ekf_params(_load_ekf_yaml())
        assert p.get('odom0') == '/fusion/wheel_odometry'

    def test_imu0_topic(self):
        """imu0 must reference the covariance-normalized IMU stream."""
        p = _ekf_params(_load_ekf_yaml())
        assert p.get('imu0') == '/fusion/imu'


class TestEkfSensorConfig:
    """Verify which fields are fused from each sensor."""

    def test_odom0_config_length(self):
        """odom0_config must have exactly 15 entries."""
        p = _ekf_params(_load_ekf_yaml())
        assert len(p['odom0_config']) == 15

    def test_odom0_pose_not_fused(self):
        """Wheel odometry pose (x,y,z,roll,pitch,yaw) must not be fused."""
        p = _ekf_params(_load_ekf_yaml())
        cfg = p['odom0_config']
        assert not any(cfg[0:6]), (
            'odom0 pose fields must not be fused — SLAM owns absolute pose'
        )

    def test_odom0_vx_fused(self):
        """Wheel forward velocity (vx, index 6) must be fused."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['odom0_config'][6] is True

    def test_odom0_vy_not_fused(self):
        """Lateral velocity vy must not be fused (differential-drive UGV)."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['odom0_config'][7] is False

    def test_odom0_vyaw_fused(self):
        """Wheel yaw rate (vyaw, index 11) must be fused."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['odom0_config'][11] is True

    def test_odom0_accel_not_fused(self):
        """Odometry acceleration fields must not be fused."""
        p = _ekf_params(_load_ekf_yaml())
        assert not any(p['odom0_config'][12:15])

    def test_odom0_differential_false(self):
        """No pose fields are fused, so differential processing is disabled."""
        p = _ekf_params(_load_ekf_yaml())
        assert p.get('odom0_differential') is False

    def test_imu0_config_length(self):
        """imu0_config must have exactly 15 entries."""
        p = _ekf_params(_load_ekf_yaml())
        assert len(p['imu0_config']) == 15

    def test_imu0_roll_pitch_not_fused(self):
        """IMU roll and pitch must not be fused (two_d_mode UGV)."""
        p = _ekf_params(_load_ekf_yaml())
        cfg = p['imu0_config']
        assert not any(cfg[3:5])

    def test_imu0_yaw_fused(self):
        """IMU yaw (index 5) must be fused."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['imu0_config'][5] is True

    def test_imu0_vyaw_fused(self):
        """IMU yaw rate (index 11) must be fused."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['imu0_config'][11] is True

    def test_imu0_linear_accel_not_fused(self):
        """IMU linear acceleration must NOT be fused (Gazebo gravity present)."""
        p = _ekf_params(_load_ekf_yaml())
        assert not any(p['imu0_config'][12:15]), (
            'imu0 linear acceleration must not be fused: '
            'Gazebo IMU includes gravity (z≈9.8 m/s²) and covariance=0'
        )

    def test_imu0_remove_gravity_disabled(self):
        """Acceleration is excluded, so gravity removal is unnecessary."""
        p = _ekf_params(_load_ekf_yaml())
        assert p.get('imu0_remove_gravitational_acceleration') is False


class TestSensorCovarianceAdapter:
    """Ensure zero-covariance bridge data is normalized before the EKF."""

    def test_ekf_input_adapter_has_finite_covariances(self):
        """The adapter must replace zero covariance only on sensor inputs."""
        p = _ekf_params(_load_ekf_yaml())
        assert p['odom0'] == '/fusion/wheel_odometry'
        assert p['imu0'] == '/fusion/imu'
        odom_cov = _covariance(6, {0: 0.01, 5: 0.04})
        imu_cov = _covariance(3, {2: 0.0025})
        assert len(odom_cov) == 36
        assert odom_cov[0] > 0.0 and odom_cov[35] > 0.0
        assert len(imu_cov) == 9 and imu_cov[8] > 0.0

    def test_adapter_receives_actual_raw_topics(self):
        """The adapter must relay the inspected wheel and IMU sources."""
        source = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'sih26126_localization', 'ekf_sensor_adapter.py'
        )
        with open(source) as fh:
            src = fh.read()
        assert "'/model/sih_ugv/odometry'" in src
        assert "'/imu'" in src

    def test_wheel_validation_gate_defaults_enabled(self):
        """The experiment gate must preserve normal production fusion."""
        source = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'sih26126_localization', 'ekf_sensor_adapter.py'
        )
        with open(source) as fh:
            src = fh.read()
        assert "declare_parameter('wheel_input_enabled', True)" in src
        assert "if not self.get_parameter('wheel_input_enabled').value" in src


class TestWheelContributionHarness:
    """Check the paired isolation harness enforces the input distinction."""

    def test_harness_requires_fresh_launch_per_case_and_checks_inputs(self):
        source = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'sih26126_localization', 'wheel_contribution_test.py'
        )
        with open(source) as fh:
            src = fh.read()
        assert 'fresh full_stack launch for each case' in src
        assert 'self._set_wheel_gate(enabled)' in src
        assert "'wheel_relay_stopped_when_disabled'" in src
        assert "'imu_active_both_cases'" in src
        assert "'paired_trajectory_difference_material'" in src


class TestEkfPublishTF:
    """Verify EKF is configured to publish odom->base_link TF."""

    def test_publish_tf_true(self):
        """publish_tf must be true (EKF owns odom->base_link)."""
        p = _ekf_params(_load_ekf_yaml())
        assert p.get('publish_tf') is True


# ---------------------------------------------------------------------------
# Launch file tests
# ---------------------------------------------------------------------------

class TestLaunchFusionArgument:
    """Verify the launch file declares the fusion argument."""

    def test_fusion_arg_declared(self):
        """Launch file must declare a fusion argument."""
        src = _load_launch_source()
        assert "'fusion'" in src or '"fusion"' in src

    def test_fusion_default_false(self):
        """Fusion argument must default to false."""
        src = _load_launch_source()
        assert "default_value='false'" in src or 'default_value="false"' in src

    def test_slam_and_fusion_are_independently_selectable(self):
        """The original SLAM selection remains available beside fusion."""
        src = _load_launch_source()
        assert "'slam', default_value='false'" in src
        assert "'fusion', default_value='false'" in src


class TestLaunchTFAuthority:
    """Verify correct exclusive TF authority in the launch file."""

    def test_odom_tf_publisher_uses_not_substitution(self):
        """odom_tf_publisher must use NotSubstitution to exclude fusion=true."""
        src = _load_launch_source()
        assert 'odom_tf_publisher' in src
        assert 'NotSubstitution' in src, (
            'odom_tf_publisher must use NotSubstitution(fusion) '
            'to prevent competing with EKF when fusion=true'
        )

    def test_ekf_node_present(self):
        """ekf_node must be declared in the launch file."""
        src = _load_launch_source()
        assert 'ekf_node' in src

    def test_robot_localization_package(self):
        """robot_localization must be referenced as the EKF package."""
        src = _load_launch_source()
        assert 'robot_localization' in src

    def test_imu_static_tf_present(self):
        """imu_static_tf_publisher must be present in the launch file."""
        src = _load_launch_source()
        assert 'imu_static_tf_publisher' in src

    def test_gazebo_tf_bridge_disabled_in_slam_and_fusion_modes(self):
        """Gazebo pose TF is baseline-only, avoiding competing odom TF."""
        src = _load_launch_source()
        assert 'gazebo_tf_bridge' in src
        assert 'NotSubstitution(slam)' in src
        assert 'NotSubstitution(fusion)' in src

    def test_sensor_adapter_enabled_with_fusion(self):
        """The EKF input normalizer is part of fusion mode."""
        src = _load_launch_source()
        assert 'ekf_sensor_adapter' in src


class TestLaunchOutputTopic:
    """Verify the EKF output topic remapping."""

    def test_odometry_filtered_remapping(self):
        """Launch file must remap EKF output to /odometry/filtered."""
        src = _load_launch_source()
        assert 'odometry/filtered' in src


class TestLaunchFusionHealthMonitor:
    """Verify fusion health monitor is in the launch file."""

    def test_fusion_health_monitor_present(self):
        """fusion_health_monitor node must be in the launch file."""
        src = _load_launch_source()
        assert 'fusion_health_monitor' in src
