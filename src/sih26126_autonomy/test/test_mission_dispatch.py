"""Pure-Python tests for MissionManager profile-based mission dispatch.

These tests do NOT call rclpy.init().  They mock the rclpy Node base class
and all ROS publisher/subscriber/timer infrastructure so the state-transition
logic inside MissionManager.process() can be exercised as a plain Python
function call.

Covered scenarios
-----------------
1. WAYPOINT mission: INITIALIZING → MAPPING → EXECUTING_PROFILE → RETURNING_HOME
2. COVERAGE mission: profile_state 'COMPLETED' triggers RETURNING_HOME
3. SEARCH mission:   profile_state 'FAILED' triggers FAILED state
4. EXPLORATION mission: existing frontier behavior unaffected (MAPPING → EXPLORING)
5. No goal conflict: mission_manager does not publish /mission/goal during
   EXECUTING_PROFILE (i.e. while the profile controller owns that topic).
"""

import importlib
import math
import sys
import time
import types
from unittest.mock import MagicMock, patch

import pytest

# ---------------------------------------------------------------------------
# Build a minimal fake rclpy + geometry_msgs + nav_msgs stubs so that the
# module can be imported without a real ROS environment.
# ---------------------------------------------------------------------------

def _make_stub_module(name):
    mod = types.ModuleType(name)
    sys.modules[name] = mod
    return mod


def _ensure_rclpy_stubs():
    """Create lightweight stubs for every rclpy / ROS sub-module used by
    mission_manager.py.

    Called unconditionally at module level.  Stubs are always installed (even
    when real rclpy has already been loaded by another test file that was
    collected first).  mission_manager is reloaded after the stubs are in
    place so it always references the fake Node class rather than the real one.

    This allows test_mission_dispatch.py and test_cost_aware_routing.py to
    coexist in the same pytest session: the routing tests load real rclpy, this
    file replaces it with a stub for its own purposes, and the reload makes the
    replacement effective for the MissionManager class used here.
    """
    # ---- rclpy ---------------------------------------------------------
    # Always replace rclpy with a stub, even if real rclpy is already present.
    rclpy_mod = _make_stub_module('rclpy')
    rclpy_mod.init = MagicMock()
    rclpy_mod.shutdown = MagicMock()
    rclpy_mod.ok = MagicMock(return_value=True)
    rclpy_mod.spin = MagicMock()

    node_mod = _make_stub_module('rclpy.node')

    class _FakeNode:
        """Mimics rclpy.node.Node just enough for MissionManager.__init__."""
        def __init__(self, name, **kwargs):
            self._name = name
            self._params = {}
            self._logger = MagicMock()
            self._logger.info = lambda msg: None
            self._logger.warn = lambda msg: None
            self._logger.error = lambda msg: None
            self._pubs = {}    # topic -> mock publisher
            self._clock = MagicMock()
            self._clock.now.return_value = MagicMock(to_msg=MagicMock(return_value=None))

        # ---- parameter API ----------------------------------------
        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            class _ParamProxy:
                def __init__(self_, v):
                    self_.value = v
            return _ParamProxy(self._params.get(name))

        def set_parameter_value(self, name, value):
            self._params[name] = value

        # ---- publisher / subscriber / timer stubs -----------------
        def create_publisher(self, msg_type, topic, qos):
            pub = MagicMock()
            pub._published = []
            def _record(msg, _pub=pub):
                _pub._published.append(msg)
            pub.publish.side_effect = _record
            self._pubs[topic] = pub
            return pub

        def create_subscription(self, msg_type, topic, cb, qos):
            return MagicMock()

        def create_timer(self, period, cb):
            return MagicMock()

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return self._clock

        def destroy_node(self):
            pass

    node_mod.Node = _FakeNode
    rclpy_mod.node = node_mod

    # Stub the rclpy sub-modules that mission_manager.py uses at import time.
    for sub in ('rclpy.qos', 'rclpy.exceptions'):
        stub = _make_stub_module(sub)
        # rclpy.qos — stub the symbols used in mission_manager imports
        if sub == 'rclpy.qos':
            stub.QoSProfile = MagicMock
            stub.DurabilityPolicy = MagicMock()
            stub.ReliabilityPolicy = MagicMock()
        elif sub == 'rclpy.exceptions':
            stub.NotInitializedException = Exception
        setattr(rclpy_mod, sub.split('.', 1)[1], stub)

    # ---- geometry_msgs -------------------------------------------------
    geom = _make_stub_module('geometry_msgs')
    geom_msg = _make_stub_module('geometry_msgs.msg')

    class _Pose:
        class _Point:
            def __init__(self): self.x = self.y = self.z = 0.0
        class _Quaternion:
            def __init__(self): self.x = self.y = self.z = 0.0; self.w = 1.0
        def __init__(self):
            self.position = self._Point()
            self.orientation = self._Quaternion()

    class _PoseStamped:
        class _Header:
            def __init__(self): self.frame_id = 'map'; self.stamp = None
        def __init__(self):
            self.header = self._Header()
            self.pose = _Pose()

    class _PointStamped:
        class _Point:
            def __init__(self): self.x = self.y = self.z = 0.0
        class _Header:
            def __init__(self): self.frame_id = 'map'; self.stamp = None
        def __init__(self):
            self.header = self._Header()
            self.point = self._Point()

    class _Twist:
        class _Vec:
            def __init__(self): self.x = self.y = self.z = 0.0
        def __init__(self):
            self.linear = self._Vec()
            self.angular = self._Vec()

    geom_msg.PoseStamped = _PoseStamped
    geom_msg.PointStamped = _PointStamped
    geom_msg.Twist = _Twist
    geom_msg.Point = _Pose._Point
    geom.msg = geom_msg

    # ---- std_msgs -------------------------------------------------------
    std = _make_stub_module('std_msgs')
    std_msg = _make_stub_module('std_msgs.msg')

    class _String:
        def __init__(self, data=''):
            self.data = data

    class _Float32:
        def __init__(self, data=0.0):
            self.data = data

    class _Int32:
        def __init__(self, data=0):
            self.data = data

    std_msg.String = _String
    std_msg.Float32 = _Float32
    std_msg.Int32 = _Int32
    std.msg = std_msg

    # ---- nav_msgs -------------------------------------------------------
    nav = _make_stub_module('nav_msgs')
    nav_msg = _make_stub_module('nav_msgs.msg')

    class _OccupancyGrid:
        class _Info:
            def __init__(self): self.resolution = 0.05
        def __init__(self):
            self.info = self._Info()
            self.data = []

    class _Odometry:
        class _TwistWithCov:
            class _Twist:
                class _Vec:
                    def __init__(self): self.x = self.y = self.z = 0.0
                def __init__(self):
                    self.linear = self._Vec()
                    self.angular = self._Vec()
            def __init__(self):
                self.twist = self._Twist()
        def __init__(self):
            self.twist = self._TwistWithCov()

    class _NavPath:
        def __init__(self):
            self.poses = []

    nav_msg.OccupancyGrid = _OccupancyGrid
    nav_msg.Odometry = _Odometry
    nav_msg.Path = _NavPath
    nav.msg = nav_msg

    # ---- ros_gz_interfaces -----------------------------------------------
    rgzi = _make_stub_module('ros_gz_interfaces')
    rgzi_msg = _make_stub_module('ros_gz_interfaces.msg')

    class _Contacts:
        def __init__(self): self.contacts = []

    rgzi_msg.Contacts = _Contacts
    rgzi.msg = rgzi_msg

    # ---- sih26126_msgs --------------------------------------------------
    # Stub the custom message types used by mission_manager at import time.
    sih_msgs = _make_stub_module('sih26126_msgs')
    sih_msgs_msg = _make_stub_module('sih26126_msgs.msg')
    sih_msgs_msg.MissionEvent = MagicMock
    sih_msgs_msg.TrackedObstacle = MagicMock
    sih_msgs_msg.TrackedObstacleArray = MagicMock
    sih_msgs_msg.AutonomyMode = MagicMock
    sih_msgs_msg.SensorHealth = MagicMock
    sih_msgs.msg = sih_msgs_msg

    # ---- numpy (real, but make sure it's loaded) -------------------------
    import numpy  # noqa: F401  real numpy must be available


# ---------------------------------------------------------------------------
# Save the original sys.modules state for every key that _ensure_rclpy_stubs()
# will replace.  After installing the stubs and reloading mission_manager to
# capture MissionManager (which bakes the stub Node into its MRO), we restore
# the original modules so that other test files collected later in the same
# pytest session (e.g. test_online_mapping.py) see the real rclpy / geometry_msgs
# instead of our lightweight stubs.
# ---------------------------------------------------------------------------
_STUB_MODULE_NAMES = [
    'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.exceptions',
    'geometry_msgs', 'geometry_msgs.msg',
    'std_msgs', 'std_msgs.msg',
    'nav_msgs', 'nav_msgs.msg',
    'ros_gz_interfaces', 'ros_gz_interfaces.msg',
    'sih26126_msgs', 'sih26126_msgs.msg',
]

_original_modules = {}
for _mod_name in _STUB_MODULE_NAMES:
    _original_modules[_mod_name] = sys.modules.get(_mod_name)

_ensure_rclpy_stubs()

# ---------------------------------------------------------------------------
# Force-reload mission_manager so it picks up the stubs installed above.
# Without this reload, if test_cost_aware_routing.py was collected first and
# triggered a real rclpy import, the sih26126_autonomy.mission_manager module
# in sys.modules would still reference the real rclpy.node.Node, causing
# NotInitializedException when MissionManager() is called here.
# ---------------------------------------------------------------------------
import sih26126_autonomy.mission_manager as _mm_mod  # noqa: E402
_mm_mod = importlib.reload(_mm_mod)
MissionManager = _mm_mod.MissionManager

# ---------------------------------------------------------------------------
# Restore the original sys.modules so that stubs do not leak into other test
# files.  MissionManager is already bound to the stub Node via its MRO; it does
# not need sys.modules['rclpy'] to point at the stub at runtime.
# ---------------------------------------------------------------------------
for _mod_name in _STUB_MODULE_NAMES:
    _orig = _original_modules[_mod_name]
    if _orig is not None:
        sys.modules[_mod_name] = _orig
    else:
        # Was not in sys.modules before we touched it; remove our stub.
        sys.modules.pop(_mod_name, None)


# ---------------------------------------------------------------------------
# Test harness helpers
# ---------------------------------------------------------------------------

def _build_manager(mission_type='EXPLORATION', **extra_params):
    """Instantiate MissionManager and override the mission_type parameter.

    Returns the node, bypassing the rclpy.init requirement because the Node
    base class is our stub.
    """
    mgr = MissionManager()
    # Override parameter values directly on the internal dict
    mgr._params['mission_type'] = mission_type
    mgr._params['mission_timeout_sec'] = 1800.0
    mgr._params['minimum_explored_area'] = 24.0
    mgr._params['frontier_threshold'] = 0
    mgr._params['minimum_completed_frontiers'] = 3
    mgr._params['home_tolerance_m'] = 0.35
    mgr._params['heading_tolerance_rad'] = 0.20
    mgr._params['stationary_linear_mps'] = 0.04
    mgr._params['stationary_angular_rps'] = 0.06
    mgr._params['stationary_dwell_sec'] = 2.0
    mgr._params['frontier_exhaustion_grace_sec'] = 15.0
    mgr._params['home_route_unavailable_timeout_sec'] = 3.0
    for k, v in extra_params.items():
        mgr._params[k] = v
    return mgr


def _give_pose(mgr, x=0.0, y=0.0):
    """Inject a minimal valid pose into the manager."""
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    # The pose_cb does many checks; bypass it and set directly
    mgr.pose = ps
    mgr.raw_pose = ps
    mgr.last_pose_xy = (x, y)
    return ps


def _give_map_with_free_cells(mgr, n_free=25):
    """Inject a minimal OccupancyGrid that will trigger MAPPING→EXPLORING."""
    import numpy as np
    from nav_msgs.msg import OccupancyGrid
    grid = OccupancyGrid()
    grid.info.resolution = 0.05
    # n_free cells of value 0 (free), rest -1 (unknown)
    data = [-1] * 100
    for i in range(n_free):
        data[i] = 0
    grid.data = data
    mgr.map = grid
    return grid


# ---------------------------------------------------------------------------
# 1. WAYPOINT mission: INITIALIZING → MAPPING → EXECUTING_PROFILE → RETURNING_HOME
# ---------------------------------------------------------------------------

class TestWaypointMissionStateTransitions:
    """WAYPOINT mission state machine exercises."""

    def test_initializing_to_mapping_on_first_pose(self):
        """process() transitions INITIALIZING → MAPPING when pose arrives."""
        mgr = _build_manager(mission_type='WAYPOINT')
        assert mgr.state == 'INITIALIZING'

        _give_pose(mgr)
        mgr.process()

        assert mgr.state == 'MAPPING', (
            f'Expected MAPPING after first pose, got {mgr.state!r}')

    def test_mapping_to_executing_profile_once_home_captured(self):
        """After home is captured, profile branch transitions MAPPING → EXECUTING_PROFILE."""
        mgr = _build_manager(mission_type='WAYPOINT')
        _give_pose(mgr)
        mgr.process()  # captures home, enters MAPPING

        assert mgr.state == 'MAPPING'
        assert mgr.home is not None

        # Second process() call: home is set, mission_type != EXPLORATION
        # → should transition to EXECUTING_PROFILE
        mgr.process()

        assert mgr.state == 'EXECUTING_PROFILE', (
            f'Expected EXECUTING_PROFILE after home captured, got {mgr.state!r}')

    def test_executing_profile_to_returning_home_on_completed(self):
        """profile_state='COMPLETED' while EXECUTING_PROFILE → RETURNING_HOME.

        The robot's pose is set far from home so the shared RETURNING_HOME
        block does not immediately advance to FINAL_ALIGNMENT in the same tick.
        """
        mgr = _build_manager(mission_type='WAYPOINT')
        _give_pose(mgr)
        mgr.process()   # MAPPING
        mgr.process()   # EXECUTING_PROFILE

        # Move robot far from home before profile completes (realistic)
        _give_pose(mgr, x=10.0, y=10.0)

        # Simulate profile controller reporting completion
        mgr.profile_state = 'COMPLETED'
        mgr.process()

        assert mgr.state == 'RETURNING_HOME', (
            f'Expected RETURNING_HOME after profile COMPLETED, got {mgr.state!r}')

    def test_full_waypoint_path_initializing_mapping_executing_returning(self):
        """Full chain: INITIALIZING → MAPPING → EXECUTING_PROFILE → RETURNING_HOME.

        We move the pose far from home before the COMPLETED signal so the
        shared RETURNING_HOME block does not advance past RETURNING_HOME in
        the same process() tick.
        """
        mgr = _build_manager(mission_type='WAYPOINT')

        states = [mgr.state]
        _give_pose(mgr)
        mgr.process()           # INITIALIZING → MAPPING
        states.append(mgr.state)

        mgr.process()           # MAPPING → EXECUTING_PROFILE
        states.append(mgr.state)

        # Robot has moved far from home during profile execution
        _give_pose(mgr, x=10.0, y=10.0)
        mgr.profile_state = 'COMPLETED'
        mgr.process()           # EXECUTING_PROFILE → RETURNING_HOME
        states.append(mgr.state)

        assert states == ['INITIALIZING', 'MAPPING', 'EXECUTING_PROFILE', 'RETURNING_HOME'], (
            f'Unexpected state chain: {states}')


# ---------------------------------------------------------------------------
# 2. COVERAGE mission: profile_state COMPLETED triggers RETURNING_HOME
# ---------------------------------------------------------------------------

class TestCoverageMissionCompletion:
    """COVERAGE mission completion handling."""

    def test_completed_triggers_returning_home(self):
        mgr = _build_manager(mission_type='COVERAGE')
        _give_pose(mgr)
        mgr.process()   # MAPPING
        mgr.process()   # EXECUTING_PROFILE

        # Robot has moved away from home during coverage run
        _give_pose(mgr, x=8.0, y=8.0)
        mgr.profile_state = 'COMPLETED'
        mgr.process()

        assert mgr.state == 'RETURNING_HOME', (
            f'COVERAGE COMPLETED must set RETURNING_HOME, got {mgr.state!r}')

    def test_completed_prefix_variants_all_trigger_returning_home(self):
        """Profile controller may emit 'COMPLETED_X_OF_Y'; the prefix check
        must still trigger RETURNING_HOME."""
        for completed_msg in ('COMPLETED', 'COMPLETED_ALL', 'COMPLETED_10_OF_10'):
            mgr = _build_manager(mission_type='COVERAGE')
            _give_pose(mgr)
            mgr.process()
            mgr.process()

            # Robot away from home so RETURNING_HOME doesn't collapse to FINAL_ALIGNMENT
            _give_pose(mgr, x=7.0, y=7.0)
            mgr.profile_state = completed_msg
            mgr.process()

            assert mgr.state == 'RETURNING_HOME', (
                f'profile_state={completed_msg!r} must trigger RETURNING_HOME, '
                f'got {mgr.state!r}')

    def test_partial_progress_does_not_trigger_returning_home(self):
        """While profile reports a waypoint index string (not COMPLETED/FAILED),
        state must stay EXECUTING_PROFILE."""
        mgr = _build_manager(mission_type='COVERAGE')
        _give_pose(mgr)
        mgr.process()
        mgr.process()

        mgr.profile_state = 'WAYPOINT_3_OF_10'
        mgr.process()

        assert mgr.state == 'EXECUTING_PROFILE', (
            f'Partial profile progress must keep EXECUTING_PROFILE, got {mgr.state!r}')


# ---------------------------------------------------------------------------
# 3. SEARCH mission: profile_state FAILED triggers FAILED state
# ---------------------------------------------------------------------------

class TestSearchMissionFailure:
    """SEARCH mission failure handling."""

    def test_failed_state_on_profile_failed(self):
        mgr = _build_manager(mission_type='SEARCH')
        _give_pose(mgr)
        mgr.process()   # MAPPING
        mgr.process()   # EXECUTING_PROFILE

        mgr.profile_state = 'FAILED'
        mgr.process()

        assert mgr.state == 'FAILED', (
            f'profile_state=FAILED must set FAILED, got {mgr.state!r}')

    def test_failed_prefix_variants_trigger_failed(self):
        """'FAILED_TIMEOUT', 'FAILED_OBSTACLE' etc. must all trigger FAILED."""
        for fail_msg in ('FAILED', 'FAILED_TIMEOUT', 'FAILED_OBSTACLE_BLOCKED'):
            mgr = _build_manager(mission_type='SEARCH')
            _give_pose(mgr)
            mgr.process()
            mgr.process()

            mgr.profile_state = fail_msg
            mgr.process()

            assert mgr.state == 'FAILED', (
                f'profile_state={fail_msg!r} must trigger FAILED, got {mgr.state!r}')

    def test_failure_reason_is_set_on_failed(self):
        """failure_reason must be non-empty when FAILED is triggered by profile."""
        mgr = _build_manager(mission_type='SEARCH')
        _give_pose(mgr)
        mgr.process()
        mgr.process()

        mgr.profile_state = 'FAILED_TIMEOUT'
        mgr.process()

        assert mgr.state == 'FAILED'
        assert mgr.failure_reason, 'failure_reason must be set when profile reports FAILED'


# ---------------------------------------------------------------------------
# 4. EXPLORATION mission: existing frontier behavior unaffected
# ---------------------------------------------------------------------------

class TestExplorationMissionUnchanged:
    """EXPLORATION mission must still use frontier logic unchanged."""

    def test_mapping_to_exploring_via_map_free_cells(self):
        """When mission_type='EXPLORATION' and the map has free cells,
        process() should transition MAPPING → EXPLORING."""
        mgr = _build_manager(mission_type='EXPLORATION')
        _give_pose(mgr)
        mgr.process()   # captures home → MAPPING

        assert mgr.state == 'MAPPING'

        # Inject map with 25 free cells (>20 threshold)
        _give_map_with_free_cells(mgr, n_free=25)
        mgr.process()   # should transition to EXPLORING

        assert mgr.state == 'EXPLORING', (
            f'EXPLORATION mission must transition MAPPING→EXPLORING via map, '
            f'got {mgr.state!r}')

    def test_exploration_does_not_enter_executing_profile(self):
        """EXPLORATION mission must never enter EXECUTING_PROFILE."""
        mgr = _build_manager(mission_type='EXPLORATION')
        _give_pose(mgr)
        mgr.process()
        _give_map_with_free_cells(mgr, n_free=25)
        mgr.process()

        assert mgr.state != 'EXECUTING_PROFILE', (
            'EXPLORATION mission must not enter EXECUTING_PROFILE')

    def test_exploration_profile_state_does_not_affect_state_machine(self):
        """Even if profile_state is 'COMPLETED', EXPLORATION ignores it."""
        mgr = _build_manager(mission_type='EXPLORATION')
        _give_pose(mgr)
        mgr.process()
        _give_map_with_free_cells(mgr, n_free=25)
        mgr.process()   # EXPLORING

        mgr.profile_state = 'COMPLETED'
        mgr.process()   # should still be EXPLORING, not RETURNING_HOME

        assert mgr.state == 'EXPLORING', (
            'EXPLORATION mission must ignore profile_state=COMPLETED')


# ---------------------------------------------------------------------------
# 5. No goal conflict: mission_manager must NOT publish /mission/goal during
#    EXECUTING_PROFILE when mission_type != 'EXPLORATION'
# ---------------------------------------------------------------------------

class TestNoGoalConflict:
    """Verify mission_manager does not publish /mission/goal during
    EXECUTING_PROFILE to avoid two concurrent publishers on that topic."""

    def _count_goal_publishes(self, mgr):
        """Return the number of times /mission/goal was published."""
        pub = mgr._pubs.get('/mission/goal')
        if pub is None:
            return 0
        return len(pub._published)

    def test_no_goal_published_while_executing_profile_waypoint(self):
        """During WAYPOINT execution, mission_manager must not call
        goal_pub.publish() (the profile controller owns /mission/goal)."""
        mgr = _build_manager(mission_type='WAYPOINT')
        _give_pose(mgr)
        mgr.process()   # MAPPING
        mgr.process()   # EXECUTING_PROFILE

        publishes_before = self._count_goal_publishes(mgr)

        # Multiple process() cycles while profile is in progress
        for _ in range(5):
            mgr.profile_state = 'WAYPOINT_2_OF_8'
            mgr.process()

        publishes_after = self._count_goal_publishes(mgr)
        assert publishes_before == publishes_after, (
            f'mission_manager must not publish /mission/goal during EXECUTING_PROFILE '
            f'(got {publishes_after - publishes_before} extra publish(es))')

    def test_no_goal_published_while_executing_profile_coverage(self):
        """Same check for COVERAGE mission."""
        mgr = _build_manager(mission_type='COVERAGE')
        _give_pose(mgr)
        mgr.process()
        mgr.process()

        publishes_before = self._count_goal_publishes(mgr)

        for _ in range(3):
            mgr.profile_state = 'WAYPOINT_1_OF_20'
            mgr.process()

        publishes_after = self._count_goal_publishes(mgr)
        assert publishes_before == publishes_after, (
            f'mission_manager must not publish /mission/goal during COVERAGE EXECUTING_PROFILE '
            f'(got {publishes_after - publishes_before} extra publish(es))')

    def test_no_goal_published_while_executing_profile_search(self):
        """Same check for SEARCH mission."""
        mgr = _build_manager(mission_type='SEARCH')
        _give_pose(mgr)
        mgr.process()
        mgr.process()

        publishes_before = self._count_goal_publishes(mgr)

        mgr.profile_state = 'WAYPOINT_0_OF_5'
        mgr.process()

        publishes_after = self._count_goal_publishes(mgr)
        assert publishes_before == publishes_after, (
            'mission_manager must not publish /mission/goal during SEARCH EXECUTING_PROFILE')

    def test_exploration_still_publishes_goal_on_return_home(self):
        """EXPLORATION mission must still call goal_pub.publish() when
        transitioning to RETURNING_HOME (the home-return goal)."""
        mgr = _build_manager(mission_type='EXPLORATION')
        _give_pose(mgr)
        mgr.process()   # MAPPING → captured home

        # Force RETURNING_HOME directly (simulate criteria already met)
        mgr.state = 'RETURNING_HOME'
        mgr.exploration_completion_reason = 'forced_for_test'
        mgr.path_status = ''   # not HOME_ROUTE_UNAVAILABLE
        # home is already set by process(); pose is set; home_distance > tolerance
        # so we don't enter FINAL_ALIGNMENT yet
        _give_pose(mgr, x=10.0, y=10.0)   # far from home (0,0)

        publishes_before = self._count_goal_publishes(mgr)
        mgr.process()
        publishes_after = self._count_goal_publishes(mgr)

        assert publishes_after > publishes_before, (
            'EXPLORATION RETURNING_HOME must publish /mission/goal for the planner')
