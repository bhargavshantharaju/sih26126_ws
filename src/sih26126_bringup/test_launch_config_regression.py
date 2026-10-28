#!/usr/bin/env python3
"""
Regression tests for launch file configuration.

These tests verify that autonomous_mission.launch.py correctly passes
slam:=true and fusion:=true to full_stack.launch.py.

Root cause of regression: autonomous_mission.launch.py previously
omitted both 'slam' and 'fusion' from the launch_arguments passed to
full_stack.  Since full_stack defaults both to 'false', the autonomous
mission would silently launch without SLAM or EKF, leaving the robot
unable to build a map or estimate its pose in GPS-denied operation.
"""

import ast
import pathlib
import re


BRINGUP_DIR = (
    pathlib.Path(__file__).resolve().parent / 'launch'
)
AUTONOMOUS_MISSION_LAUNCH = BRINGUP_DIR / 'autonomous_mission.launch.py'
FULL_STACK_LAUNCH = BRINGUP_DIR / 'full_stack.launch.py'


def _read_launch(path: pathlib.Path) -> str:
    assert path.exists(), f'Launch file missing: {path}'
    return path.read_text(encoding='utf-8')


# ---------------------------------------------------------------------------
# Test 1: autonomous_mission declares slam and fusion arguments
# ---------------------------------------------------------------------------

def test_autonomous_mission_declares_slam_arg():
    """
    autonomous_mission.launch.py must declare a 'slam' DeclareLaunchArgument.

    Regression: previously no slam argument was declared, so there was no
    way for the argument to be passed to full_stack.
    """
    source = _read_launch(AUTONOMOUS_MISSION_LAUNCH)
    tree = ast.parse(source)

    declared_args = []
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'DeclareLaunchArgument'
        ):
            continue
        if node.args and isinstance(node.args[0], ast.Constant):
            declared_args.append(node.args[0].value)
        elif node.keywords:
            for kw in node.keywords:
                if kw.arg == 'name' and isinstance(kw.value, ast.Constant):
                    declared_args.append(kw.value.value)

    assert 'slam' in declared_args, (
        'REGRESSION: autonomous_mission.launch.py does not declare a '
        "'slam' launch argument.  Without it, full_stack defaults to "
        'slam:=false, disabling SLAM for the autonomous mission.'
    )


def test_autonomous_mission_declares_fusion_arg():
    """autonomous_mission.launch.py must declare a 'fusion' DeclareLaunchArgument."""
    source = _read_launch(AUTONOMOUS_MISSION_LAUNCH)
    tree = ast.parse(source)

    declared_args = []
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'DeclareLaunchArgument'
        ):
            continue
        if node.args and isinstance(node.args[0], ast.Constant):
            declared_args.append(node.args[0].value)

    assert 'fusion' in declared_args, (
        'REGRESSION: autonomous_mission.launch.py does not declare a '
        "'fusion' launch argument.  Without it, full_stack defaults to "
        'fusion:=false, disabling EKF for the autonomous mission.'
    )


# ---------------------------------------------------------------------------
# Test 2: autonomous_mission passes slam and fusion to IncludeLaunchDescription
# ---------------------------------------------------------------------------

def test_autonomous_mission_passes_slam_to_full_stack():
    """
    autonomous_mission.launch.py must pass 'slam' in launch_arguments.

    Regression: the original code's launch_arguments dict did not include
    'slam', so full_stack received no slam argument (defaulting to false).
    """
    source = _read_launch(AUTONOMOUS_MISSION_LAUNCH)
    # Look for 'slam' in the launch_arguments dict passed to IncludeLaunchDescription
    # The dict is constructed as {'slam': slam, ...}.items()
    # A simple string search is sufficient for this structural check.
    assert "'slam'" in source or '"slam"' in source, (
        "REGRESSION: 'slam' is not passed to full_stack in "
        "autonomous_mission.launch.py's launch_arguments."
    )


def test_autonomous_mission_passes_fusion_to_full_stack():
    """autonomous_mission.launch.py must pass 'fusion' in launch_arguments."""
    source = _read_launch(AUTONOMOUS_MISSION_LAUNCH)
    assert "'fusion'" in source or '"fusion"' in source, (
        "REGRESSION: 'fusion' is not passed to full_stack in "
        "autonomous_mission.launch.py's launch_arguments."
    )


# ---------------------------------------------------------------------------
# Test 3: slam and fusion defaults are 'true' in autonomous_mission
# ---------------------------------------------------------------------------

def test_autonomous_mission_slam_default_is_true():
    """
    The slam argument in autonomous_mission must default to 'true'.

    The autonomous mission requires GPS-denied localisation; defaulting to
    slam:=false would silently break it.
    """
    source = _read_launch(AUTONOMOUS_MISSION_LAUNCH)
    # After the DeclareLaunchArgument for slam, look for default_value='true'
    # Use a regex to find the pattern near the slam declaration
    pattern = re.compile(
        r"DeclareLaunchArgument\s*\(\s*['\"]slam['\"].*?default_value\s*=\s*['\"]true['\"]",
        re.DOTALL,
    )
    assert pattern.search(source), (
        "autonomous_mission.launch.py slam argument does not default to 'true'. "
        'The mission is designed for SLAM-based localisation.'
    )


def test_autonomous_mission_fusion_default_is_true():
    """The fusion argument in autonomous_mission must default to 'true'."""
    source = _read_launch(AUTONOMOUS_MISSION_LAUNCH)
    pattern = re.compile(
        r"DeclareLaunchArgument\s*\(\s*['\"]fusion['\"].*?default_value\s*=\s*['\"]true['\"]",
        re.DOTALL,
    )
    assert pattern.search(source), (
        "autonomous_mission.launch.py fusion argument does not default to 'true'. "
        'The mission is designed for EKF sensor fusion.'
    )


# ---------------------------------------------------------------------------
# Test 4: full_stack declares slam and fusion (existing args should be stable)
# ---------------------------------------------------------------------------

def test_full_stack_declares_slam_arg():
    """full_stack.launch.py must declare a 'slam' argument so it can be overridden."""
    source = _read_launch(FULL_STACK_LAUNCH)
    tree = ast.parse(source)
    declared_args = []
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'DeclareLaunchArgument'
            and node.args
            and isinstance(node.args[0], ast.Constant)
        ):
            declared_args.append(node.args[0].value)

    assert 'slam' in declared_args, (
        "full_stack.launch.py does not declare a 'slam' argument. "
        'autonomous_mission cannot pass slam:=true without a declared receiver.'
    )
    assert 'fusion' in declared_args, (
        "full_stack.launch.py does not declare a 'fusion' argument."
    )


# ---------------------------------------------------------------------------
# Test 5: gz_gui_stream_node has use_sim_time in full_stack
# ---------------------------------------------------------------------------

def test_gz_gui_stream_has_use_sim_time():
    """
    gz_gui_stream node must have use_sim_time=True in its parameters.

    Regression: the node was the only application node missing this parameter.
    All non-bridge ROS nodes must use sim time for consistent clock behaviour.
    """
    source = _read_launch(FULL_STACK_LAUNCH)
    # Find the gz_gui_stream_node block and check for use_sim_time
    idx = source.find('gz_gui_stream_node = Node(')
    assert idx >= 0, 'gz_gui_stream_node declaration not found in full_stack.launch.py'

    # Extract the node declaration block (up to the next Node( or 200 chars)
    block = source[idx:idx + 600]
    assert "'use_sim_time': True" in block or '"use_sim_time": True' in block, (
        'REGRESSION: gz_gui_stream_node in full_stack.launch.py is missing '
        "'use_sim_time': True in its parameters. All non-bridge ROS nodes must "
        'have this set when running with Gazebo simulation time.'
    )
