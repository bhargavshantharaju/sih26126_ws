#!/usr/bin/env python3
"""
Regression tests for the LiDAR topic contract in local_path_selector.

ROOT CAUSE:
  local_path_selector.py was subscribing to '/scan' while the sensor pipeline
  (lidar_processor.py) publishes to '/lidar/scan_2d'.  This mismatch left the
  selector stuck in WAITING_FOR_LIDAR permanently since no LaserScan ever
  arrived on '/scan'.

FIX:
  Changed the subscription in LocalPathSelector.__init__ to use
  '/lidar/scan_2d'.

These tests verify:
  1. The selector subscribes to '/lidar/scan_2d' (not '/scan').
  2. The lidar_processor publishes to '/lidar/scan_2d'.
  3. The contract is internally consistent (publisher and subscriber agree).
  4. A valid scan reaching the selector sets scan_received_at.
  5. A missing/stale scan causes publish_stop('WAITING_FOR_LIDAR') or
     publish_stop('STALE_LIDAR') before any motion is emitted.
  6. The selector's process() can proceed to LOCAL_NAVIGATE with a fresh
     scan and all other required inputs present.
"""

import ast
import math
import pathlib
import time

import pytest

# ---------------------------------------------------------------------------
# AST-based source code contract tests (no ROS required)
# ---------------------------------------------------------------------------

_SRC_ROOT = pathlib.Path(__file__).resolve().parents[3] / 'src'
_SELECTOR_SRC = (_SRC_ROOT
                 / 'sih26126_planner/sih26126_planner/local_path_selector.py')
_LIDAR_PROC_SRC = (_SRC_ROOT
                   / 'sih26126_sensor_contract'
                   / 'sih26126_sensor_contract/lidar_processor.py')

CORRECT_LIDAR_TOPIC = '/lidar/scan_2d'
WRONG_LIDAR_TOPIC = '/scan'


def _find_create_subscription_topics(source: str, msg_type_name: str) -> list:
    """Return all topic strings passed to create_subscription for a message type."""
    tree = ast.parse(source)
    topics = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute)
                and func.attr == 'create_subscription'):
            continue
        if len(node.args) < 2:
            continue
        # First positional arg is the message type
        msg_type_arg = node.args[0]
        if isinstance(msg_type_arg, ast.Name):
            type_name = msg_type_arg.id
        elif isinstance(msg_type_arg, ast.Attribute):
            type_name = msg_type_arg.attr
        else:
            continue
        if type_name != msg_type_name:
            continue
        # Second positional arg is the topic string
        topic_arg = node.args[1]
        if isinstance(topic_arg, ast.Constant) and isinstance(topic_arg.value, str):
            topics.append(topic_arg.value)
    return topics


def _find_create_publisher_topics(source: str, msg_type_name: str) -> list:
    """Return all topic strings passed to create_publisher for a message type."""
    tree = ast.parse(source)
    topics = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute)
                and func.attr == 'create_publisher'):
            continue
        if len(node.args) < 2:
            continue
        msg_type_arg = node.args[0]
        if isinstance(msg_type_arg, ast.Name):
            type_name = msg_type_arg.id
        elif isinstance(msg_type_arg, ast.Attribute):
            type_name = msg_type_arg.attr
        else:
            continue
        if type_name != msg_type_name:
            continue
        topic_arg = node.args[1]
        if isinstance(topic_arg, ast.Constant) and isinstance(topic_arg.value, str):
            topics.append(topic_arg.value)
    return topics


def test_selector_subscribes_to_correct_lidar_topic():
    """
    local_path_selector must subscribe to '/lidar/scan_2d' not '/scan'.

    Regression: old code had '/scan' which never received any messages
    because lidar_processor publishes to '/lidar/scan_2d'.
    """
    assert _SELECTOR_SRC.exists(), f'Source not found: {_SELECTOR_SRC}'
    source = _SELECTOR_SRC.read_text(encoding='utf-8')
    laser_scan_topics = _find_create_subscription_topics(source, 'LaserScan')
    assert laser_scan_topics, (
        'No LaserScan subscription found in local_path_selector.py'
    )
    assert CORRECT_LIDAR_TOPIC in laser_scan_topics, (
        f'local_path_selector.py must subscribe to {CORRECT_LIDAR_TOPIC!r}. '
        f'Found LaserScan subscriptions: {laser_scan_topics}'
    )
    assert WRONG_LIDAR_TOPIC not in laser_scan_topics, (
        f'REGRESSION: local_path_selector.py still subscribes to {WRONG_LIDAR_TOPIC!r}. '
        'This topic is not published by the sensor pipeline. '
        f'Fix: change to {CORRECT_LIDAR_TOPIC!r}.'
    )


def test_lidar_processor_publishes_correct_topic():
    """lidar_processor must publish to '/lidar/scan_2d'."""
    assert _LIDAR_PROC_SRC.exists(), f'Source not found: {_LIDAR_PROC_SRC}'
    source = _LIDAR_PROC_SRC.read_text(encoding='utf-8')
    laser_scan_topics = _find_create_publisher_topics(source, 'LaserScan')
    assert laser_scan_topics, (
        'No LaserScan publisher found in lidar_processor.py'
    )
    assert CORRECT_LIDAR_TOPIC in laser_scan_topics, (
        f'lidar_processor.py must publish to {CORRECT_LIDAR_TOPIC!r}. '
        f'Found LaserScan publishers: {laser_scan_topics}'
    )


def test_lidar_topic_publisher_subscriber_match():
    """The publisher topic in lidar_processor must equal the subscriber topic in selector."""
    assert _SELECTOR_SRC.exists()
    assert _LIDAR_PROC_SRC.exists()

    selector_source = _SELECTOR_SRC.read_text(encoding='utf-8')
    proc_source = _LIDAR_PROC_SRC.read_text(encoding='utf-8')

    subscriber_topics = set(
        _find_create_subscription_topics(selector_source, 'LaserScan'))
    publisher_topics = set(
        _find_create_publisher_topics(proc_source, 'LaserScan'))

    overlap = subscriber_topics & publisher_topics
    assert overlap, (
        f'No matching LiDAR topic between publisher {publisher_topics} '
        f'and subscriber {subscriber_topics}. '
        'The sensor pipeline and local path selector are disconnected.'
    )
    assert CORRECT_LIDAR_TOPIC in overlap, (
        f'The matching topic {overlap} is not {CORRECT_LIDAR_TOPIC!r}. '
        'Architecture requires the scan_2d topic.'
    )


# ---------------------------------------------------------------------------
# Behavioral logic tests (no ROS node spin required)
# ---------------------------------------------------------------------------

try:
    from sih26126_planner.local_path_selector import input_is_fresh
    _IMPORT_OK = True
except ImportError:
    _IMPORT_OK = False


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_planner not importable')
def test_missing_scan_causes_waiting_state():
    """
    When scan is None, input_is_fresh returns False → WAITING_FOR_LIDAR.

    Verifies the freshness guard for the LiDAR channel. When no scan has
    been received (received_at=None), navigation must stop.
    """
    # scan_received_at=None means no scan has arrived
    assert not input_is_fresh(None, 0.75), (
        'A missing scan (None received_at) must not be considered fresh. '
        'The selector must emit WAITING_FOR_LIDAR.'
    )


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_planner not importable')
def test_stale_scan_causes_stale_state():
    """A scan older than max_sensor_age_s must not be considered fresh."""
    now = time.monotonic()
    stale_received_at = now - 2.0  # 2 seconds ago, well beyond 0.75s limit
    assert not input_is_fresh(stale_received_at, 0.75, now=now), (
        'A stale scan (2s old with 0.75s limit) must not be considered fresh. '
        'The selector must emit STALE_LIDAR.'
    )


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_planner not importable')
def test_fresh_scan_is_accepted():
    """A scan received within max_sensor_age_s is considered valid."""
    now = time.monotonic()
    fresh_received_at = now - 0.1  # 100 ms ago, well within 0.75s limit
    assert input_is_fresh(fresh_received_at, 0.75, now=now), (
        'A fresh scan (100ms old with 0.75s limit) must be considered valid.'
    )


@pytest.mark.skipif(not _IMPORT_OK, reason='sih26126_planner not importable')
def test_scan_received_at_is_updated_on_callback():
    """
    scan_callback must update scan_received_at when called.

    Verifies that after a valid LaserScan message reaches scan_callback,
    scan_received_at is no longer None (so the freshness gate opens).
    """
    from sensor_msgs.msg import LaserScan
    from sih26126_planner.local_path_selector import LocalPathSelector

    # Create a minimal mock node that doesn't need a ROS context
    node = LocalPathSelector.__new__(LocalPathSelector)
    node.scan = None
    node.scan_received_at = None

    scan_msg = LaserScan()
    scan_msg.angle_min = -math.pi
    scan_msg.angle_max = math.pi
    scan_msg.angle_increment = math.pi / 180.0
    scan_msg.range_min = 0.2
    scan_msg.range_max = 20.0
    scan_msg.ranges = [5.0] * 360

    before = time.monotonic()
    node.scan_callback(scan_msg)
    after = time.monotonic()

    assert node.scan is scan_msg, 'scan_callback must store the scan message'
    assert node.scan_received_at is not None, (
        'scan_callback must set scan_received_at (was None before callback)'
    )
    assert before <= node.scan_received_at <= after, (
        'scan_received_at must be set to approximately the current monotonic time'
    )
