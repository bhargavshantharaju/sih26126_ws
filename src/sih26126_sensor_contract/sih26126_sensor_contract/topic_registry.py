#!/usr/bin/env python3
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
"""DRISTHI canonical topic registry.

Provides programmatic access to the canonical sensor/topic contract
defined in config/topics.yaml.  Use this module instead of hardcoding
topic strings throughout the codebase.

Usage example::

    from sih26126_sensor_contract.topic_registry import Topics, get_topics
    t = Topics()
    scan_topic = t.sensors.lidar_scan_2d          # '/lidar/scan_2d'
    cmd_vel    = t.safety.cmd_vel                 # '/cmd_vel'
    # singleton:
    scan_topic = get_topics().sensors.lidar_scan_2d
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import yaml

_DEFAULT_CONFIG_PATH = Path(__file__).parent.parent / 'config' / 'topics.yaml'


class _Namespace:
    """Attribute-style access to a flat YAML section."""

    def __init__(self, data: dict) -> None:
        for key, value in data.items():
            setattr(self, key, value)

    def get(self, key: str, default: Optional[str] = None) -> Optional[str]:
        """Return topic string by key with optional default."""
        return getattr(self, key, default)

    def all_topics(self) -> list:
        """Return all topic strings in this namespace."""
        return [v for v in vars(self).values() if isinstance(v, str)]


class Topics:
    """Accessor for DRISTHI canonical topic names.

    Loads from config/topics.yaml.  Provides attribute-style access
    per namespace (sensors, localization, planning, etc.).
    """

    def __init__(self, config_path: Optional[str] = None) -> None:
        """Load topics from YAML, falling back to ament share directory."""
        path = Path(config_path) if config_path else _DEFAULT_CONFIG_PATH

        if not path.exists():
            try:
                from ament_index_python.packages import (
                    get_package_share_directory,
                )
                share = get_package_share_directory('sih26126_sensor_contract')
                path = Path(share) / 'config' / 'topics.yaml'
            except Exception:
                pass

        if not path.exists():
            raise FileNotFoundError(
                f'DRISTHI topic registry not found at {path}. '
                'Ensure sih26126_sensor_contract is built and installed.'
            )

        with open(path, 'r') as fh:
            raw: dict = yaml.safe_load(fh)

        for section, data in raw.items():
            if isinstance(data, dict):
                setattr(self, section, _Namespace(data))

    def get_topic(
        self,
        section: str,
        key: str,
        default: Optional[str] = None,
    ) -> Optional[str]:
        """Return a topic string by section and key."""
        ns = getattr(self, section, None)
        if ns is None:
            return default
        return ns.get(key, default)

    def all_topics(self) -> list:
        """Return every topic string defined in the registry."""
        result: list = []
        for ns in vars(self).values():
            if isinstance(ns, _Namespace):
                result.extend(ns.all_topics())
        return result


_instance: Optional[Topics] = None


def get_topics() -> Topics:
    """Return the global Topics singleton (loaded on first call)."""
    global _instance
    if _instance is None:
        _instance = Topics()
    return _instance
