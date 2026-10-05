#!/usr/bin/env python3
"""Read-only live autonomy dashboard driven by current ROS messages."""

import base64
import json
import math
import os
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu, PointCloud2
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32, Int32, String
from visualization_msgs.msg import MarkerArray
from sih26126_msgs.msg import AutonomyMode, MissionEvent, PlannerRisk, TrackedObstacleArray


HOST = '0.0.0.0'
PORT = int(os.environ.get('DRISTHI_DASHBOARD_PORT', '8080'))
LOCK = threading.RLock()
STARTED = time.monotonic()
EVENTS = deque(maxlen=120)
TOPIC_TYPES = {}
TOPIC_TIMES = {}
TOPIC_COUNTS = {}
TOPIC_WINDOWS = {}
CLOUD_TIMES = deque(maxlen=80)


def event(message, reason='', value=''):
    """Append a timestamped event with an optional reason and relevant value."""
    EVENTS.appendleft({
        'time': datetime.now().strftime('%H:%M:%S'),
        'message': str(message),
        'reason': str(reason) if reason else '',
        'value': str(value) if value else '',
    })


def jpeg_rgb(msg):
    if msg.encoding not in ('rgb8', 'bgr8') or msg.step < msg.width * 3:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    needed = msg.step * msg.height
    if raw.size < needed:
        return None
    image = raw[:needed].reshape(msg.height, msg.step)[:, :msg.width * 3]
    image = image.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
    return encoded.tobytes() if ok else None


def jpeg_traversability(msg):
    if msg.encoding != 'mono8' or msg.step < msg.width:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < msg.step * msg.height:
        return None
    image = raw[:msg.step * msg.height].reshape(msg.height, msg.step)[:, :msg.width]
    colored = cv2.applyColorMap(image, cv2.COLORMAP_TURBO)
    colored[image == 0] = (20, 25, 30)
    ok, encoded = cv2.imencode('.jpg', colored, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
    return encoded.tobytes() if ok else None


def cloud_stats(msg):
    """Compute bounded statistics from sampled real XYZ fields."""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')) or msg.point_step <= 0:
        return None
    if any(fields[k].datatype != 7 for k in ('x', 'y', 'z')):
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < msg.row_step * msg.height:
        return None
    sampled = []
    for name in ('x', 'y', 'z'):
        f = fields[name]
        a = np.ndarray(
            (msg.height, msg.width), dtype=np.float32, buffer=raw,
            offset=f.offset, strides=(msg.row_step, msg.point_step),
        )[::8, ::8]
        sampled.append(a.reshape(-1))
    xyz = np.column_stack(sampled)
    valid = np.isfinite(xyz).all(axis=1)
    finite = xyz[valid]
    if not len(finite):
        return {'finite': 0, 'samples': len(xyz), 'nearest': None}
    ranges = np.min(finite, axis=0), np.max(finite, axis=0)
    obstacle = (
        (finite[:, 0] > 0.2) & (finite[:, 0] < 20.0)
        & (finite[:, 2] > -0.96) & (finite[:, 2] < 2.5)
    )
    obstacle_points = finite[obstacle]
    projected = finite[
        (finite[:, 0] >= 0.0) & (finite[:, 0] <= 18.0)
        & (np.abs(finite[:, 1]) <= 7.0)
    ]
    if len(projected) > 1200:
        projected = projected[::int(math.ceil(len(projected) / 1200))]
    nearest = (
        float(np.min(np.hypot(obstacle_points[:, 0], obstacle_points[:, 1])))
        if len(obstacle_points) else None
    )
    return {
        'finite': int(len(finite)), 'samples': int(len(xyz)),
        'nearest': nearest, 'obstacle_samples': int(len(obstacle_points)),
        'x_range': [float(ranges[0][0]), float(ranges[1][0])],
        'y_range': [float(ranges[0][1]), float(ranges[1][1])],
        'z_range': [float(ranges[0][2]), float(ranges[1][2])],
        'frame': msg.header.frame_id,
        'points': projected[:, :3].astype(float).tolist(),
    }


STATE = {
    'camera_jpeg': None, 'camera_last': None, 'camera_count': 0,
    'camera_width': 0, 'camera_height': 0, 'camera_times': deque(maxlen=120),
    'trav_jpeg': None, 'trav_last': None, 'trav_width': 0, 'trav_height': 0,
    'trav_count': 0, 'mean_score': None, 'trav_class': None,
    'class_confidence': None, 'geometry_confidence': None,
    'slope_mean': None, 'roughness_mean': None,
    'odom_last': None, 'odom': {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'vx': 0.0, 'wz': 0.0},
    'localization_last': None, 'localization_source': None, 'localization_pose': None,
    'imu_last': None, 'imu': {'ax': 0.0, 'ay': 0.0, 'az': 0.0, 'gz': 0.0},
    'cmd_last': None, 'cmd': {'vx': 0.0, 'wz': 0.0},
    'local_cmd_last': None, 'local_cmd': {'vx': 0.0, 'wz': 0.0},
    'cloud_last': None, 'cloud_stats': None, 'clock_sec': None,
    'map_last': None, 'map': None, 'mapping_state': 'INITIALIZING',
    'exploration_state': 'MAPPING', 'frontier_count': 0, 'explored_percent': 0.0,
    'frontier_distance': None, 'goal': None, 'global_path': [], 'local_path': [],
    'frontiers': [], 'mission_state': 'INITIALIZING', 'home': None,
    'known_area_m2': None, 'home_logged': False,
    'mission_reason': '', 'mission_metrics': None, 'mission_result_path': None,
    'global_path_status': 'WAITING', 'planner_state': 'WAITING',
    'recommended_speed': None, 'local_state': 'WAITING',
    'steering_deg': None, 'clearance': None,
    'obstacle_score': None, 'safety_state': 'UNKNOWN', 'safety_reason': 'WAITING',
    # ── New: degraded autonomy, obstacle tracking, planner risk, mission events ──
    'autonomy_mode': 'UNKNOWN',
    'autonomy_reason': '',
    'autonomy_is_degraded': False,
    'autonomy_capability': 1.0,
    'autonomy_degradations': [],
    'autonomy_last': None,
    'dynamic_obs_count': 0,
    'dynamic_obs_list': [],
    'dynamic_risk': 0.0,
    'dynamic_risk_last': None,
    'planner_risk': None,
    'planner_risk_last': None,
    'mission_events': deque(maxlen=50),
    'autonomy_summary': '',
    'localization_confidence': None,
}

# ---------------------------------------------------------------------------
# HTML — futuristic professional dark-theme UGV mission-control interface
# ---------------------------------------------------------------------------
HTML = r'''<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DRISTHI · Autonomous UGV Navigation</title>
<style>
/* ── design tokens ───────────────────────────────────────────────────────── */
:root{
  --bg:#080d12;--surface:#0d1520;--surface2:#101c27;--border:#1a2f42;
  --border2:#243d52;--muted:#6b8499;--text:#d6e8f5;--dim:#9ab5c8;
  --cyan:#29d4eb;--green:#35e07a;--amber:#f5c842;--red:#f05c5c;
  --violet:#9d7fff;--pink:#ff6eb4;--blue:#4394f5;
  --font:'Inter','Segoe UI',Arial,sans-serif;
  --mono:'JetBrains Mono','Fira Code','Consolas',monospace;
  color-scheme:dark;
}
/* ── reset ───────────────────────────────────────────────────────────────── */
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font:13px/1.5 var(--font);min-height:100vh}
/* ── header bar ──────────────────────────────────────────────────────────── */
#topbar{
  position:sticky;top:0;z-index:100;
  background:rgba(8,13,18,.97);
  border-bottom:1px solid var(--border);
  padding:0 20px;height:54px;
  display:flex;align-items:center;gap:16px;
}
#topbar .logo{
  display:flex;align-items:center;gap:10px;
  font:700 15px var(--mono);letter-spacing:.12em;color:var(--cyan);
  flex-shrink:0;
}
#topbar .logo svg{flex-shrink:0}
.divider{width:1px;height:28px;background:var(--border);flex-shrink:0}
/* large mission badge */
#mission-badge{
  display:flex;align-items:center;gap:8px;padding:5px 14px;
  border-radius:6px;border:1px solid var(--border2);
  background:var(--surface);font:700 13px var(--mono);letter-spacing:.08em;
  flex-shrink:0;
}
/* safety, autonomy, risk badges */
#safety-badge{
  display:flex;align-items:center;gap:8px;padding:5px 14px;
  border-radius:6px;border:1px solid var(--border2);
  background:var(--surface);font:700 13px var(--mono);letter-spacing:.08em;
  flex-shrink:0;
}
#autonomy-badge{
  display:flex;align-items:center;gap:8px;padding:5px 14px;
  border-radius:6px;border:1px solid var(--border2);
  background:var(--surface);font:700 12px var(--mono);letter-spacing:.08em;
  flex-shrink:0;cursor:default;
}
#risk-badge{
  display:flex;align-items:center;gap:8px;padding:5px 14px;
  border-radius:6px;border:1px solid var(--border2);
  background:var(--surface);font:700 12px var(--mono);letter-spacing:.08em;
  flex-shrink:0;cursor:default;
}
.risk-low   {color:var(--green);border-color:#35e07a44}
.risk-mod   {color:var(--amber);border-color:#f5c84244}
.risk-high  {color:var(--red);border-color:#f05c5c44}
.risk-crit  {color:#fff;background:#8b1a1a;border-color:var(--red)}
.badge-dot{width:9px;height:9px;border-radius:50%;background:var(--muted);flex-shrink:0}
.hdr-meta{display:flex;gap:10px;align-items:center;font:12px var(--mono);color:var(--dim);flex-wrap:wrap}
.hdr-meta span{display:flex;align-items:center;gap:5px}
.hdr-meta b{color:var(--text)}
#system-pill{
  margin-left:auto;padding:5px 14px;border-radius:6px;
  font:700 13px var(--mono);letter-spacing:.1em;
  border:1px solid var(--border2);background:var(--surface);
  flex-shrink:0;
}
/* ── main grid ───────────────────────────────────────────────────────────── */
main{
  display:grid;
  grid-template-columns:1fr 360px;
  grid-template-rows:auto;
  gap:10px;padding:10px;
  max-width:1800px;margin:0 auto;
}
/* ── panel ───────────────────────────────────────────────────────────────── */
.panel{
  background:var(--surface);border:1px solid var(--border);border-radius:8px;
  overflow:hidden;display:flex;flex-direction:column;min-width:0;
}
.panel-hdr{
  padding:9px 14px;border-bottom:1px solid var(--border);
  display:flex;align-items:center;gap:8px;flex-shrink:0;
}
.panel-hdr h2{
  font:600 11px var(--mono);letter-spacing:.1em;text-transform:uppercase;
  color:var(--dim);flex:1;
}
.panel-body{padding:12px 14px;flex:1;overflow:hidden}
/* ── left column ─────────────────────────────────────────────────────────── */
#left-col{
  display:flex;flex-direction:column;gap:10px;min-width:0;
}
/* ── right column ────────────────────────────────────────────────────────── */
#right-col{
  width:360px;display:flex;flex-direction:column;gap:10px;
}
/* ── map canvas ──────────────────────────────────────────────────────────── */
#map-wrap{
  height:clamp(340px,52vh,660px);
  background:#060b10;
  display:block;position:relative;
}
#map{width:100%;height:100%;display:block}
#map-legend{
  display:flex;flex-wrap:wrap;gap:12px;padding:8px 14px;
  border-top:1px solid var(--border);font:11px var(--mono);color:var(--muted);
}
.swatch{
  display:inline-block;width:9px;height:9px;border-radius:2px;
  vertical-align:middle;margin-right:4px;
}
/* ── map stats row ───────────────────────────────────────────────────────── */
#map-stats{
  display:flex;flex-wrap:wrap;gap:0;border-top:1px solid var(--border);
}
.map-stat{
  flex:1;min-width:120px;padding:8px 14px;border-right:1px solid var(--border);
  font:11px var(--mono);
}
.map-stat:last-child{border-right:none}
.map-stat .k{color:var(--muted);text-transform:uppercase;font-size:10px;letter-spacing:.08em}
.map-stat .v{color:var(--text);font-weight:700;margin-top:2px;font-size:12px}
/* ── bottom row ──────────────────────────────────────────────────────────── */
#bottom-row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}
/* ── cameras strip ───────────────────────────────────────────────────────── */
#cam-strip{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.cam-img{
  width:100%;aspect-ratio:4/3;object-fit:contain;
  background:#050a0f;border-radius:4px;display:block;
}
.cam-meta{font:11px var(--mono);color:var(--muted);margin-top:6px}
/* ── cloud radar canvas ──────────────────────────────────────────────────── */
#cloud-wrap{height:220px;background:#060b10;border-radius:4px;overflow:hidden}
#pointcloud{width:100%;height:100%;display:block}
/* ── data rows ───────────────────────────────────────────────────────────── */
.drow{
  display:flex;justify-content:space-between;align-items:baseline;
  padding:6px 0;border-bottom:1px solid var(--border);font:12px var(--mono);
  gap:8px;
}
.drow:last-child{border-bottom:none}
.drow .k{color:var(--muted);font-size:11px;flex-shrink:0}
.drow .v{color:var(--text);font-weight:600;text-align:right;word-break:break-all}
/* ── metrics grid ────────────────────────────────────────────────────────── */
.metrics{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.metric{
  background:var(--surface2);border:1px solid var(--border);border-radius:6px;
  padding:9px 10px;
}
.metric .k{font:10px var(--mono);color:var(--muted);text-transform:uppercase;letter-spacing:.08em}
.metric .v{font:700 18px var(--mono);color:var(--text);margin-top:4px;line-height:1}
.metric .u{font:11px var(--mono);color:var(--muted)}
/* ── health grid ─────────────────────────────────────────────────────────── */
#health-grid{
  display:grid;grid-template-columns:repeat(2,1fr);gap:6px;
}
.health-chip{
  display:flex;justify-content:space-between;align-items:center;
  background:var(--surface2);border:1px solid var(--border);border-radius:5px;
  padding:6px 9px;font:11px var(--mono);
}
.health-chip .name{color:var(--dim);text-transform:uppercase;font-size:10px;letter-spacing:.06em}
.chip-badge{
  font:700 10px var(--mono);padding:2px 7px;border-radius:3px;letter-spacing:.06em;
}
/* ── status indicator colours ────────────────────────────────────────────── */
.live  {color:var(--green)}
.degraded{color:var(--amber)}
.stale {color:var(--amber)}
.offline{color:var(--red)}
.bg-live   {background:rgba(53,224,122,.12);color:var(--green);border:1px solid rgba(53,224,122,.25)}
.bg-degraded{background:rgba(245,200,66,.10);color:var(--amber);border:1px solid rgba(245,200,66,.22)}
.bg-stale  {background:rgba(245,200,66,.10);color:var(--amber);border:1px solid rgba(245,200,66,.22)}
.bg-offline{background:rgba(240,92,92,.11);color:var(--red);border:1px solid rgba(240,92,92,.23)}
/* mission state colours */
.ms-online {color:var(--green)}
.ms-degraded{color:var(--amber)}
.ms-offline{color:var(--red)}
.ms-unknown{color:var(--muted)}
/* safety state */
.safe-ok  {background:rgba(53,224,122,.13);border-color:rgba(53,224,122,.3)!important;color:var(--green)}
.safe-stop{background:rgba(240,92,92,.13);border-color:rgba(240,92,92,.30)!important;color:var(--red)}
.safe-unk {background:rgba(107,132,153,.10);border-color:rgba(107,132,153,.25)!important;color:var(--muted)}
/* ── safety banner ───────────────────────────────────────────────────────── */
#safety-banner{
  border-radius:6px;border:1px solid var(--border2);
  padding:10px 14px;font:700 14px var(--mono);letter-spacing:.08em;
  display:flex;align-items:center;gap:10px;
}
/* ── exploration state tag ───────────────────────────────────────────────── */
.tag{
  font:700 10px var(--mono);padding:3px 8px;border-radius:4px;letter-spacing:.07em;
  background:rgba(41,212,235,.1);color:var(--cyan);border:1px solid rgba(41,212,235,.2);
}
/* ── event timeline ──────────────────────────────────────────────────────── */
#timeline-wrap{overflow-y:auto;max-height:360px;flex:1}
#timeline-wrap::-webkit-scrollbar{width:5px}
#timeline-wrap::-webkit-scrollbar-track{background:transparent}
#timeline-wrap::-webkit-scrollbar-thumb{background:var(--border2);border-radius:3px}
table.timeline{width:100%;border-collapse:collapse;font:11px var(--mono)}
table.timeline th{
  padding:6px 8px;text-align:left;color:var(--muted);font-size:10px;
  text-transform:uppercase;letter-spacing:.08em;
  border-bottom:1px solid var(--border);position:sticky;top:0;
  background:var(--surface);z-index:1;
}
table.timeline td{
  padding:6px 8px;border-bottom:1px solid rgba(26,47,66,.6);
  vertical-align:top;
}
table.timeline tr:last-child td{border-bottom:none}
table.timeline .ts{color:var(--cyan);white-space:nowrap}
table.timeline .ev{color:var(--text)}
table.timeline .rs{color:var(--dim);font-size:10px}
table.timeline .vl{color:var(--amber);font-size:10px;white-space:nowrap}
/* ── topic table ─────────────────────────────────────────────────────────── */
#topic-wrap{overflow-x:auto;flex:1}
table.topics{width:100%;border-collapse:collapse;font:11px var(--mono)}
table.topics th{
  padding:6px 8px;text-align:left;color:var(--muted);font-size:10px;
  text-transform:uppercase;letter-spacing:.08em;
  border-bottom:1px solid var(--border);
}
table.topics td{padding:6px 8px;border-bottom:1px solid rgba(26,47,66,.5)}
table.topics tr:last-child td{border-bottom:none}
/* ── responsive ──────────────────────────────────────────────────────────── */
@media(max-width:1100px){
  main{grid-template-columns:1fr}
  #right-col{width:100%}
  #bottom-row{grid-template-columns:1fr 1fr}
}
@media(max-width:720px){
  #bottom-row{grid-template-columns:1fr}
  #cam-strip{grid-template-columns:1fr}
  #health-grid{grid-template-columns:repeat(2,1fr)}
  .hdr-meta{display:none}
}
</style>
</head>
<body>

<!-- ══ TOP BAR ═══════════════════════════════════════════════════════════ -->
<div id="topbar">
  <div class="logo">
    <svg width="26" height="26" viewBox="0 0 26 26" fill="none">
      <rect x="1" y="1" width="24" height="24" rx="5" stroke="#29d4eb" stroke-width="1.5"/>
      <polygon points="13,4 22,20 4,20" fill="none" stroke="#29d4eb" stroke-width="1.5" stroke-linejoin="round"/>
      <circle cx="13" cy="13" r="2.5" fill="#29d4eb"/>
    </svg>
    DRISTHI
  </div>
  <div class="divider"></div>
  <div id="mission-badge">
    <div class="badge-dot" id="mission-dot"></div>
    <span id="mission-text">INITIALIZING</span>
  </div>
  <div class="divider"></div>
  <div id="safety-badge">
    <div class="badge-dot" id="safety-dot"></div>
    SAFETY&nbsp;<span id="safety-text">UNKNOWN</span>
  </div>
  <div class="divider"></div>
  <div id="autonomy-badge" class="safe-unk">
    <span id="autonomy-mode-text">MODE: UNKNOWN</span>
  </div>
  <div class="divider"></div>
  <div id="risk-badge" class="safe-unk">
    RISK&nbsp;<span id="risk-level-text">—</span>
  </div>
  <div class="divider"></div>
  <div class="hdr-meta">
    <span>DOMAIN&nbsp;<b id="domain">—</b></span>
    <span>SIM&nbsp;<b id="simtime">—</b></span>
    <span>UPTIME&nbsp;<b id="runtime">—</b></span>
    <span>MAP&nbsp;<b id="mapstate-hdr">INITIALIZING</b></span>
    <span>EXPL&nbsp;<b id="explore-hdr">—</b></span>
  </div>
  <div id="system-pill" class="ms-unknown">OFFLINE</div>
</div>

<!-- ══ MAIN LAYOUT ════════════════════════════════════════════════════════ -->
<main>

  <!-- LEFT COLUMN -->
  <div id="left-col">

    <!-- OCCUPANCY MAP -->
    <div class="panel">
      <div class="panel-hdr">
        <h2>Live Occupancy Map</h2>
        <span class="tag" id="map-expl-tag">EXPLORATION: —</span>
        <span class="tag" style="margin-left:4px" id="map-path-tag">PATH: WAITING</span>
      </div>
      <div id="map-wrap"><canvas id="map"></canvas></div>
      <div id="map-legend">
        <span><i class="swatch" style="background:#17232d"></i>Unknown</span>
        <span><i class="swatch" style="background:#d4e8d0"></i>Free</span>
        <span><i class="swatch" style="background:#e85248"></i>Occupied</span>
        <span><i class="swatch" style="background:#29d4eb"></i>Global path</span>
        <span><i class="swatch" style="background:#ffd166"></i>Local path</span>
        <span><i class="swatch" style="background:#3ee8c1"></i>Frontier</span>
        <span><i class="swatch" style="background:#ff6eb4"></i>Home</span>
        <span>▲ Robot heading</span>
      </div>
      <div id="map-stats">
        <div class="map-stat"><div class="k">Grid</div><div class="v" id="mapdims">—</div></div>
        <div class="map-stat"><div class="k">Known</div><div class="v" id="known-cells">—</div></div>
        <div class="map-stat"><div class="k">Free / Occupied</div><div class="v" id="free-occ">—</div></div>
        <div class="map-stat"><div class="k">Coverage</div><div class="v" id="knownpct">—</div></div>
        <div class="map-stat"><div class="k">Known area</div><div class="v" id="knownarea">—</div></div>
      </div>
    </div>

    <!-- BOTTOM ROW: cameras | radar | terrain -->
    <div id="bottom-row">

      <!-- CAMERAS -->
      <div class="panel">
        <div class="panel-hdr"><h2>RGB Sensor</h2></div>
        <div class="panel-body">
          <img class="cam-img" src="/camera.mjpg" alt="Live RGB">
          <div class="cam-meta">
            <span id="cam-res">—</span> &nbsp;·&nbsp;
            <span id="cam-fps">— Hz</span> &nbsp;·&nbsp;
            <span id="cam-age">—</span>
          </div>
        </div>
      </div>

      <!-- TRAVERSABILITY -->
      <div class="panel">
        <div class="panel-hdr"><h2>Traversability</h2></div>
        <div class="panel-body">
          <img class="cam-img" src="/trav.mjpg" alt="Traversability">
          <div class="drow"><span class="k">Mean score</span><span class="v" id="travscore">—</span></div>
          <div class="drow"><span class="k">Class / confidence</span><span class="v" id="terrain-class">—</span></div>
          <div class="drow"><span class="k">Slope / roughness / geom conf</span><span class="v" id="geometry">—</span></div>
        </div>
      </div>

      <!-- DEPTH RADAR -->
      <div class="panel">
        <div class="panel-hdr"><h2>Depth Radar · Forward Projection</h2></div>
        <div class="panel-body">
          <div id="cloud-wrap"><canvas id="pointcloud"></canvas></div>
          <div class="drow"><span class="k">Nearest obstacle</span><span class="v" id="nearest">—</span></div>
          <div class="drow"><span class="k">Obstacle / valid samples</span><span class="v" id="depthcount">—</span></div>
          <div class="drow"><span class="k">Rate / age</span><span class="v" id="cloudrate">—</span></div>
          <div class="drow"><span class="k">XYZ range</span><span class="v" id="xyzrange">—</span></div>
          <div style="font:10px var(--mono);color:var(--muted);margin-top:4px" id="cloudframe">Frame —</div>
        </div>
      </div>

    </div><!-- /bottom-row -->

    <!-- TELEMETRY ROW -->
    <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px">

      <!-- LOCALIZATION + POSE -->
      <div class="panel">
        <div class="panel-hdr"><h2>Localization · Vehicle</h2></div>
        <div class="panel-body">
          <div class="metrics" style="margin-bottom:8px">
            <div class="metric"><div class="k">X</div><div class="v" id="pose-x">—</div><div class="u">m</div></div>
            <div class="metric"><div class="k">Y</div><div class="v" id="pose-y">—</div><div class="u">m</div></div>
            <div class="metric"><div class="k">Yaw</div><div class="v" id="pose-yaw">—</div><div class="u">rad</div></div>
            <div class="metric"><div class="k">Speed</div><div class="v" id="pose-vx">—</div><div class="u">m/s</div></div>
          </div>
          <div class="drow"><span class="k">Angular vel</span><span class="v" id="pose-wz">—</span></div>
          <div class="drow"><span class="k">Pose age</span><span class="v" id="pose-age">—</span></div>
          <div class="drow"><span class="k">Source</span><span class="v" id="locsource">Gazebo odometry</span></div>
          <div class="drow"><span class="k">IMU accel (x/y/z)</span><span class="v" id="imu-accel">—</span></div>
          <div class="drow"><span class="k">IMU gyro z</span><span class="v" id="imu-gz">—</span></div>
        </div>
      </div>

      <!-- PLANNER -->
      <div class="panel">
        <div class="panel-hdr"><h2>Planner Telemetry</h2></div>
        <div class="panel-body">
          <div class="metrics" style="margin-bottom:8px">
            <div class="metric"><div class="k">Steering</div><div class="v" id="steering">—</div><div class="u">°</div></div>
            <div class="metric"><div class="k">Frontier dist</div><div class="v" id="frontierdist2">—</div><div class="u">m</div></div>
          </div>
          <div class="drow"><span class="k">Global recommended speed</span><span class="v" id="recommendedspeed">—</span></div>
          <div class="drow"><span class="k">Global planner</span><span class="v" id="plannerstate">WAITING</span></div>
          <div class="drow"><span class="k">Global path status</span><span class="v" id="pathstatus">WAITING</span></div>
          <div class="drow"><span class="k">Local planner</span><span class="v" id="localstate">WAITING</span></div>
          <div class="drow"><span class="k">Local command vx/wz</span><span class="v" id="localcommand">—</span></div>
          <div class="drow"><span class="k">Obstacle clearance</span><span class="v" id="clearance">—</span></div>
          <div class="drow"><span class="k">Obstacle score</span><span class="v" id="obscore">—</span></div>
          <div class="drow"><span class="k">Frontier distance</span><span class="v" id="frontierdist">—</span></div>
        </div>
      </div>

      <!-- SAFETY + COMMAND -->
      <div class="panel">
        <div class="panel-hdr"><h2>Safety Supervisor</h2></div>
        <div class="panel-body">
          <div id="safety-banner" class="safe-unk" style="margin-bottom:10px">
            <span style="font-size:18px" id="safety-icon">⬤</span>
            <span id="safety-banner-text">SAFETY: UNKNOWN</span>
          </div>
          <div class="drow"><span class="k">Safety reason</span><span class="v" id="safetyreason">WAITING</span></div>
          <div class="drow"><span class="k">Auth cmd  vx / wz</span><span class="v" id="command">—</span></div>
          <div class="drow"><span class="k">Command age</span><span class="v" id="commandage">—</span></div>
          <div class="drow"><span class="k">Localization source</span><span class="v" id="locsource2">—</span></div>
          <div class="drow"><span class="k">Mapping mode</span><span class="v">ODOMETRY-ASSISTED</span></div>
          <div class="drow"><span class="k">Exploration state</span><span class="v" id="expl-state-bottom">—</span></div>
        </div>
      </div>

    </div><!-- /telemetry row -->

    <!-- TOPIC TABLE (collapsible) -->
    <details class="panel" style="cursor:pointer">
      <summary style="padding:9px 14px;border-bottom:1px solid var(--border);display:flex;align-items:center;gap:8px;list-style:none;user-select:none">
        <h2 style="font:600 11px var(--mono);letter-spacing:.1em;text-transform:uppercase;color:var(--dim);flex:1">Technical Details — ROS Topic Status</h2>
        <span style="font:11px var(--mono);color:var(--muted)">▼ expand</span>
      </summary>
      <div class="panel-body" style="padding:0">
        <div id="topic-wrap">
          <table class="topics">
            <thead><tr><th>Topic</th><th>Type</th><th>Rate</th><th>Age</th><th>Status</th></tr></thead>
            <tbody id="topics"></tbody>
          </table>
        </div>
      </div>
    </details>

  </div><!-- /left-col -->

  <!-- RIGHT COLUMN -->
  <div id="right-col">

    <!-- DRISTHI STATUS: quick-read panel for non-ROS users -->
    <div class="panel">
      <div class="panel-hdr"><h2>DRISTHI Status</h2></div>
      <div class="panel-body">
        <div id="stop-banner" style="display:none;background:#2a0a0a;border:1px solid var(--red);border-radius:6px;padding:10px 14px;margin-bottom:10px">
          <div style="font:700 13px var(--mono);color:var(--red);margin-bottom:4px">⛔ WHY DID THE ROBOT STOP?</div>
          <div id="stop-reason-text" style="color:var(--text)">—</div>
        </div>
        <div class="drow"><span class="k">Autonomy mode</span><span class="v" id="autonomy-mode-display">UNKNOWN</span></div>
        <div class="drow"><span class="k">Mode reason</span><span class="v" id="autonomy-reason-display">—</span></div>
        <div class="drow"><span class="k">Capability score</span><span class="v" id="autonomy-capability-display">—</span></div>
        <div class="drow"><span class="k">Localization confidence</span><span class="v" id="loc-confidence-display">—</span></div>
        <div class="drow"><span class="k">Safety state</span><span class="v" id="safety-state-display">—</span></div>
        <div class="drow"><span class="k">Safety reason</span><span class="v" id="safety-reason-display">—</span></div>
        <div class="drow"><span class="k">Autonomy summary</span><span class="v" id="autonomy-summary-display" style="font-size:10px;color:var(--dim)">—</span></div>
      </div>
    </div>

    <!-- ACTIVE RISK PANEL -->
    <div class="panel">
      <div class="panel-hdr"><h2>Active Risk Assessment</h2></div>
      <div class="panel-body">
        <div id="risk-level-banner" class="safe-unk" style="text-align:center;padding:8px;margin-bottom:10px;border-radius:6px;font:700 16px var(--mono)">
          RISK: —
        </div>
        <div class="drow"><span class="k">Total risk</span><span class="v" id="risk-total">—</span></div>
        <div class="drow"><span class="k">Terrain risk</span><span class="v" id="risk-terrain">—</span></div>
        <div class="drow"><span class="k">Static obstacle risk</span><span class="v" id="risk-obstacle">—</span></div>
        <div class="drow"><span class="k">Dynamic obstacle risk</span><span class="v" id="risk-dynamic">—</span></div>
        <div class="drow"><span class="k">Localization risk</span><span class="v" id="risk-localization">—</span></div>
        <div class="drow"><span class="k">Sensor degradation</span><span class="v" id="risk-sensor">—</span></div>
        <div class="drow"><span class="k">Path valid</span><span class="v" id="risk-path-valid">—</span></div>
        <div class="drow"><span class="k">Selected path reason</span><span class="v" id="risk-path-reason" style="font-size:10px;color:var(--dim)">—</span></div>
      </div>
    </div>

    <!-- DYNAMIC OBSTACLES -->
    <div class="panel">
      <div class="panel-hdr"><h2>Dynamic Obstacles</h2></div>
      <div class="panel-body">
        <div class="metrics" style="margin-bottom:8px">
          <div class="metric"><div class="k">Dynamic</div><div class="v" id="dyn-count">0</div></div>
          <div class="metric"><div class="k">Total tracked</div><div class="v" id="obs-total">0</div></div>
          <div class="metric"><div class="k">Risk score</div><div class="v" id="obs-risk-score">0.00</div></div>
        </div>
        <div id="obs-list" style="max-height:200px;overflow-y:auto;font:11px var(--mono)">
          <div style="color:var(--muted)">No obstacles tracked.</div>
        </div>
      </div>
    </div>

    <!-- MISSION EVENTS TIMELINE (from MissionEvent messages) -->
    <div class="panel" style="flex:1;min-height:0">
      <div class="panel-hdr"><h2>Mission Intelligence Events</h2></div>
      <div id="mission-events-wrap" style="overflow-y:auto;max-height:220px">
        <table class="timeline">
          <thead><tr>
            <th>Time</th><th>Severity</th><th>System</th><th>Description</th>
          </tr></thead>
          <tbody id="mission-events-body"></tbody>
        </table>
      </div>
    </div>

    <!-- EXPLORATION + MISSION -->
    <div class="panel">
      <div class="panel-hdr"><h2>Mission Intelligence</h2></div>
      <div class="panel-body">
        <div class="metrics" style="margin-bottom:8px">
          <div class="metric"><div class="k">Mission state</div><div class="v" id="mission">INIT</div></div>
          <div class="metric"><div class="k">Exploration</div><div class="v" id="expl-mode">MAPPING</div></div>
        </div>
        <div class="drow"><span class="k">Coverage</span><span class="v" id="explored">0%</span></div>
        <div class="drow"><span class="k">Frontiers / distance</span><span class="v" id="frontiercount">0 / —</span></div>
        <div class="drow"><span class="k">Active goal</span><span class="v" id="target">—</span></div>
        <div class="drow"><span class="k">Home pose</span><span class="v" id="homepose">—</span></div>
        <div class="drow"><span class="k">Known area</span><span class="v" id="knownarea2">—</span></div>
        <div class="drow"><span class="k">Mission metrics</span><span class="v" id="missionmetrics">—</span></div>
        <div class="drow"><span class="k">Result artifact</span><span class="v" id="resultpath">pending</span></div>
      </div>
    </div>

    <!-- MISSION MEASUREMENTS -->
    <div class="panel">
      <div class="panel-hdr"><h2>Mission Measurements</h2></div>
      <div class="panel-body">
        <div class="drow"><span class="k">Elapsed / traveled</span><span class="v" id="mm-elapsed">—</span></div>
        <div class="drow"><span class="k">Home position error</span><span class="v" id="mm-home-err">—</span></div>
        <div class="drow"><span class="k">Replans / safety stops</span><span class="v" id="mm-replans">—</span></div>
        <div class="drow"><span class="k">Blocked goals</span><span class="v" id="mm-blocked">—</span></div>
        <div class="drow"><span class="k">Min clearance</span><span class="v" id="mm-clearance">—</span></div>
        <div class="drow"><span class="k">Obstacle observations</span><span class="v" id="mm-obstacles">—</span></div>
        <div class="drow"><span class="k">Collision detected</span><span class="v" id="mm-collision">—</span></div>
        <div class="drow"><span class="k">Map cells (K/F/O/U)</span><span class="v" id="mm-cells">—</span></div>
      </div>
    </div>

    <!-- SUBSYSTEM HEALTH -->
    <div class="panel">
      <div class="panel-hdr"><h2>Subsystem Health</h2></div>
      <div class="panel-body"><div id="health-grid"></div></div>
    </div>

    <!-- DECISION TIMELINE -->
    <div class="panel" style="flex:1;min-height:0">
      <div class="panel-hdr"><h2>Decision Timeline · Mission Intelligence</h2></div>
      <div id="timeline-wrap">
        <table class="timeline">
          <thead>
            <tr>
              <th>Time</th>
              <th>Event</th>
              <th>Reason</th>
              <th>Value</th>
            </tr>
          </thead>
          <tbody id="timeline-body"></tbody>
        </table>
      </div>
    </div>

  </div><!-- /right-col -->
</main>

<script>
/* ── helpers ─────────────────────────────────────────────────────────────── */
'use strict';
let trail=[];

function fmt(v,n=2){
  return (v===null||v===undefined||!Number.isFinite(v))?'—':Number(v).toFixed(n);
}
function age(v){
  return (v===null||v===undefined)?'—':fmt(v,1)+' s';
}
function el(id){return document.getElementById(id)}
function healthClass(s){
  if(s==='LIVE')return 'bg-live';
  if(s==='DEGRADED')return 'bg-degraded';
  if(s==='STALE')return 'bg-stale';
  return 'bg-offline';
}

/* ── map canvas ──────────────────────────────────────────────────────────── */
function drawMap(s){
  const c=el('map'),ctx=c.getContext('2d'),m=s.map;
  const W=c.clientWidth*devicePixelRatio,H=c.clientHeight*devicePixelRatio;
  if(c.width!==W||c.height!==H){c.width=W;c.height=H}
  ctx.fillStyle='#06090d';ctx.fillRect(0,0,W,H);
  if(!m||!m.data||!m.width||!m.height)return;
  const mW=m.width,mH=m.height;
  const raw=atob(m.data),cells=new Int8Array(raw.length);
  for(let i=0;i<raw.length;i++)cells[i]=raw.charCodeAt(i)>127?raw.charCodeAt(i)-256:raw.charCodeAt(i);
  const off=document.createElement('canvas');off.width=mW;off.height=mH;
  const ox=off.getContext('2d'),im=ox.createImageData(mW,mH);
  for(let r=0;r<mH;r++)for(let x=0;x<mW;x++){
    let v=cells[r*mW+x],j=((mH-1-r)*mW+x)*4;
    if(v<0){im.data[j]=23;im.data[j+1]=35;im.data[j+2]=45}
    else if(v>=50){im.data[j]=232;im.data[j+1]=88;im.data[j+2]=82}
    else{im.data[j]=212;im.data[j+1]=232;im.data[j+2]=208}
    im.data[j+3]=255;
  }
  ox.putImageData(im,0,0);
  ctx.imageSmoothingEnabled=false;
  ctx.drawImage(off,0,0,W,H);

  const sx=W/mW,sy=H/mH;
  const pix=p=>[(p.x-m.origin_x)/m.resolution*sx, H-(p.y-m.origin_y)/m.resolution*sy];

  function drawPath(pts,color,lw){
    if(!pts||pts.length<2)return;
    ctx.strokeStyle=color;ctx.lineWidth=lw*devicePixelRatio;
    ctx.lineCap='round';ctx.lineJoin='round';
    ctx.beginPath();
    pts.forEach((p,i)=>{let q=pix(p);i?ctx.lineTo(q[0],q[1]):ctx.moveTo(q[0],q[1])});
    ctx.stroke();
  }

  // trail
  if(s.pose){
    trail.push({x:s.pose.x,y:s.pose.y});
    if(trail.length>2000)trail.shift();
    drawPath(trail,'rgba(120,140,255,.55)',1);
  }

  // global path
  drawPath(m.global_path,'#29d4eb',1.8);
  // local path
  drawPath(m.local_path,'#ffd166',2.5);

  // frontiers
  ctx.fillStyle='#3ee8c1';
  for(const p of m.frontiers||[]){let q=pix(p);ctx.fillRect(q[0]-2,q[1]-2,4,4)}

  // active goal circle
  if(m.goal){
    let q=pix(m.goal);
    ctx.strokeStyle='#3ee8c1';ctx.lineWidth=1.5*devicePixelRatio;
    ctx.beginPath();ctx.arc(q[0],q[1],6*devicePixelRatio,0,2*Math.PI);ctx.stroke();
    ctx.fillStyle='rgba(62,232,193,.2)';ctx.fill();
  }

  // home marker
  if(m.home){
    let h=pix(m.home);
    ctx.strokeStyle='#ff6eb4';ctx.lineWidth=1.5*devicePixelRatio;
    ctx.beginPath();ctx.arc(h[0],h[1],7*devicePixelRatio,0,2*Math.PI);ctx.stroke();
    ctx.fillStyle='#ff6eb4';ctx.font=`${10*devicePixelRatio}px monospace`;
    ctx.fillText('HOME',h[0]+9*devicePixelRatio,h[1]-9*devicePixelRatio);
  }

  // robot heading triangle
  if(s.pose){
    let q=pix(s.pose),sc=10*devicePixelRatio;
    ctx.save();ctx.translate(q[0],q[1]);ctx.rotate(-s.pose.yaw);
    ctx.fillStyle='#ffffff';ctx.strokeStyle='#29d4eb';ctx.lineWidth=devicePixelRatio;
    ctx.beginPath();
    ctx.moveTo(sc,0);ctx.lineTo(-sc*.65,-sc*.55);ctx.lineTo(-sc*.65,sc*.55);
    ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
  }
}

/* ── depth radar ─────────────────────────────────────────────────────────── */
function drawCloud(s){
  const c=el('pointcloud'),ctx=c.getContext('2d');
  const W=c.clientWidth*devicePixelRatio,H=c.clientHeight*devicePixelRatio;
  if(c.width!==W||c.height!==H){c.width=W;c.height=H}
  ctx.fillStyle='#060b10';ctx.fillRect(0,0,W,H);
  const maxX=18,maxY=7,scale=Math.min(W/(2*maxY),H/maxX);
  // grid arcs
  ctx.strokeStyle='#1a2f42';ctx.lineWidth=devicePixelRatio;
  for(const d of[4,8,12,16]){
    ctx.beginPath();
    ctx.arc(W/2,H,d*scale,Math.PI,0);
    ctx.stroke();
    ctx.fillStyle='#3a5060';ctx.font=`${9*devicePixelRatio}px monospace`;
    ctx.fillText(d+'m',W/2+2,H-d*scale);
  }
  // horizontal grid lines
  for(const x of[4,8,12,16]){
    let yy=H-x*scale;
    ctx.beginPath();ctx.moveTo(W/2-maxY*scale,yy);ctx.lineTo(W/2+maxY*scale,yy);ctx.stroke();
  }
  // centre line
  ctx.strokeStyle='#1a3a4a';ctx.beginPath();ctx.moveTo(W/2,0);ctx.lineTo(W/2,H);ctx.stroke();
  // robot icon
  ctx.fillStyle='#4394f5';ctx.font=`${10*devicePixelRatio}px monospace`;
  ctx.fillText('⬡ ROBOT',W/2-22*devicePixelRatio,H-4*devicePixelRatio);

  for(const p of s.depth.points||[]){
    const px=W/2-p[1]*scale,py=H-p[0]*scale;
    if(px<0||px>W||py<0||py>H)continue;
    const elevated=p[2]>0.3;
    ctx.fillStyle=elevated?'rgba(240,92,92,.8)':'rgba(41,212,235,.7)';
    ctx.fillRect(px,py,2*devicePixelRatio,2*devicePixelRatio);
  }
}

/* ── main update loop ────────────────────────────────────────────────────── */
async function update(){
  let s;
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    s=await r.json();
  }catch(e){
    el('system-pill').textContent='API UNAVAILABLE';
    el('system-pill').className='ms-offline';
    return;
  }

  /* ── header ── */
  const sysClass={'ONLINE':'ms-online','DEGRADED':'ms-degraded','OFFLINE':'ms-offline'}[s.system]||'ms-unknown';
  el('system-pill').textContent=s.system;
  el('system-pill').className=sysClass;
  el('domain').textContent=s.domain;
  el('runtime').textContent=fmt(s.runtime,0)+'s';
  el('simtime').textContent=s.sim_time===null?'—':fmt(s.sim_time,1)+'s';
  el('mapstate-hdr').textContent=s.mapping.state;
  el('explore-hdr').textContent=s.exploration.state;

  /* ── mission badge ── */
  const ms=s.mission.state+(s.mission.reason?' · '+s.mission.reason:'');
  el('mission-text').textContent=ms;
  el('mission-badge').className=({'ONLINE':'safe-ok','DEGRADED':'bg-degraded','STOPPED':'bg-degraded'}[s.system]||'safe-unk');
  const mdot=el('mission-dot');
  mdot.style.background=s.system==='ONLINE'?'var(--green)':s.system==='DEGRADED'?'var(--amber)':'var(--muted)';

  /* ── safety badge + banner ── */
  const safe=s.safety.state;
  el('safety-text').textContent=safe;
  const sdot=el('safety-dot');
  const sbanner=el('safety-banner');
  el('safety-banner-text').textContent='SAFETY: '+safe;
  if(safe==='SAFE'){
    sdot.style.background='var(--green)';
    sbanner.className='safe-ok';
  }else if(safe==='UNKNOWN'){
    sdot.style.background='var(--muted)';
    sbanner.className='safe-unk';
  }else{
    sdot.style.background='var(--red)';
    sbanner.className='safe-stop';
  }
  el('safetyreason').textContent=s.safety.reason;
  el('command').textContent=fmt(s.command.vx,3)+' m/s · '+fmt(s.command.wz,3)+' rad/s';
  el('commandage').textContent=age(s.command.age);
  el('locsource2').textContent=s.localization.source||'—';
  el('expl-state-bottom').textContent=s.exploration.state;

  /* ── map tags ── */
  el('map-expl-tag').textContent='EXPLORATION: '+s.exploration.state;
  el('map-path-tag').textContent='PATH: '+s.planner.global_path_status;

  /* ── map stats ── */
  const m=s.map;
  el('mapdims').textContent=m.width+'×'+m.height+' @ '+fmt(m.resolution,2)+'m';
  el('known-cells').textContent=(m.known||0).toLocaleString();
  el('free-occ').textContent=(m.free||0).toLocaleString()+' / '+(m.occupied||0).toLocaleString();
  el('knownpct').textContent=fmt(m.known_percent,1)+'%';
  el('knownarea').textContent=fmt(m.known_area_m2,1)+' m²';

  /* ── camera ── */
  el('cam-res').textContent=s.camera.resolution;
  el('cam-fps').textContent=fmt(s.camera.fps,1)+' Hz';
  el('cam-age').textContent='age '+age(s.camera.age);

  /* ── traversability ── */
  el('travscore').textContent=fmt(s.traversability.mean_score,3);
  el('terrain-class').textContent=(s.terrain.class||'—')+' · conf '+fmt(s.terrain.class_confidence,2);
  el('geometry').textContent=fmt(s.terrain.slope_deg,1)+'° · '+fmt(s.terrain.roughness_m,3)+'m · '+fmt(s.terrain.geometry_confidence,2);

  /* ── depth radar ── */
  el('nearest').textContent=s.depth.nearest===null?'—':fmt(s.depth.nearest,2)+' m';
  el('depthcount').textContent=(s.depth.obstacle_samples||0)+' / '+(s.depth.finite||0)+
    ' ('+fmt((s.depth.valid_ratio||0)*100,1)+'%)';
  el('cloudrate').textContent=fmt(s.depth.rate_hz,1)+' Hz · '+age(s.depth.age);
  el('xyzrange').textContent=s.depth.ranges;
  el('cloudframe').textContent='Frame: '+(s.depth.frame||'—');

  /* ── localization / vehicle ── */
  const pose=s.pose||{};
  el('pose-x').textContent=fmt(pose.x,2);
  el('pose-y').textContent=fmt(pose.y,2);
  el('pose-yaw').textContent=fmt(pose.yaw,3);
  el('pose-vx').textContent=fmt(pose.vx,2);
  el('pose-wz').textContent=fmt(pose.wz,3)+' rad/s';
  el('pose-age').textContent=age(pose.age);
  el('locsource').textContent=s.localization.source||'—';
  const imu=s.imu||{};
  el('imu-accel').textContent=fmt(imu.ax,2)+' / '+fmt(imu.ay,2)+' / '+fmt(imu.az,2)+' m/s²';
  el('imu-gz').textContent=fmt(imu.gz,3)+' rad/s';

  /* ── planner ── */
  el('recommendedspeed').textContent=fmt(s.planner.recommended_speed,3)+' m/s';
  el('frontierdist2').textContent=fmt(s.exploration.distance,2);
  el('steering').textContent=fmt(s.planner.steering_deg,1);
  el('plannerstate').textContent=s.planner.state;
  el('pathstatus').textContent=s.planner.global_path_status;
  el('localstate').textContent=s.planner.local_state;
  const lc=s.planner.local_command||{};
  el('localcommand').textContent=fmt(lc.vx,3)+' m/s · '+fmt(lc.wz,3)+' rad/s · '+age(lc.age);
  el('clearance').textContent=fmt(s.planner.clearance,2)+' m';
  el('obscore').textContent=fmt(s.planner.obstacle_score,3);
  el('frontierdist').textContent=s.exploration.distance===null?'—':fmt(s.exploration.distance,2)+' m';

  /* ── mission intelligence ── */
  el('mission').textContent=s.mission.state;
  el('expl-mode').textContent=s.exploration.state;
  el('explored').textContent=fmt(s.map.known_percent,1)+'%';
  el('frontiercount').textContent=(s.exploration.frontier_count||0)+
    ' / '+(s.exploration.distance===null?'—':fmt(s.exploration.distance,2)+' m');
  el('target').textContent=s.exploration.goal?
    fmt(s.exploration.goal.x,2)+', '+fmt(s.exploration.goal.y,2):'—';
  el('homepose').textContent=s.mission.home?
    fmt(s.mission.home.x,1)+', '+fmt(s.mission.home.y,1):'waiting';
  el('knownarea2').textContent=fmt(s.mission.known_area_m2,1)+' m²';
  const mm=s.mission.metrics;
  el('missionmetrics').textContent=mm?
    mm.mission_result+' · '+fmt(mm.distance_travelled_m,1)+'m · '+fmt(mm.frontiers_completed,0)+' frontiers':'—';
  el('resultpath').textContent=s.mission.result_path||'pending';

  /* ── mission measurements ── */
  if(mm){
    el('mm-elapsed').textContent=fmt(mm.mission_duration_s,1)+'s · '+fmt(mm.distance_travelled_m,2)+'m';
    el('mm-home-err').textContent=fmt(mm.final_position_error_m,2)+' m';
    el('mm-replans').textContent=fmt(mm.number_of_replans,0)+' / '+fmt(mm.number_of_safety_stops,0);
    el('mm-blocked').textContent=fmt(mm.blocked_goals,0);
    el('mm-clearance').textContent=fmt(mm.minimum_obstacle_clearance_m,2)+' m';
    el('mm-obstacles').textContent=fmt(mm.obstacle_observations,0);
    el('mm-collision').textContent=mm.collision_detected===null?'unknown':mm.collision_detected?'YES':'NO';
    el('mm-cells').textContent=[mm.known_cells,mm.free_cells,mm.occupied_cells,mm.unknown_cells].map(v=>v===null?'?':v).join(' / ');
  }

  /* ── subsystem health ── */
  el('health-grid').innerHTML=Object.entries(s.health).map(([k,v])=>
    `<div class="health-chip"><span class="name">${k}</span><span class="chip-badge ${healthClass(v)}">${v}</span></div>`
  ).join('');

  /* ── decision timeline ── */
  if(s.events&&s.events.length){
    el('timeline-body').innerHTML=s.events.map(e=>`<tr>
      <td class="ts">${e.time}</td>
      <td class="ev">${esc(e.message)}</td>
      <td class="rs">${esc(e.reason||'')}</td>
      <td class="vl">${esc(e.value||'')}</td>
    </tr>`).join('');
  }else{
    el('timeline-body').innerHTML='<tr><td colspan="4" style="color:var(--muted);padding:12px 8px">Awaiting events from live ROS stack.</td></tr>';
  }

  /* ── autonomy mode ── */
  const aut=s.autonomy||{};
  const autMode=aut.mode||'UNKNOWN';
  el('autonomy-mode-text').textContent='MODE: '+autMode;
  el('autonomy-mode-display').textContent=autMode;
  el('autonomy-reason-display').textContent=aut.reason||'—';
  el('autonomy-capability-display').textContent=aut.capability_score!==undefined?fmt(aut.capability_score,2):'—';
  el('autonomy-summary-display').textContent=aut.summary||'—';
  {
    const ab=el('autonomy-badge');
    if(autMode==='FULL_AUTONOMY'){ab.className='safe-ok'}
    else if(autMode==='RETURN_HOME'){ab.className='bg-degraded'}
    else if(autMode==='SAFE_HOLD'){ab.className='bg-degraded'}
    else if(autMode==='RECOVERY'){ab.className='bg-degraded'}
    else{ab.className='safe-unk'}
  }

  /* ── localization confidence ── */
  el('loc-confidence-display').textContent=s.localization_confidence!=null?fmt(s.localization_confidence,3):'—';

  /* ── safety status display ── */
  el('safety-state-display').textContent=s.safety.state||'—';
  el('safety-reason-display').textContent=s.safety.reason||'—';
  {
    const stopBanner=el('stop-banner');
    const isStopped=s.safety.state==='STOPPED'||s.safety.state==='E_STOP'||
      (s.safety.reason&&s.safety.reason!=='SAFE_COMMAND'&&s.safety.reason!=='RECOVERY_COMMAND');
    stopBanner.style.display=isStopped?'block':'none';
    if(isStopped){
      const reasonMap={
        'STALE_LIDAR':'LiDAR data is stale — sensor not responding',
        'STALE_LOCALIZATION':'Localization data is stale — pose uncertain',
        'LOW_LOCALIZATION_CONFIDENCE':'Localization confidence too low to move safely',
        'OBSTACLE_STOP':'Obstacle detected in the path',
        'STALE_COMMAND':'Planner command timed out',
        'STALE_NAVIGATION_STATE':'Navigation state timed out',
        'WAITING_FOR_NAVIGATION_STATE':'Waiting for navigation system to start',
        'NAVIGATION_NOT_ACTIVE':'Navigation is not active',
        'STALE_OBSTACLE_SCORE':'Obstacle sensor data is stale',
        'SAFE_HOLD':'System in SAFE HOLD — no safe motion possible',
      };
      el('stop-reason-text').textContent=reasonMap[s.safety.reason]||s.safety.reason||'—';
    }
  }

  /* ── risk assessment ── */
  const pr=s.planner_risk||{};
  const riskLevel=pr.risk_level||'—';
  el('risk-level-text').textContent=riskLevel;
  el('risk-total').textContent=pr.total_cost!=null?fmt(pr.total_cost,3):'—';
  el('risk-terrain').textContent=pr.terrain_cost!=null?fmt(pr.terrain_cost,3):'—';
  el('risk-obstacle').textContent=pr.obstacle_cost!=null?fmt(pr.obstacle_cost,3):'—';
  el('risk-dynamic').textContent=pr.dynamic_risk_cost!=null?fmt(pr.dynamic_risk_cost,3):'—';
  el('risk-localization').textContent=pr.localization_risk!=null?fmt(pr.localization_risk,3):'—';
  el('risk-sensor').textContent=pr.sensor_degradation_cost!=null?fmt(pr.sensor_degradation_cost,3):'—';
  el('risk-path-valid').textContent=pr.path_valid!=null?(pr.path_valid?'YES':'NO'):'—';
  el('risk-path-reason').textContent=pr.selected_path_reason||'—';
  {
    const rb=el('risk-badge');
    const rlb=el('risk-level-banner');
    rlb.textContent='RISK: '+riskLevel;
    const riskClass={'LOW':'risk-low','MODERATE':'risk-mod','HIGH':'risk-high','CRITICAL':'risk-crit'}[riskLevel]||'safe-unk';
    rb.className=riskClass;
    rlb.className=riskClass;
  }

  /* ── dynamic obstacles ── */
  const obs=s.obstacles||{};
  el('dyn-count').textContent=obs.dynamic_count||0;
  el('obs-total').textContent=obs.total_tracked||0;
  el('obs-risk-score').textContent=fmt(obs.dynamic_risk,3);
  {
    const tracked=obs.tracked||[];
    if(tracked.length===0){
      el('obs-list').innerHTML='<div style="color:var(--muted)">No obstacles tracked.</div>';
    }else{
      el('obs-list').innerHTML=tracked.map(o=>{
        const dynamic=o.is_dynamic?'<span style="color:var(--red)">DYNAMIC</span>':'<span style="color:var(--muted)">static</span>';
        const speed=o.speed?` ${fmt(o.speed,2)} m/s`:'';
        return `<div style="padding:4px 0;border-bottom:1px solid var(--border)">
          <b style="color:var(--cyan)">${esc(o.id)}</b> ${dynamic}${speed}<br>
          pos (${fmt(o.x,2)}, ${fmt(o.y,2)}) dim ${fmt(o.dim_x,1)}×${fmt(o.dim_y,1)}m conf ${fmt(o.confidence,2)}
        </div>`;
      }).join('');
    }
  }

  /* ── mission intelligence events ── */
  const mev=s.mission_events||[];
  if(mev.length){
    const sevColor={
      'INFO':'var(--dim)','WARNING':'var(--amber)','CRITICAL':'var(--red)',
      'RECOVERY':'var(--violet)','MISSION':'var(--cyan)'};
    el('mission-events-body').innerHTML=mev.map(e=>`<tr>
      <td class="ts">${esc(e.time)}</td>
      <td style="color:${sevColor[e.severity]||'var(--dim)'}">${esc(e.severity)}</td>
      <td class="rs">${esc(e.subsystem||'')}</td>
      <td class="ev">${esc(e.description||'')}</td>
    </tr>`).join('');
  }else{
    el('mission-events-body').innerHTML='<tr><td colspan="4" style="color:var(--muted);padding:12px 8px">Awaiting mission events from live ROS stack.</td></tr>';
  }

  /* ── topic table ── */
  el('topics').innerHTML=(s.topics||[]).map(t=>`<tr>
    <td style="color:var(--text)">${t.name}</td>
    <td style="color:var(--muted);font-size:10px">${t.type.split('/').pop()}</td>
    <td>${fmt(t.rate,1)} Hz</td>
    <td>${age(t.age)}</td>
    <td class="${t.status==='LIVE'?'live':t.status==='STALE'?'stale':'offline'}">${t.status}</td>
  </tr>`).join('');

  /* ── canvas draws ── */
  drawMap(s);
  drawCloud(s);
}

function esc(s){
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}

/* imu panel needs the imu sub-object exposed; polyfill if absent */
Object.defineProperty(Object.prototype,'imu',{get(){return undefined},configurable:true});

setInterval(update,1000);
update();
</script>
</body></html>'''


class DashboardNode(Node):
    """Collect actual ROS runtime state for the browser UI."""

    def __init__(self):
        super().__init__('sih26126_dashboard')
        self.cloud_times = deque(maxlen=80)
        self.last_exploration = None
        self.last_safety = None
        self.last_steering = None
        self.last_obstacle_event = 0.0
        self.last_health = {}
        self.last_system = None
        self.map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, '/camera', self.camera_callback, 5)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 5)
        self.create_subscription(Image, '/traversability/score_map', self.trav_callback, 5)
        self.create_subscription(Float32, '/traversability/mean_score', self.mean_callback, 10)
        self.create_subscription(String, '/terrain/class', self.class_callback, 10)
        self.create_subscription(Float32, '/terrain/class_confidence', self.class_confidence_callback, 10)
        self.create_subscription(Float32, '/geometry/geometry_confidence', self.geometry_confidence_callback, 10)
        self.create_subscription(Image, '/geometry/slope_map', self.slope_callback, 2)
        self.create_subscription(Image, '/geometry/roughness_map', self.roughness_callback, 2)
        self.create_subscription(Odometry, '/model/sih_ugv/odometry', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/localization/pose', self.pose_callback, 10)
        self.create_subscription(String, '/localization/source', self.source_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(PointCloud2, '/depth_camera/points', self.cloud_callback, 2)
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, self.map_qos)
        self.create_subscription(String, '/mapping/state', self.mapping_callback, 10)
        self.create_subscription(String, '/exploration/state', self.exploration_callback, 10)
        self.create_subscription(String, '/mission/state', self.mission_callback, 10)
        self.create_subscription(PoseStamped, '/mission/home_pose', self.home_callback, 10)
        self.create_subscription(Float32, '/mission/known_area_m2', self.known_area_callback, 10)
        self.create_subscription(String, '/mission/reason', self.mission_reason_callback, 10)
        self.create_subscription(String, '/mission/metrics', self.mission_metrics_callback, 10)
        self.create_subscription(String, '/mission/result_path', self.mission_result_path_callback, 10)
        self.create_subscription(Int32, '/exploration/frontier_count', self.frontier_count_callback, 10)
        self.create_subscription(MarkerArray, '/exploration/frontiers', self.frontiers_callback, 5)
        self.create_subscription(Float32, '/exploration/explored_percentage', self.explored_callback, 10)
        self.create_subscription(Float32, '/exploration/distance_to_frontier', self.frontier_distance_callback, 10)
        self.create_subscription(PointStamped, '/exploration/goal', self.goal_callback, 10)
        self.create_subscription(PointStamped, '/mission/goal', self.mission_goal_callback, 10)
        self.create_subscription(Path, '/planner/global_path', self.global_path_callback, 10)
        self.create_subscription(Path, '/planner/local_path', self.local_path_callback, 10)
        self.create_subscription(String, '/planner/global_path_status', self.path_status_callback, 10)
        self.create_subscription(String, '/planner/navigation_state', self.planner_state_callback, 10)
        self.create_subscription(Float32, '/planner/recommended_speed', self.recommended_speed_callback, 10)
        self.create_subscription(String, '/planner/local_navigation_state', self.local_state_callback, 10)
        self.create_subscription(Float32, '/planner/local_steering_angle', self.steering_callback, 10)
        self.create_subscription(Float32, '/planner/local_obstacle_clearance', self.clearance_callback, 10)
        self.create_subscription(Float32, '/planner/local_obstacle_score', self.obstacle_score_callback, 10)
        self.create_subscription(String, '/safety/state', self.safety_callback, 10)
        self.create_subscription(String, '/safety/reason', self.safety_reason_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 10)
        self.create_subscription(
            Twist, '/planner/local_recommended_cmd_vel', self.local_command_callback, 10)
        # ── New: autonomy mode, obstacle tracking, planner risk, mission intelligence ──
        self.create_subscription(
            AutonomyMode, '/autonomy/mode', self.autonomy_mode_callback, 10)
        self.create_subscription(
            TrackedObstacleArray, '/obstacles/tracked', self.tracked_obstacles_callback, 5)
        self.create_subscription(
            Float32, '/obstacles/dynamic_risk', self.dynamic_risk_cb, 10)
        self.create_subscription(
            PlannerRisk, '/planner/risk_assessment', self.planner_risk_callback, 10)
        self.create_subscription(
            MissionEvent, '/autonomy/events', self.mission_event_callback, 10)
        self.create_subscription(
            String, '/autonomy/summary', self.autonomy_summary_callback, 10)
        self.create_subscription(
            Float32, '/localization/confidence', self.loc_confidence_callback, 10)
        self.create_timer(2.0, self.refresh_topics)
        event('Dashboard connected; awaiting live subsystem messages',
              reason='startup', value='')

    def mark(self, name):
        now = time.time()
        TOPIC_TIMES[name] = now
        TOPIC_COUNTS[name] = TOPIC_COUNTS.get(name, 0) + 1
        TOPIC_WINDOWS.setdefault(name, deque(maxlen=100)).append(now)

    def camera_callback(self, msg):
        frame = jpeg_rgb(msg)
        if frame is None:
            return
        now = time.time()
        with LOCK:
            first = STATE['camera_count'] == 0
            STATE.update(camera_jpeg=frame, camera_last=now,
                         camera_width=msg.width, camera_height=msg.height)
            STATE['camera_count'] += 1
            STATE['camera_times'].append(now)
            self.mark('/camera')
        if first:
            event(f'RGB sensor live',
                  reason='first frame received',
                  value=f'{msg.width}×{msg.height}')

    def camera_info_callback(self, _msg):
        with LOCK:
            self.mark('/camera_info')

    def trav_callback(self, msg):
        frame = jpeg_traversability(msg)
        if frame is None:
            return
        with LOCK:
            STATE.update(trav_jpeg=frame, trav_last=time.time(),
                         trav_width=msg.width, trav_height=msg.height)
            STATE['trav_count'] += 1
            self.mark('/traversability/score_map')

    def mean_callback(self, msg):
        with LOCK:
            STATE['mean_score'] = float(msg.data)
            self.mark('/traversability/mean_score')

    def class_callback(self, msg):
        with LOCK:
            STATE['trav_class'] = msg.data
            self.mark('/terrain/class')

    def class_confidence_callback(self, msg):
        with LOCK:
            STATE['class_confidence'] = float(msg.data)
            self.mark('/terrain/class_confidence')

    def geometry_confidence_callback(self, msg):
        with LOCK:
            STATE['geometry_confidence'] = float(msg.data)
            self.mark('/geometry/geometry_confidence')

    def image_mean(self, msg):
        if msg.encoding != '32FC1' or msg.step < msg.width * 4:
            return None
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < msg.step * msg.height:
            return None
        values = np.ndarray((msg.height, msg.width), dtype=np.float32,
                            buffer=raw, strides=(msg.step, 4))
        finite = values[np.isfinite(values)]
        return float(np.mean(finite)) if finite.size else None

    def slope_callback(self, msg):
        value = self.image_mean(msg)
        with LOCK:
            STATE['slope_mean'] = value
            self.mark('/geometry/slope_map')

    def roughness_callback(self, msg):
        value = self.image_mean(msg)
        with LOCK:
            STATE['roughness_mean'] = value
            self.mark('/geometry/roughness_map')

    def odom_callback(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with LOCK:
            STATE.update(odom_last=time.time(), odom={
                'x': float(p.x), 'y': float(p.y), 'yaw': float(yaw),
                'vx': float(v.linear.x), 'wz': float(v.angular.z)})
            self.mark('/model/sih_ugv/odometry')

    def pose_callback(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with LOCK:
            STATE['localization_last'] = time.time()
            STATE['localization_pose'] = {'x': float(p.x), 'y': float(p.y), 'yaw': float(yaw)}
            self.mark('/localization/pose')

    def source_callback(self, msg):
        with LOCK:
            STATE['localization_source'] = msg.data
            self.mark('/localization/source')

    def imu_callback(self, msg):
        a, g = msg.linear_acceleration, msg.angular_velocity
        with LOCK:
            STATE['imu_last'] = time.time()
            STATE['imu'] = {'ax': float(a.x), 'ay': float(a.y), 'az': float(a.z), 'gz': float(g.z)}
            self.mark('/imu')

    def cloud_callback(self, msg):
        stats = cloud_stats(msg)
        now = time.time()
        with LOCK:
            self.cloud_times.append(now)
            CLOUD_TIMES.append(now)
            STATE['cloud_last'] = now
            STATE['cloud_stats'] = stats
            self.mark('/depth_camera/points')
        if (stats and stats.get('nearest') is not None
                and stats['nearest'] < 1.5
                and now - self.last_obstacle_event > 3.0):
            event('Obstacle in depth field',
                  reason='height-filtered return within threshold',
                  value=f"{stats['nearest']:.2f} m")
            self.last_obstacle_event = now

    def clock_callback(self, msg):
        with LOCK:
            STATE['clock_sec'] = msg.clock.sec + msg.clock.nanosec * 1e-9
            self.mark('/clock')

    def map_callback(self, msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        known = data >= 0
        known_count = int(np.count_nonzero(known))
        free = int(np.count_nonzero(data == 0))
        occupied = int(np.count_nonzero(data >= 50))
        unknown = int(data.size - known_count)
        bounds = None
        rows, cols = np.nonzero(known)
        if len(rows):
            bounds = [int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())]
        map_data = {
            'width': int(msg.info.width), 'height': int(msg.info.height),
            'resolution': float(msg.info.resolution),
            'origin_x': float(msg.info.origin.position.x),
            'origin_y': float(msg.info.origin.position.y),
            'frame': msg.header.frame_id,
            'data': base64.b64encode(data.tobytes()).decode('ascii'),
            'known': known_count, 'free': free, 'occupied': occupied, 'unknown': unknown,
            'known_percent': 100.0 * known_count / max(1, data.size),
            'known_area_m2': known_count * float(msg.info.resolution) ** 2,
            'bounds': bounds,
        }
        with LOCK:
            previous = STATE['map']
            STATE['map'] = map_data
            STATE['map_last'] = time.time()
            self.mark('/map')
        if known_count and (previous is None or previous['known'] == 0):
            event('Online map building from live depth observations',
                  reason='first known cells observed',
                  value=f'{known_count} cells')
        elif previous and map_data['known_percent'] - previous['known_percent'] >= 2.0:
            event('Known map area expanded',
                  reason='frontier exploration progress',
                  value=f"{map_data['known_percent']:.1f}%")

    def mapping_callback(self, msg):
        with LOCK:
            changed = STATE['mapping_state'] != msg.data
            STATE['mapping_state'] = msg.data
            self.mark('/mapping/state')
        if changed:
            event('Mapping state changed',
                  reason='mapping subsystem transition',
                  value=msg.data)

    def exploration_callback(self, msg):
        with LOCK:
            prev = STATE['exploration_state']
            changed = prev != msg.data
            STATE['exploration_state'] = msg.data
            self.mark('/exploration/state')
        if changed:
            event('Exploration mode changed',
                  reason='exploration state machine transition',
                  value=f'{prev} → {msg.data}')

    def mission_callback(self, msg):
        with LOCK:
            prev = STATE['mission_state']
            changed = prev != msg.data
            STATE['mission_state'] = msg.data
            self.mark('/mission/state')
        if changed:
            event('Mission state changed',
                  reason='mission manager transition',
                  value=f'{prev} → {msg.data}')

    def home_callback(self, msg):
        q = msg.pose.orientation
        with LOCK:
            STATE['home'] = {
                'x': float(msg.pose.position.x), 'y': float(msg.pose.position.y),
                'yaw': math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z)),
            }
            self.mark('/mission/home_pose')
            first = not STATE['home_logged']
            STATE['home_logged'] = True
        if first:
            event('Startup home pose captured',
                  reason='first /mission/home_pose received',
                  value=f"({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})")

    def known_area_callback(self, msg):
        with LOCK:
            STATE['known_area_m2'] = float(msg.data)
            self.mark('/mission/known_area_m2')

    def mission_reason_callback(self, msg):
        with LOCK:
            changed = STATE['mission_reason'] != msg.data
            STATE['mission_reason'] = msg.data
            self.mark('/mission/reason')
        if changed and msg.data:
            event('Mission issue reported',
                  reason='mission manager',
                  value=msg.data)

    def mission_metrics_callback(self, msg):
        try:
            metrics = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        with LOCK:
            STATE['mission_metrics'] = metrics
            self.mark('/mission/metrics')

    def mission_result_path_callback(self, msg):
        with LOCK:
            STATE['mission_result_path'] = msg.data
            self.mark('/mission/result_path')

    def frontier_count_callback(self, msg):
        with LOCK:
            STATE['frontier_count'] = int(msg.data)
            self.mark('/exploration/frontier_count')

    def frontiers_callback(self, msg):
        points = []
        goal = None
        for marker in msg.markers:
            if marker.ns == 'frontiers':
                points.extend({'x': float(p.x), 'y': float(p.y)} for p in marker.points)
            elif marker.ns == 'active_frontier' and marker.action != 2:
                goal = {'x': float(marker.pose.position.x), 'y': float(marker.pose.position.y)}
        with LOCK:
            STATE['frontiers'] = points[:2000]
            if goal is not None:
                STATE['goal'] = goal
            self.mark('/exploration/frontiers')

    def explored_callback(self, msg):
        with LOCK:
            STATE['explored_percent'] = float(msg.data)
            self.mark('/exploration/explored_percentage')

    def frontier_distance_callback(self, msg):
        with LOCK:
            STATE['frontier_distance'] = float(msg.data)
            self.mark('/exploration/distance_to_frontier')

    def goal_callback(self, msg):
        p = {'x': float(msg.point.x), 'y': float(msg.point.y)}
        with LOCK:
            old = STATE['goal']
            STATE['goal'] = p
            self.mark('/exploration/goal')
        if old is None or math.hypot(old['x'] - p['x'], old['y'] - p['y']) > 0.4:
            event('Frontier target selected',
                  reason='exploration planner goal update',
                  value=f"({p['x']:.1f}, {p['y']:.1f})")

    def mission_goal_callback(self, msg):
        p = {'x': float(msg.point.x), 'y': float(msg.point.y)}
        with LOCK:
            previous = STATE['goal']
            STATE['goal'] = p
            self.mark('/mission/goal')
        if previous is None or math.hypot(previous['x'] - p['x'], previous['y'] - p['y']) > 0.4:
            event('Mission navigation target updated',
                  reason='/mission/goal changed',
                  value=f"({p['x']:.1f}, {p['y']:.1f})")

    @staticmethod
    def path_points(msg):
        return [{'x': float(p.pose.position.x), 'y': float(p.pose.position.y)}
                for p in msg.poses]

    def global_path_callback(self, msg):
        pts = self.path_points(msg)
        with LOCK:
            old_len = len(STATE['global_path'])
            STATE['global_path'] = pts
            self.mark('/planner/global_path')
        if abs(len(pts) - old_len) > 5:
            event('Global path updated',
                  reason='planner replanned route',
                  value=f'{len(pts)} waypoints')

    def local_path_callback(self, msg):
        with LOCK:
            STATE['local_path'] = self.path_points(msg)
            self.mark('/planner/local_path')

    def path_status_callback(self, msg):
        with LOCK:
            old = STATE['global_path_status']
            STATE['global_path_status'] = msg.data
            self.mark('/planner/global_path_status')
        if old != msg.data:
            event('Global path status changed',
                  reason='planner status update',
                  value=f'{old} → {msg.data}')

    def planner_state_callback(self, msg):
        with LOCK:
            old = STATE['planner_state']
            STATE['planner_state'] = msg.data
            self.mark('/planner/navigation_state')
        if old != msg.data:
            event('Global planner state changed',
                  reason='navigation state machine',
                  value=f'{old} → {msg.data}')

    def recommended_speed_callback(self, msg):
        with LOCK:
            old_speed = STATE['recommended_speed']
            STATE['recommended_speed'] = float(msg.data)
            self.mark('/planner/recommended_speed')
        # Log significant speed changes (>0.1 m/s change and not first msg)
        new_speed = float(msg.data)
        if old_speed is not None and abs(new_speed - old_speed) >= 0.1:
            event('Recommended speed changed',
                  reason='terrain/heading planner advisory',
                  value=f'{old_speed:.2f} → {new_speed:.2f} m/s')

    def local_state_callback(self, msg):
        with LOCK:
            old = STATE['local_state']
            STATE['local_state'] = msg.data
            self.mark('/planner/local_navigation_state')
        if old != msg.data:
            event('Local planner state changed',
                  reason='local navigation state machine',
                  value=f'{old} → {msg.data}')

    def steering_callback(self, msg):
        angle = float(msg.data)
        with LOCK:
            old = self.last_steering
            self.last_steering = angle
            STATE['steering_deg'] = angle
            self.mark('/planner/local_steering_angle')
        if old is not None and abs(angle - old) >= 8.0:
            event('Local steering changed',
                  reason='corridor selection updated',
                  value=f'{old:.1f}° → {angle:.1f}°')

    def clearance_callback(self, msg):
        with LOCK:
            STATE['clearance'] = float(msg.data)
            self.mark('/planner/local_obstacle_clearance')

    def obstacle_score_callback(self, msg):
        with LOCK:
            STATE['obstacle_score'] = float(msg.data)
            self.mark('/planner/local_obstacle_score')

    def safety_callback(self, msg):
        with LOCK:
            old = self.last_safety
            self.last_safety = msg.data
            STATE['safety_state'] = msg.data
            self.mark('/safety/state')
            reason = STATE['safety_reason']
        if old != msg.data:
            event('Safety state changed',
                  reason=reason or 'safety supervisor',
                  value=f'{old} → {msg.data}')

    def safety_reason_callback(self, msg):
        with LOCK:
            old = STATE['safety_reason']
            STATE['safety_reason'] = msg.data
            self.mark('/safety/reason')
            state = STATE['safety_state']
        if old != msg.data and state != 'SAFE':
            event('Safety stop reason updated',
                  reason='safety supervisor',
                  value=msg.data)

    def command_callback(self, msg):
        with LOCK:
            STATE['cmd_last'] = time.time()
            STATE['cmd'] = {'vx': float(msg.linear.x), 'wz': float(msg.angular.z)}
            self.mark('/cmd_vel')

    def local_command_callback(self, msg):
        with LOCK:
            STATE['local_cmd_last'] = time.time()
            STATE['local_cmd'] = {
                'vx': float(msg.linear.x), 'wz': float(msg.angular.z),
            }
            self.mark('/planner/local_recommended_cmd_vel')

    def autonomy_mode_callback(self, msg: AutonomyMode):
        with LOCK:
            old = STATE['autonomy_mode']
            STATE['autonomy_mode'] = msg.mode
            STATE['autonomy_reason'] = msg.reason
            STATE['autonomy_is_degraded'] = bool(msg.is_degraded)
            STATE['autonomy_capability'] = float(msg.capability_score)
            STATE['autonomy_degradations'] = list(msg.active_degradations)
            STATE['autonomy_last'] = time.time()
            self.mark('/autonomy/mode')
        if old != msg.mode:
            event('Autonomy mode changed', reason=msg.reason or 'state machine', value=msg.mode)

    def tracked_obstacles_callback(self, msg: TrackedObstacleArray):
        with LOCK:
            obs_list = []
            for o in msg.obstacles:
                obs_list.append({
                    'id': o.id,
                    'x': round(float(o.position.x), 2),
                    'y': round(float(o.position.y), 2),
                    'z': round(float(o.position.z), 2),
                    'vx': round(float(o.velocity.x), 3),
                    'vy': round(float(o.velocity.y), 3),
                    'speed': round(math.hypot(float(o.velocity.x), float(o.velocity.y)), 3),
                    'confidence': round(float(o.confidence), 2),
                    'is_dynamic': bool(o.is_dynamic),
                    'classification': o.classification,
                    'age_sec': round(float(o.age_sec), 1),
                    'dim_x': round(float(o.dimensions.x), 2),
                    'dim_y': round(float(o.dimensions.y), 2),
                    'dim_z': round(float(o.dimensions.z), 2),
                    'predicted_x': round(float(o.predicted_position.x), 2),
                    'predicted_y': round(float(o.predicted_position.y), 2),
                })
            dynamic_count = sum(1 for o in obs_list if o['is_dynamic'])
            was_dynamic = STATE['dynamic_obs_count']
            STATE['dynamic_obs_list'] = obs_list
            STATE['dynamic_obs_count'] = dynamic_count
            self.mark('/obstacles/tracked')
        if was_dynamic == 0 and dynamic_count > 0:
            event('Dynamic obstacle detected', reason='obstacle tracker', value=f'{dynamic_count} moving object(s)')
        elif was_dynamic > 0 and dynamic_count == 0:
            event('Dynamic obstacle cleared', reason='obstacle tracker', value='area clear')

    def dynamic_risk_cb(self, msg: Float32):
        with LOCK:
            STATE['dynamic_risk'] = max(0.0, min(1.0, float(msg.data)))
            STATE['dynamic_risk_last'] = time.time()
            self.mark('/obstacles/dynamic_risk')

    def planner_risk_callback(self, msg: PlannerRisk):
        with LOCK:
            STATE['planner_risk'] = {
                'total_cost': round(float(msg.total_cost), 3),
                'terrain_cost': round(float(msg.terrain_cost), 3),
                'obstacle_cost': round(float(msg.obstacle_cost), 3),
                'dynamic_risk_cost': round(float(msg.dynamic_risk_cost), 3),
                'localization_risk': round(float(msg.localization_risk), 3),
                'sensor_degradation_cost': round(float(msg.sensor_degradation_cost), 3),
                'risk_level': msg.risk_level,
                'path_valid': bool(msg.path_valid),
                'selected_path_reason': msg.selected_path_reason,
            }
            STATE['planner_risk_last'] = time.time()
            self.mark('/planner/risk_assessment')

    def mission_event_callback(self, msg: MissionEvent):
        with LOCK:
            ev = {
                'time': datetime.now().strftime('%H:%M:%S'),
                'severity': msg.severity,
                'subsystem': msg.subsystem,
                'description': msg.description,
                'cause': msg.cause,
                'action': msg.action_taken,
                'mode': msg.autonomy_mode,
            }
            STATE['mission_events'].appendleft(ev)
            self.mark('/autonomy/events')
        # Also push critical/warning events to the main EVENTS deque
        if msg.severity in ('WARNING', 'CRITICAL', 'RECOVERY', 'MISSION'):
            event(msg.description, reason=msg.subsystem, value=msg.cause)

    def autonomy_summary_callback(self, msg: String):
        with LOCK:
            STATE['autonomy_summary'] = msg.data
            self.mark('/autonomy/summary')

    def loc_confidence_callback(self, msg: Float32):
        with LOCK:
            STATE['localization_confidence'] = round(float(msg.data), 3)
            self.mark('/localization/confidence')

    def refresh_topics(self):
        try:
            found = dict(self.get_topic_names_and_types())
        except Exception:
            return
        with LOCK:
            TOPIC_TYPES.clear()
            TOPIC_TYPES.update({k: (v[0] if v else 'unknown') for k, v in found.items()})
        now = time.time()
        health, system = subsystem_health({
            'camera': age_of(STATE['camera_last'], now),
            'cloud': age_of(STATE['cloud_last'], now),
            'imu': age_of(STATE['imu_last'], now),
            'localization': age_of(STATE['localization_last'], now),
            'mapping': age_of(STATE['map_last'], now),
            'traversability': age_of(STATE['trav_last'], now),
            'exploration': age_of(TOPIC_TIMES.get('/exploration/state'), now),
            'global_planner': age_of(TOPIC_TIMES.get('/planner/global_path'), now),
            'local_planner': age_of(TOPIC_TIMES.get('/planner/local_steering_angle'), now),
            'safety': age_of(TOPIC_TIMES.get('/safety/state'), now),
        })
        for name, status in health.items():
            if (self.last_health.get(name) != status
                    and status in ('STALE', 'OFFLINE', 'DEGRADED')):
                event(f'Subsystem health degraded',
                      reason=f'{name} sensor stream',
                      value=status)
        if self.last_system != system and system != 'ONLINE':
            event('ROS autonomy status changed',
                  reason='subsystem health check',
                  value=system)
        self.last_health = health
        self.last_system = system


def age_of(stamp, now):
    return None if stamp is None else max(0.0, now - stamp)


def age_health(age, stale_after=3.0):
    """Classify a message stream from its measured age."""
    if age is None:
        return 'OFFLINE'
    return 'LIVE' if age < stale_after else 'STALE'


def subsystem_health(ages):
    """Return subsystem health and system state from measured message ages."""
    camera = age_health(ages.get('camera'))
    cloud = age_health(ages.get('cloud'))
    sensor = ('LIVE' if camera == cloud == 'LIVE'
              else 'DEGRADED' if 'LIVE' in (camera, cloud)
              else 'OFFLINE' if camera == cloud == 'OFFLINE' else 'STALE')
    health = {
        'RGB': camera,
        'DEPTH': cloud,
        'IMU': age_health(ages.get('imu')),
        'POSE': age_health(ages.get('localization')),
        'MAP': age_health(ages.get('mapping')),
        'SENSOR': sensor,
        'LOCALIZATION': age_health(ages.get('localization')),
        'MAPPING': age_health(ages.get('mapping')),
        'TRAVERSABILITY': age_health(ages.get('traversability')),
        'EXPLORATION': age_health(ages.get('exploration')),
        'GLOBAL PLANNER': age_health(ages.get('global_planner')),
        'LOCAL PLANNER': age_health(ages.get('local_planner')),
        'SAFETY': age_health(ages.get('safety')),
        'DASHBOARD': 'LIVE',
    }
    core = [status for name, status in health.items() if name != 'DASHBOARD']
    if not any(status in ('LIVE', 'DEGRADED') for status in core):
        system = 'OFFLINE'
    elif all(status == 'LIVE' for status in core):
        system = 'ONLINE'
    else:
        system = 'DEGRADED'
    return health, system


def status_payload():
    now = time.time()
    with LOCK:
        state = dict(STATE)
        state['camera_times'] = list(STATE['camera_times'])
        cloud_times = list(CLOUD_TIMES)
        camera_times = state['camera_times']
        fps = ((len(camera_times) - 1) / max(1e-6, camera_times[-1] - camera_times[0])
               if len(camera_times) > 1 else 0.0)
        cloud_rate = 0.0
        if len(cloud_times) > 1:
            cloud_rate = (len(cloud_times) - 1) / max(1e-6, cloud_times[-1] - cloud_times[0])
        camera_age = age_of(state['camera_last'], now)
        trav_age = age_of(state['trav_last'], now)
        cloud_age = age_of(state['cloud_last'], now)
        odom_age = age_of(state['odom_last'], now)
        loc_age = age_of(state['localization_last'], now)
        map_age = age_of(state['map_last'], now)
        cmd_age = age_of(state['cmd_last'], now)
        health, system = subsystem_health({
            'camera': camera_age,
            'cloud': cloud_age,
            'imu': age_of(state['imu_last'], now),
            'localization': loc_age,
            'mapping': map_age,
            'traversability': trav_age,
            'exploration': age_of(TOPIC_TIMES.get('/exploration/state'), now),
            'global_planner': age_of(TOPIC_TIMES.get('/planner/global_path'), now),
            'local_planner': age_of(TOPIC_TIMES.get('/planner/local_steering_angle'), now),
            'safety': age_of(TOPIC_TIMES.get('/safety/state'), now),
        })
        m = state['map'] or {'width': 0, 'height': 0, 'resolution': 0.0,
                             'origin_x': 0.0, 'origin_y': 0.0, 'data': '',
                             'known': 0, 'free': 0, 'occupied': 0, 'unknown': 0,
                             'known_percent': 0.0, 'known_area_m2': 0.0, 'bounds': None}
        d = state['cloud_stats'] or {}
        ranges = '—'
        if d.get('x_range'):
            ranges = ('x %.1f..%.1f · y %.1f..%.1f · z %.1f..%.1f' % (
                d['x_range'][0], d['x_range'][1], d['y_range'][0], d['y_range'][1],
                d['z_range'][0], d['z_range'][1]))
        topics = []
        for name in sorted(TOPIC_TYPES):
            if name not in TOPIC_TIMES and name not in {
                '/camera', '/camera_info', '/depth_camera/points', '/imu', '/clock',
                '/model/sih_ugv/odometry', '/map', '/cmd_vel',
                '/planner/global_path', '/planner/local_path',
                '/planner/local_recommended_cmd_vel', '/safety/state',
            }:
                continue
            stamp = TOPIC_TIMES.get(name)
            a = age_of(stamp, now)
            n = TOPIC_COUNTS.get(name, 0)
            stamps = list(TOPIC_WINDOWS.get(name, ()))
            hz = ((len(stamps) - 1) / max(1e-6, stamps[-1] - stamps[0])
                  if len(stamps) > 1 else float(n))
            topics.append({'name': name, 'type': TOPIC_TYPES[name], 'age': a,
                           'rate': hz, 'status': 'OFFLINE' if a is None else 'LIVE' if a < 3 else 'STALE'})
        pose = state.get('localization_pose') or state['odom']
        pose = dict(pose)
        pose.update({'vx': state['odom']['vx'], 'wz': state['odom']['wz'], 'age': loc_age})
        depth = {
            'age': cloud_age, 'rate_hz': cloud_rate, 'frame': d.get('frame'),
            'nearest': d.get('nearest'), 'finite': d.get('finite', 0),
            'samples': d.get('samples', 0),
            'valid_ratio': d.get('finite', 0) / max(1, d.get('samples', 0)),
            'obstacle_samples': d.get('obstacle_samples', 0), 'ranges': ranges,
            'points': d.get('points', []),
        }
        # Enrich events with reason and value for the timeline
        enriched_events = list(EVENTS)
        payload = {
            'system': system, 'domain': os.environ.get('ROS_DOMAIN_ID', '0'),
            'runtime': time.monotonic() - STARTED, 'sim_time': state['clock_sec'],
            'camera': {
                'resolution': f"{state['camera_width']}×{state['camera_height']}"
                              if state['camera_width'] else '—',
                'fps': fps, 'age': camera_age, 'frames': state['camera_count']},
            'traversability': {'mean_score': state['mean_score'], 'age': trav_age},
            'terrain': {
                'class': state['trav_class'],
                'class_confidence': state['class_confidence'],
                'slope_deg': state['slope_mean'],
                'roughness_m': state['roughness_mean'],
                'geometry_confidence': state['geometry_confidence']},
            'depth': depth,
            'pose': pose,
            'imu': state['imu'],
            'localization': {'source': state['localization_source']},
            'mapping': {'state': state['mapping_state'], 'age': map_age},
            'mission': {
                'state': state['mission_state'],
                'home': state['home'],
                'known_area_m2': state['known_area_m2'],
                'reason': state['mission_reason'],
                'metrics': state['mission_metrics'],
                'result_path': state['mission_result_path']},
            'map': {
                **m,
                'home': state['home'],
                'global_path': state['global_path'],
                'local_path': state['local_path'],
                'frontiers': state.get('frontiers', []),
                'goal': state['goal']},
            'exploration': {
                'state': state['exploration_state'],
                'frontier_count': state['frontier_count'],
                'explored_percent': state['explored_percent'],
                'distance': state['frontier_distance'],
                'goal': state['goal']},
            'planner': {
                'global_path_status': state['global_path_status'],
                'state': state['planner_state'],
                'recommended_speed': state['recommended_speed'],
                'local_state': state['local_state'],
                'steering_deg': state['steering_deg'],
                'clearance': state['clearance'],
                'obstacle_score': state['obstacle_score'],
                'local_command': {
                    **state['local_cmd'],
                    'age': age_of(state['local_cmd_last'], now)}},
            'safety': {'state': state['safety_state'], 'reason': state['safety_reason']},
            'command': {**state['cmd'], 'age': cmd_age},
            'health': health,
            'events': enriched_events,
            'topics': topics,
            'autonomy': {
                'mode': state['autonomy_mode'],
                'reason': state['autonomy_reason'],
                'is_degraded': state['autonomy_is_degraded'],
                'capability_score': state['autonomy_capability'],
                'active_degradations': state['autonomy_degradations'],
                'age': age_of(state['autonomy_last'], now),
                'summary': state['autonomy_summary'],
            },
            'obstacles': {
                'dynamic_count': state['dynamic_obs_count'],
                'total_tracked': len(state['dynamic_obs_list']),
                'tracked': state['dynamic_obs_list'],
                'dynamic_risk': state['dynamic_risk'],
                'risk_age': age_of(state['dynamic_risk_last'], now),
            },
            'planner_risk': state['planner_risk'] or {},
            'planner_risk_age': age_of(state['planner_risk_last'], now),
            'mission_events': list(state['mission_events'])[:20],
            'localization_confidence': state['localization_confidence'],
        }
    return payload


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_args):
        return

    def send_data(self, data, content_type):
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/':
            self.send_data(HTML.encode(), 'text/html; charset=utf-8')
        elif self.path == '/api/status':
            self.send_data(json.dumps(status_payload(), separators=(',', ':')).encode(),
                           'application/json')
        elif self.path in ('/camera.mjpg', '/trav.mjpg'):
            self.stream('camera_jpeg' if self.path.startswith('/camera') else 'trav_jpeg')
        else:
            self.send_response(404)
            self.end_headers()

    def stream(self, key):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache, no-store')
        self.end_headers()
        previous = None
        try:
            while True:
                with LOCK:
                    frame = STATE[key]
                if frame is not None and frame is not previous:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                    self.wfile.flush()
                    previous = frame
                time.sleep(0.04)
        except (BrokenPipeError, ConnectionResetError):
            return


def main():
    rclpy.init()
    node = DashboardNode()
    server = None
    selected_port = PORT
    for selected_port in range(PORT, PORT + 11):
        try:
            server = ThreadingHTTPServer((HOST, selected_port), Handler)
            break
        except OSError as exc:
            print(f'Dashboard port {selected_port} unavailable: {exc}', flush=True)
    if server is None:
        node.get_logger().error(f'No dashboard port available from {PORT} through {PORT + 10}')
        node.destroy_node()
        rclpy.shutdown()
        return
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'DRISTHI live autonomy dashboard: http://localhost:{selected_port}', flush=True)
    print('Read-only UI; map, sensors, autonomy, and safety are sourced from ROS messages.', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
