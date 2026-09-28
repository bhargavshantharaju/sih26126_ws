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


HOST = '0.0.0.0'
PORT = int(os.environ.get('SIH26126_DASHBOARD_PORT', '8080'))
LOCK = threading.RLock()
STARTED = time.monotonic()
EVENTS = deque(maxlen=80)
TOPIC_TYPES = {}
TOPIC_TIMES = {}
TOPIC_COUNTS = {}
TOPIC_WINDOWS = {}
CLOUD_TIMES = deque(maxlen=80)


def event(message):
    """Append an event based on a received ROS/runtime transition."""
    EVENTS.appendleft({
        'time': datetime.now().strftime('%H:%M:%S'),
        'message': str(message),
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
    'cloud_last': None, 'cloud_stats': None, 'clock_sec': None,
    'map_last': None, 'map': None, 'mapping_state': 'INITIALIZING',
    'exploration_state': 'MAPPING', 'frontier_count': 0, 'explored_percent': 0.0,
    'frontier_distance': None, 'goal': None, 'global_path': [], 'local_path': [],
    'frontiers': [],
    'global_path_status': 'WAITING', 'planner_state': 'WAITING',
    'recommended_speed': None, 'local_state': 'WAITING',
    'steering_deg': None, 'clearance': None,
    'obstacle_score': None, 'safety_state': 'UNKNOWN', 'safety_reason': 'WAITING',
}

HTML = r'''<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SIH26126 Autonomous UGV</title><style>
:root{color-scheme:dark;--bg:#0a1017;--panel:#111b25;--line:#263746;--muted:#91a2b2;--text:#e6edf3;--cyan:#42d7e8;--green:#52d273;--amber:#f1bb55;--red:#fa6e68}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px Inter,Arial,sans-serif}header{position:sticky;top:0;z-index:4;background:#0d151e;border-bottom:1px solid var(--line);padding:13px 22px;display:flex;align-items:center;justify-content:space-between}h1{font-size:20px;letter-spacing:.08em;margin:0}h2{font-size:14px;letter-spacing:.09em;color:#b9c9d7;margin:0 0 12px;text-transform:uppercase}.sub{color:var(--muted);font-size:12px;margin-top:4px}.header-right{display:flex;gap:10px;align-items:center}.pill{border:1px solid var(--line);padding:7px 10px;border-radius:7px;font-size:12px}.status{font-weight:700;color:var(--green)}main{max-width:1600px;margin:auto;padding:16px;display:grid;grid-template-columns:repeat(12,minmax(0,1fr));gap:12px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:9px;padding:14px;min-width:0}.map-panel{grid-column:span 8}.side-panel{grid-column:span 4}.half{grid-column:span 6}.third{grid-column:span 4}.wide{grid-column:span 12}.map-wrap{height:min(62vh,680px);min-height:380px;background:#111820;border-radius:6px;overflow:hidden}.map-wrap canvas{width:100%;height:100%;display:block}.legend{display:flex;gap:14px;flex-wrap:wrap;color:var(--muted);font-size:11px;margin:10px 0}.swatch{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px}.metrics{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}.metric{background:#0c141c;border:1px solid #20303d;border-radius:7px;padding:9px}.label{font-size:10px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}.value{font-size:19px;font-weight:700;margin-top:5px}.small{font-size:12px;color:var(--muted)}.tag{font-size:11px;padding:5px 8px;border-radius:5px;background:#182734;color:var(--cyan);display:inline-block}.good{color:var(--green)}.warn{color:var(--amber)}.bad{color:var(--red)}.streams{display:grid;grid-template-columns:1fr 1fr;gap:10px}.stream img{width:100%;aspect-ratio:4/3;object-fit:contain;background:#05080b;border-radius:6px}.row{display:flex;justify-content:space-between;gap:8px;padding:7px 0;border-bottom:1px solid #20303d}.row:last-child{border:0}.eventlist{max-height:280px;overflow:auto}.event{padding:7px 0;border-bottom:1px solid #20303d;font-size:12px}.event time{color:var(--cyan);margin-right:10px}table{width:100%;border-collapse:collapse;font-size:12px}th,td{text-align:left;padding:8px;border-bottom:1px solid #20303d}th{color:var(--muted)}.health{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.health-item{padding:8px;background:#0c141c;border-radius:6px}.health-item b{display:block;margin-top:5px}.safety-banner{padding:11px;border-radius:6px;background:#133020;border:1px solid #245438;margin-bottom:10px;font-weight:800}.safety-banner.stopped{background:#3a191b;border-color:#743034}.topic-wrap{overflow:auto}@media(max-width:1000px){.map-panel,.side-panel,.half,.third{grid-column:span 12}.map-wrap{height:50vh}.streams{grid-template-columns:1fr}.health{grid-template-columns:repeat(2,1fr)}}
</style></head><body>
<header><div><h1>SIH26126 AUTONOMOUS UGV</h1><div class="sub">LIVE OPERATIONS · ODOMETRY-ASSISTED ONLINE MAPPING</div></div><div class="header-right"><span class="pill">DOMAIN <b id="domain">—</b></span><span class="pill">SIM <b id="simtime">—</b></span><span class="pill">RUNTIME <b id="runtime">—</b></span><span class="pill status" id="system">OFFLINE</span></div></header>
<main>
<section class="panel map-panel"><h2>Live occupancy map <span class="tag" id="mapstate">MAP: INITIALIZING</span></h2><div class="map-wrap"><canvas id="map"></canvas></div><div class="legend"><span><i class="swatch" style="background:#17232d"></i>UNKNOWN</span><span><i class="swatch" style="background:#dce8da"></i>FREE</span><span><i class="swatch" style="background:#e65d52"></i>OCCUPIED</span><span><i class="swatch" style="background:#42d7e8"></i>GLOBAL PATH</span><span><i class="swatch" style="background:#ffd166"></i>LOCAL PATH</span><span><i class="swatch" style="background:#3ee8c1"></i>FRONTIER</span><span>▲ ROBOT</span></div><div class="row"><span>Grid</span><b id="mapdims">—</b></div><div class="row"><span>Known / free / occupied / unknown</span><b id="cellcounts">—</b></div><div class="row"><span>Known-space</span><b id="knownpct">—</b></div></section>
<section class="panel side-panel"><h2>Live autonomy status</h2><div class="metric"><div class="label">Current mode</div><div class="value" id="mode">MAPPING</div></div><div class="metrics" style="margin-top:10px"><div class="metric"><div class="label">Active frontier distance</div><div class="value" id="frontierdist">—</div></div><div class="metric"><div class="label">Frontier candidates</div><div class="value" id="frontiercount">0</div></div><div class="metric"><div class="label">Global path</div><div class="value" id="pathstatus">WAITING</div></div><div class="metric"><div class="label">Explored</div><div class="value" id="explored">0%</div></div></div><div class="row"><span>Target</span><b id="target">—</b></div><div class="row"><span>Planner state</span><b id="plannerstate">WAITING</b></div><div class="row"><span>Localization source</span><b id="locsource">—</b></div><div class="row"><span>Mapping mode</span><b>ODOMETRY-ASSISTED</b></div></section>
<section class="panel half"><h2>Live RGB sensor</h2><div class="stream"><img src="/camera.mjpg" alt="Live RGB sensor"></div><div class="row"><span>Resolution / FPS / age</span><b id="camerainfo">—</b></div></section>
<section class="panel half"><h2>Derived traversability</h2><div class="stream"><img src="/trav.mjpg" alt="Live traversability score map"></div><div class="row"><span>Mean score</span><b id="travscore">—</b></div><div class="row"><span>Terrain class / confidence</span><b id="terrain">—</b></div><div class="row"><span>Slope / roughness / geometry confidence</span><b id="geometry">—</b></div></section>
<section class="panel third"><h2>Depth and obstacle observations</h2><div class="row"><span>Nearest height-filtered return</span><b id="nearest">—</b></div><div class="row"><span>Obstacle samples / valid sampled points</span><b id="depthcount">—</b></div><div class="row"><span>Cloud rate / sensor age</span><b id="cloudrate">—</b></div><div class="row"><span>XYZ range</span><b id="xyzrange">—</b></div><div class="sub" id="cloudframe">Sensor frame —</div></section>
<section class="panel third"><h2>Localization</h2><div class="metrics"><div class="metric"><div class="label">X</div><div class="value" id="x">—</div></div><div class="metric"><div class="label">Y</div><div class="value" id="y">—</div></div><div class="metric"><div class="label">Yaw</div><div class="value" id="yaw">—</div></div><div class="metric"><div class="label">Linear velocity</div><div class="value" id="vx">—</div></div></div><div class="row"><span>Angular velocity / pose age</span><b id="wzage">—</b></div><div class="row"><span>Pose source</span><b id="pose-source">Gazebo odometry</b></div></section>
<section class="panel third"><h2>Planner</h2><div class="row"><span>Global recommended speed</span><b id="recommendedspeed">—</b></div><div class="row"><span>Local state</span><b id="localstate">WAITING</b></div><div class="row"><span>Selected steering</span><b id="steering">—</b></div><div class="row"><span>Obstacle clearance / score</span><b id="obstacle">—</b></div><div class="row"><span>Goal/frontier distance</span><b id="plannerdistance">—</b></div></section>
<section class="panel half"><h2>Safety supervisor</h2><div id="safetybanner" class="safety-banner">SAFETY: UNKNOWN</div><div class="row"><span>Reason</span><b id="safetyreason">WAITING</b></div><div class="row"><span>Authoritative command (linear.x / angular.z)</span><b id="command">—</b></div><div class="row"><span>Command age</span><b id="commandage">—</b></div></section>
<section class="panel half"><h2>Subsystem health</h2><div id="health" class="health"></div></section>
<section class="panel half"><h2>Live event log</h2><div id="events" class="eventlist"></div></section>
<section class="panel half"><h2>ROS topic status</h2><div class="topic-wrap"><table><thead><tr><th>Topic</th><th>Type</th><th>Rate</th><th>Age</th><th>Status</th></tr></thead><tbody id="topics"></tbody></table></div></section>
</main>
<script>
let trail=[];let lastEventCount=0;
function fmt(v,n=2){return v===null||v===undefined||!Number.isFinite(v)?'—':Number(v).toFixed(n)}
function age(v){return v===null||v===undefined?'—':fmt(v,1)+' s'}
function drawMap(s){const c=document.getElementById('map'),ctx=c.getContext('2d'),m=s.map;if(!m||!m.data){ctx.clearRect(0,0,c.width,c.height);return}const W=m.width,H=m.height;if(c.width!==c.clientWidth*devicePixelRatio||c.height!==c.clientHeight*devicePixelRatio){c.width=c.clientWidth*devicePixelRatio;c.height=c.clientHeight*devicePixelRatio}const raw=atob(m.data),cells=new Int8Array(raw.length);for(let i=0;i<raw.length;i++)cells[i]=raw.charCodeAt(i)>127?raw.charCodeAt(i)-256:raw.charCodeAt(i);const off=document.createElement('canvas');off.width=W;off.height=H;const ox=off.getContext('2d'),im=ox.createImageData(W,H);for(let r=0;r<H;r++)for(let x=0;x<W;x++){let v=cells[r*W+x],j=((H-1-r)*W+x)*4;if(v<0){im.data[j]=23;im.data[j+1]=35;im.data[j+2]=45}else if(v>=50){im.data[j]=230;im.data[j+1]=93;im.data[j+2]=82}else{im.data[j]=220;im.data[j+1]=232;im.data[j+2]=218}im.data[j+3]=255}ox.putImageData(im,0,0);ctx.fillStyle='#0b1118';ctx.fillRect(0,0,c.width,c.height);ctx.imageSmoothingEnabled=false;ctx.drawImage(off,0,0,c.width,c.height);const sx=c.width/W,sy=c.height/H;const pix=p=>[(p.x-m.origin_x)/m.resolution*sx,c.height-(p.y-m.origin_y)/m.resolution*sy];function path(points,color,width){if(!points||points.length<2)return;ctx.strokeStyle=color;ctx.lineWidth=width*devicePixelRatio;ctx.beginPath();points.forEach((p,i)=>{let q=pix(p);if(i)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1])});ctx.stroke()}path(m.global_path,'#42d7e8',2);path(m.local_path,'#ffd166',3);ctx.fillStyle='#3ee8c1';for(const p of m.frontiers||[]){let q=pix(p);ctx.fillRect(q[0]-2,q[1]-2,4,4)}if(m.goal){let q=pix(m.goal);ctx.fillStyle='#3ee8c1';ctx.beginPath();ctx.arc(q[0],q[1],6*devicePixelRatio,0,Math.PI*2);ctx.fill()}if(s.pose){trail.push({x:s.pose.x,y:s.pose.y});if(trail.length>1500)trail.shift();path(trail,'#7b8dff',1.5)}if(s.pose){let q=pix(s.pose),scale=devicePixelRatio*8;ctx.save();ctx.translate(q[0],q[1]);ctx.rotate(-s.pose.yaw);ctx.fillStyle='#ffffff';ctx.beginPath();ctx.moveTo(scale,0);ctx.lineTo(-scale*.7,-scale*.58);ctx.lineTo(-scale*.7,scale*.58);ctx.closePath();ctx.fill();ctx.restore()}}
async function update(){try{let r=await fetch('/api/status',{cache:'no-store'}),s=await r.json();document.getElementById('system').textContent=s.system;document.getElementById('system').className='pill status '+(s.system==='ONLINE'?'good':s.system==='DEGRADED'?'warn':'bad');document.getElementById('domain').textContent=s.domain;document.getElementById('runtime').textContent=fmt(s.runtime,0)+' s';document.getElementById('simtime').textContent=s.sim_time===null?'—':fmt(s.sim_time,1)+' s';document.getElementById('mapstate').textContent='MAP: '+s.mapping.state;document.getElementById('mode').textContent=s.exploration.state;document.getElementById('frontierdist').textContent=s.exploration.distance===null?'—':fmt(s.exploration.distance,2)+' m';document.getElementById('frontiercount').textContent=s.exploration.frontier_count;document.getElementById('pathstatus').textContent=s.planner.global_path_status;document.getElementById('explored').textContent=fmt(s.map.known_percent,1)+'%';document.getElementById('target').textContent=s.exploration.goal?fmt(s.exploration.goal.x,2)+', '+fmt(s.exploration.goal.y,2):'—';document.getElementById('plannerstate').textContent=s.planner.state;document.getElementById('locsource').textContent=s.localization.source||'WAITING';document.getElementById('camerainfo').textContent=s.camera.resolution+' · '+fmt(s.camera.fps,1)+' FPS · '+age(s.camera.age);document.getElementById('travscore').textContent=fmt(s.traversability.mean_score,3);document.getElementById('terrain').textContent=(s.terrain.class||'—')+' · conf '+fmt(s.terrain.class_confidence,2);document.getElementById('geometry').textContent=fmt(s.terrain.slope_deg,1)+'° · '+fmt(s.terrain.roughness_m,3)+' m · '+fmt(s.terrain.geometry_confidence,2);document.getElementById('nearest').textContent=s.depth.nearest===null?'—':fmt(s.depth.nearest,2)+' m';document.getElementById('depthcount').textContent=s.depth.obstacle_samples+' / '+s.depth.finite+' ('+fmt(s.depth.valid_ratio*100,1)+'%)';document.getElementById('cloudrate').textContent=fmt(s.depth.rate_hz,1)+' Hz · '+age(s.depth.age);document.getElementById('xyzrange').textContent=s.depth.ranges;document.getElementById('cloudframe').textContent='Frame: '+(s.depth.frame||'—');document.getElementById('x').textContent=fmt(s.pose?.x,2)+' m';document.getElementById('y').textContent=fmt(s.pose?.y,2)+' m';document.getElementById('yaw').textContent=fmt(s.pose?.yaw,2)+' rad';document.getElementById('vx').textContent=fmt(s.pose?.vx,2)+' m/s';document.getElementById('wzage').textContent=fmt(s.pose?.wz,2)+' rad/s · '+age(s.pose?.age);document.getElementById('pose-source').textContent=s.localization.source||'—';document.getElementById('recommendedspeed').textContent=s.planner.recommended_speed===null?'—':fmt(s.planner.recommended_speed,3)+' m/s';document.getElementById('localstate').textContent=s.planner.local_state;document.getElementById('steering').textContent=fmt(s.planner.steering_deg,1)+'°';document.getElementById('obstacle').textContent=fmt(s.planner.clearance,2)+' m · '+fmt(s.planner.obstacle_score,2);document.getElementById('plannerdistance').textContent=s.exploration.distance===null?'—':fmt(s.exploration.distance,2)+' m';let sb=document.getElementById('safetybanner');sb.textContent='SAFETY: '+s.safety.state;sb.className='safety-banner '+(s.safety.state==='SAFE'?'':'stopped');document.getElementById('safetyreason').textContent=s.safety.reason;document.getElementById('command').textContent=fmt(s.command.vx,3)+' m/s · '+fmt(s.command.wz,3)+' rad/s';document.getElementById('commandage').textContent=age(s.command.age);document.getElementById('mapdims').textContent=s.map.width+'×'+s.map.height+' @ '+fmt(s.map.resolution,2)+' m';document.getElementById('cellcounts').textContent=s.map.known+' known · '+s.map.free+' free · '+s.map.occupied+' occupied · '+s.map.unknown+' unknown';document.getElementById('knownpct').textContent=fmt(s.map.known_percent,2)+'%';document.getElementById('health').innerHTML=Object.entries(s.health).map(([k,v])=>'<div class="health-item"><span class="label">'+k+'</span><b class="'+(v==='LIVE'?'good':(v==='STALE'||v==='DEGRADED')?'warn':'bad')+'">'+v+'</b></div>').join('');document.getElementById('events').innerHTML=s.events.map(e=>'<div class="event"><time>'+e.time+'</time>'+e.message+'</div>').join('')||'<div class="small">No runtime events received yet.</div>';document.getElementById('topics').innerHTML=s.topics.map(t=>'<tr><td>'+t.name+'</td><td>'+t.type+'</td><td>'+fmt(t.rate,1)+' Hz</td><td>'+age(t.age)+'</td><td class="'+(t.status==='LIVE'?'good':t.status==='STALE'?'warn':'bad')+'">'+t.status+'</td></tr>').join('');drawMap(s)}catch(e){document.getElementById('system').textContent='DASHBOARD API UNAVAILABLE'}}setInterval(update,1000);update();
</script></body></html>'''


class DashboardNode(Node):
    """Collect actual ROS runtime state for the browser UI."""

    def __init__(self):
        super().__init__('sih26126_dashboard')
        self.cloud_times = deque(maxlen=80)
        self.last_exploration = None
        self.last_safety = None
        self.last_steering = None
        self.last_obstacle_event = 0.0
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
        self.create_subscription(Int32, '/exploration/frontier_count', self.frontier_count_callback, 10)
        self.create_subscription(MarkerArray, '/exploration/frontiers', self.frontiers_callback, 5)
        self.create_subscription(Float32, '/exploration/explored_percentage', self.explored_callback, 10)
        self.create_subscription(Float32, '/exploration/distance_to_frontier', self.frontier_distance_callback, 10)
        self.create_subscription(PointStamped, '/exploration/goal', self.goal_callback, 10)
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
        self.create_timer(2.0, self.refresh_topics)
        event('Dashboard connected; awaiting live subsystem messages')

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
            event(f'RGB sensor live at {msg.width}×{msg.height}')

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
            event(f"Depth obstacle returns within {stats['nearest']:.2f} m")
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
            'origin_x': float(msg.info.origin.position.x), 'origin_y': float(msg.info.origin.position.y),
            'frame': msg.header.frame_id, 'data': base64.b64encode(data.tobytes()).decode('ascii'),
            'known': known_count, 'free': free, 'occupied': occupied, 'unknown': unknown,
            'known_percent': 100.0 * known_count / max(1, data.size), 'bounds': bounds,
        }
        with LOCK:
            previous = STATE['map']
            STATE['map'] = map_data
            STATE['map_last'] = time.time()
            self.mark('/map')
        if known_count and (previous is None or previous['known'] == 0):
            event('Online map building from live depth observations')
        elif previous and map_data['known_percent'] - previous['known_percent'] >= 2.0:
            event(f"Known map area grew to {map_data['known_percent']:.1f}%")

    def mapping_callback(self, msg):
        with LOCK:
            changed = STATE['mapping_state'] != msg.data
            STATE['mapping_state'] = msg.data
            self.mark('/mapping/state')
        if changed:
            event(f'Mapping state: {msg.data}')

    def exploration_callback(self, msg):
        with LOCK:
            changed = STATE['exploration_state'] != msg.data
            STATE['exploration_state'] = msg.data
            self.mark('/exploration/state')
        if changed:
            event(f'Exploration mode: {msg.data}')

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
            event(f"New map frontier target: ({p['x']:.1f}, {p['y']:.1f})")

    @staticmethod
    def path_points(msg):
        return [{'x': float(p.pose.position.x), 'y': float(p.pose.position.y)}
                for p in msg.poses]

    def global_path_callback(self, msg):
        with LOCK:
            STATE['global_path'] = self.path_points(msg)
            self.mark('/planner/global_path')

    def local_path_callback(self, msg):
        with LOCK:
            STATE['local_path'] = self.path_points(msg)
            self.mark('/planner/local_path')

    def path_status_callback(self, msg):
        with LOCK:
            STATE['global_path_status'] = msg.data
            self.mark('/planner/global_path_status')

    def planner_state_callback(self, msg):
        with LOCK:
            STATE['planner_state'] = msg.data
            self.mark('/planner/navigation_state')

    def recommended_speed_callback(self, msg):
        with LOCK:
            STATE['recommended_speed'] = float(msg.data)
            self.mark('/planner/recommended_speed')

    def local_state_callback(self, msg):
        with LOCK:
            STATE['local_state'] = msg.data
            self.mark('/planner/local_navigation_state')

    def steering_callback(self, msg):
        angle = float(msg.data)
        with LOCK:
            old = self.last_steering
            self.last_steering = angle
            STATE['steering_deg'] = angle
            self.mark('/planner/local_steering_angle')
        if old is not None and abs(angle - old) >= 8.0:
            event(f'Local steering changed to {angle:.1f}°')

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
            event(f'Safety state: {msg.data} ({reason})')

    def safety_reason_callback(self, msg):
        with LOCK:
            old = STATE['safety_reason']
            STATE['safety_reason'] = msg.data
            self.mark('/safety/reason')
            state = STATE['safety_state']
        if old != msg.data and state != 'SAFE':
            event(f'Safety stop reason: {msg.data}')

    def command_callback(self, msg):
        with LOCK:
            STATE['cmd_last'] = time.time()
            STATE['cmd'] = {'vx': float(msg.linear.x), 'wz': float(msg.angular.z)}
            self.mark('/cmd_vel')

    def refresh_topics(self):
        try:
            found = dict(self.get_topic_names_and_types())
        except Exception:
            return
        with LOCK:
            TOPIC_TYPES.clear()
            TOPIC_TYPES.update({k: (v[0] if v else 'unknown') for k, v in found.items()})


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
        # Rate uses real receive times maintained with the subscriber.
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
                             'known_percent': 0.0, 'bounds': None}
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
                '/planner/global_path', '/planner/local_path', '/safety/state',
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
            'samples': d.get('samples', 0), 'valid_ratio': d.get('finite', 0) / max(1, d.get('samples', 0)),
            'obstacle_samples': d.get('obstacle_samples', 0), 'ranges': ranges,
        }
        payload = {
            'system': system, 'domain': os.environ.get('ROS_DOMAIN_ID', '0'),
            'runtime': time.monotonic() - STARTED, 'sim_time': state['clock_sec'],
            'camera': {'resolution': f"{state['camera_width']}×{state['camera_height']}" if state['camera_width'] else '—',
                       'fps': fps, 'age': camera_age},
            'traversability': {'mean_score': state['mean_score'], 'age': trav_age},
            'terrain': {'class': state['trav_class'], 'class_confidence': state['class_confidence'],
                        'slope_deg': state['slope_mean'], 'roughness_m': state['roughness_mean'],
                        'geometry_confidence': state['geometry_confidence']},
            'depth': depth, 'pose': pose,
            'localization': {'source': state['localization_source']},
            'mapping': {'state': state['mapping_state'], 'age': map_age},
            'map': {**m, 'global_path': state['global_path'], 'local_path': state['local_path'],
                    'frontiers': state.get('frontiers', []), 'goal': state['goal']},
            'exploration': {'state': state['exploration_state'], 'frontier_count': state['frontier_count'],
                            'explored_percent': state['explored_percent'], 'distance': state['frontier_distance'],
                            'goal': state['goal']},
            'planner': {'global_path_status': state['global_path_status'], 'state': state['planner_state'],
                        'recommended_speed': state['recommended_speed'],
                        'local_state': state['local_state'], 'steering_deg': state['steering_deg'],
                        'clearance': state['clearance'], 'obstacle_score': state['obstacle_score']},
            'safety': {'state': state['safety_state'], 'reason': state['safety_reason']},
            'command': {**state['cmd'], 'age': cmd_age}, 'health': health,
            'events': list(EVENTS), 'topics': topics,
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
    print(f'SIH26126 live autonomy dashboard: http://localhost:{selected_port}', flush=True)
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
