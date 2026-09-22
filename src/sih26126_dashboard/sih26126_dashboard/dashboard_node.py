#!/usr/bin/env python3

import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


HOST = "0.0.0.0"
PORT = 8080
WORKSPACE = Path.home() / "sih26126_ws"

STATE = {
    "lock": threading.Lock(),

    "camera": None,
    "camera_count": 0,
    "camera_last": None,
    "camera_width": 0,
    "camera_height": 0,
    "camera_times": [],

    "trav": None,
    "trav_count": 0,
    "trav_last": None,
    "trav_width": 0,
    "trav_height": 0,

    "mean_score": None,

    "topics": {},
}


HTML = r"""
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SIH26126 Live Dashboard</title>

<style>
*{box-sizing:border-box}

body{
  margin:0;
  background:#0b1118;
  color:#e6edf3;
  font-family:Arial,Helvetica,sans-serif;
}

header{
  background:#111923;
  border-bottom:1px solid #303b48;
  padding:20px 28px;
  display:flex;
  align-items:center;
  justify-content:space-between;
}

h1{
  margin:0;
  font-size:27px;
}

.subtitle{
  margin-top:5px;
  color:#8b98a7;
}

.status{
  background:#238636;
  padding:9px 15px;
  border-radius:18px;
  font-weight:700;
}

main{
  max-width:1500px;
  margin:auto;
  padding:20px;
}

.grid{
  display:grid;
  grid-template-columns:repeat(12,1fr);
  gap:16px;
}

.card{
  background:#111923;
  border:1px solid #303b48;
  border-radius:10px;
  padding:16px;
}

.c4{grid-column:span 4}
.c6{grid-column:span 6}
.c8{grid-column:span 8}
.c12{grid-column:span 12}

h2{
  margin:0 0 14px;
  font-size:17px;
}

img{
  width:100%;
  display:block;
  background:#000;
  border-radius:8px;
  min-height:270px;
  object-fit:contain;
}

.row{
  display:flex;
  justify-content:space-between;
  padding:10px 0;
  border-bottom:1px solid #202b36;
}

.row:last-child{
  border-bottom:none;
}

.badge{
  padding:4px 9px;
  border-radius:12px;
  font-size:11px;
  font-weight:bold;
}

.ok{background:#238636}
.base{background:#9e6a03}
.gray{background:#57606a}
.red{background:#da3633}

.metric{
  font-size:34px;
  font-weight:800;
  margin:4px 0 12px;
}

.meta{
  color:#8b98a7;
  font-size:13px;
}

table{
  width:100%;
  border-collapse:collapse;
}

th,td{
  padding:9px;
  border-bottom:1px solid #202b36;
  text-align:left;
}

th{
  color:#8b98a7;
  font-size:12px;
  text-transform:uppercase;
}

.small{
  font-size:13px;
}

footer{
  padding:24px;
  text-align:center;
  color:#687584;
}

@media(max-width:1000px){
  .c4,.c6,.c8,.c12{
    grid-column:span 12;
  }
}
</style>
</head>

<body>

<header>
  <div>
    <h1>SIH26126 — Autonomous UGV</h1>
    <div class="subtitle">
      Live ROS 2 / Gazebo engineering dashboard
    </div>
  </div>

  <div id="system" class="status">STARTING</div>
</header>

<main>

<div class="grid">

<div class="card c4">
<h2>Build Status</h2>

<div class="row">
  <span>Custom UGV simulation</span>
  <span class="badge ok">COMPLETE</span>
</div>

<div class="row">
  <span>Camera pipeline</span>
  <span id="cameraBadge" class="badge gray">WAITING</span>
</div>

<div class="row">
  <span>Traversability baseline</span>
  <span id="travBadge" class="badge gray">WAITING</span>
</div>

<div class="row">
  <span>Obstacle benchmark</span>
  <span class="badge ok">COMPLETE</span>
</div>

<div class="row">
  <span>Localization / SLAM</span>
  <span class="badge gray">SCAFFOLD</span>
</div>

<div class="row">
  <span>Planner</span>
  <span class="badge gray">SCAFFOLD</span>
</div>

<div class="row">
  <span>Safety supervisor</span>
  <span class="badge gray">SCAFFOLD</span>
</div>

<div class="row">
  <span>Depth / geometry</span>
  <span class="badge base">NOT STARTED</span>
</div>
</div>


<div class="card c8">
<h2>Repository</h2>

<div class="row">
  <span>Branch</span>
  <strong id="branch">—</strong>
</div>

<div class="row">
  <span>Latest commit</span>
  <strong id="commit">—</strong>
</div>

<div class="row">
  <span>Repository state</span>
  <strong id="gitstate">—</strong>
</div>

<div class="row">
  <span>ROS distribution</span>
  <strong>Jazzy</strong>
</div>

<div class="row">
  <span>Simulator</span>
  <strong>Gazebo</strong>
</div>
</div>


<div class="card c6">
<h2>LIVE CAMERA — /camera</h2>

<img src="/camera.mjpg">

<p class="meta">
Resolution:
<span id="camres">—</span>
&nbsp; | &nbsp;
Frames:
<span id="camframes">0</span>
&nbsp; | &nbsp;
FPS:
<span id="camfps">0.0</span>
</p>
</div>


<div class="card c6">
<h2>LIVE TRAVERSABILITY — /traversability/score_map</h2>

<img src="/trav.mjpg">

<p class="meta">
Resolution:
<span id="travres">—</span>
&nbsp; | &nbsp;
Frames:
<span id="travframes">0</span>
</p>
</div>


<div class="card c4">
<h2>Traversability</h2>

<div class="meta">CURRENT MEAN SCORE</div>

<div class="metric" id="score">—</div>

<div class="row">
  <span>Camera age</span>
  <span id="camage">—</span>
</div>

<div class="row">
  <span>Score-map age</span>
  <span id="travage">—</span>
</div>
</div>


<div class="card c8">
<h2>ROS Topic Status</h2>

<table>
<thead>
<tr>
<th>Topic</th>
<th>Type</th>
<th>Status</th>
</tr>
</thead>

<tbody id="topics"></tbody>
</table>
</div>


<div class="card c12">
<h2>What Is Actually Built</h2>

<table>
<thead>
<tr>
<th>Subsystem</th>
<th>Status</th>
<th>Current implementation</th>
</tr>
</thead>

<tbody>

<tr>
<td>Simulation</td>
<td><span class="badge ok">COMPLETE</span></td>
<td>Custom 4-wheel UGV in Gazebo</td>
</tr>

<tr>
<td>RGB camera</td>
<td><span class="badge ok">COMPLETE</span></td>
<td>640×480 simulated RGB camera</td>
</tr>

<tr>
<td>ROS-Gazebo bridge</td>
<td><span class="badge ok">COMPLETE</span></td>
<td>Camera / camera_info bridge</td>
</tr>

<tr>
<td>Perception</td>
<td><span class="badge base">BASELINE</span></td>
<td>Deterministic image-space processing</td>
</tr>

<tr>
<td>Traversability</td>
<td><span class="badge base">BASELINE</span></td>
<td>Full-frame mono8 traversability score map</td>
</tr>

<tr>
<td>Obstacle benchmark</td>
<td><span class="badge ok">COMPLETE</span></td>
<td>Controlled obstacle + measured response</td>
</tr>

<tr>
<td>Localization</td>
<td><span class="badge gray">SCAFFOLD</span></td>
<td>Package exists; autonomy logic not implemented</td>
</tr>

<tr>
<td>Planner</td>
<td><span class="badge gray">SCAFFOLD</span></td>
<td>Package exists; route generation not implemented</td>
</tr>

<tr>
<td>Safety</td>
<td><span class="badge gray">SCAFFOLD</span></td>
<td>Package exists; supervisory logic not implemented</td>
</tr>

<tr>
<td>Depth / geometry</td>
<td><span class="badge base">NOT STARTED</span></td>
<td>Next major engineering subsystem</td>
</tr>

</tbody>
</table>
</div>

</div>

</main>

<footer>
SIH26126 • Live read-only engineering dashboard
</footer>


<script>

function setBadge(id,live){
  const e=document.getElementById(id);

  if(live){
    e.textContent="LIVE";
    e.className="badge ok";
  }else{
    e.textContent="WAITING";
    e.className="badge gray";
  }
}

function age(x){
  if(x===null) return "—";
  return x.toFixed(1)+" s";
}

async function update(){

  try{

    const r=await fetch("/api/status");
    const s=await r.json();

    document.getElementById("system").textContent =
      s.system_live ? "SYSTEM LIVE" : "WAITING FOR ROS";

    setBadge("cameraBadge",s.camera.live);
    setBadge("travBadge",s.traversability.live);

    document.getElementById("camres").textContent =
      s.camera.resolution;

    document.getElementById("camframes").textContent =
      s.camera.frames;

    document.getElementById("camfps").textContent =
      s.camera.fps.toFixed(1);

    document.getElementById("travres").textContent =
      s.traversability.resolution;

    document.getElementById("travframes").textContent =
      s.traversability.frames;

    document.getElementById("score").textContent =
      s.traversability.mean_score === null
        ? "—"
        : s.traversability.mean_score.toFixed(3);

    document.getElementById("camage").textContent =
      age(s.camera.age);

    document.getElementById("travage").textContent =
      age(s.traversability.age);

    document.getElementById("branch").textContent =
      s.git.branch;

    document.getElementById("commit").textContent =
      s.git.commit;

    document.getElementById("gitstate").textContent =
      s.git.clean ? "CLEAN" : "MODIFIED";

    const list=[
      "/camera",
      "/camera_info",
      "/traversability/score_map",
      "/traversability/mean_score",
      "/clock",
      "/imu",
      "/cmd_vel"
    ];

    document.getElementById("topics").innerHTML =
      list.map(function(name){

        const type=s.topics[name];

        return `
        <tr>
          <td>${name}</td>
          <td class="small">${type || "—"}</td>
          <td>
            <span class="badge ${type ? "ok" : "gray"}">
              ${type ? "LIVE" : "OFFLINE"}
            </span>
          </td>
        </tr>
        `;

      }).join("");

  }catch(e){

    document.getElementById("system").textContent =
      "DASHBOARD SERVER ONLY";

  }
}

setInterval(update,1000);
update();

</script>

</body>
</html>
"""


def jpeg_camera(msg):
    if msg.encoding not in ("rgb8", "bgr8"):
        return None

    raw = np.frombuffer(msg.data, dtype=np.uint8)

    needed = msg.width * msg.height * 3

    if raw.size < needed:
        return None

    image = raw[:needed].reshape(
        msg.height,
        msg.width,
        3,
    )

    if msg.encoding == "rgb8":
        image = cv2.cvtColor(
            image,
            cv2.COLOR_RGB2BGR,
        )

    ok, data = cv2.imencode(
        ".jpg",
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), 82],
    )

    return data.tobytes() if ok else None


def jpeg_traversability(msg):
    if msg.encoding != "mono8":
        return None

    raw = np.frombuffer(msg.data, dtype=np.uint8)

    needed = msg.width * msg.height

    if raw.size < needed:
        return None

    image = raw[:needed].reshape(
        msg.height,
        msg.width,
    )

    # Apply a visible heatmap for the dashboard.
    color = cv2.applyColorMap(
        image,
        cv2.COLORMAP_TURBO,
    )

    # Current baseline intentionally leaves unknown
    # pixels at zero. Keep them visibly dark.
    color[image == 0] = 0

    ok, data = cv2.imencode(
        ".jpg",
        color,
        [int(cv2.IMWRITE_JPEG_QUALITY), 82],
    )

    return data.tobytes() if ok else None


def git_state():
    result = {
        "branch": "—",
        "commit": "—",
        "clean": False,
    }

    try:
        result["branch"] = subprocess.check_output(
            [
                "git",
                "-C",
                str(WORKSPACE),
                "branch",
                "--show-current",
            ],
            text=True,
            timeout=2,
        ).strip() or "—"

        result["commit"] = subprocess.check_output(
            [
                "git",
                "-C",
                str(WORKSPACE),
                "rev-parse",
                "--short",
                "HEAD",
            ],
            text=True,
            timeout=2,
        ).strip() or "—"

        dirty = subprocess.check_output(
            [
                "git",
                "-C",
                str(WORKSPACE),
                "status",
                "--porcelain",
            ],
            text=True,
            timeout=2,
        ).strip()

        result["clean"] = not bool(dirty)

    except Exception:
        pass

    return result


class DashboardNode(Node):

    def __init__(self):

        super().__init__("sih26126_dashboard")

        self.create_subscription(
            Image,
            "/camera",
            self.camera_callback,
            10,
        )

        self.create_subscription(
            Image,
            "/traversability/score_map",
            self.trav_callback,
            10,
        )

        self.create_subscription(
            Float32,
            "/traversability/mean_score",
            self.mean_callback,
            10,
        )

        self.create_timer(
            2.0,
            self.refresh_topics,
        )

        self.refresh_topics()

    def camera_callback(self, msg):

        frame = jpeg_camera(msg)

        if frame is None:
            return

        now = time.time()

        with STATE["lock"]:

            STATE["camera"] = frame
            STATE["camera_count"] += 1
            STATE["camera_last"] = now

            STATE["camera_width"] = msg.width
            STATE["camera_height"] = msg.height

            STATE["camera_times"].append(now)

            cutoff = now - 10.0

            STATE["camera_times"] = [
                t
                for t in STATE["camera_times"]
                if t >= cutoff
            ]

    def trav_callback(self, msg):

        frame = jpeg_traversability(msg)

        if frame is None:
            return

        with STATE["lock"]:

            STATE["trav"] = frame
            STATE["trav_count"] += 1
            STATE["trav_last"] = time.time()

            STATE["trav_width"] = msg.width
            STATE["trav_height"] = msg.height

    def mean_callback(self, msg):

        with STATE["lock"]:
            STATE["mean_score"] = float(msg.data)

    def refresh_topics(self):

        wanted = {
            "/camera",
            "/camera_info",
            "/traversability/score_map",
            "/traversability/mean_score",
            "/clock",
            "/imu",
            "/cmd_vel",
        }

        found = {}

        try:
            for name, types in self.get_topic_names_and_types():
                if name in wanted:
                    found[name] = (
                        types[0] if types else "unknown"
                    )
        except Exception:
            pass

        with STATE["lock"]:
            STATE["topics"] = found


class Handler(BaseHTTPRequestHandler):

    def log_message(self, *_):
        pass

    def send_bytes(self, data, content_type):

        self.send_response(200)

        self.send_header(
            "Content-Type",
            content_type,
        )

        self.send_header(
            "Content-Length",
            str(len(data)),
        )

        self.end_headers()

        self.wfile.write(data)

    def do_GET(self):

        if self.path == "/":

            self.send_bytes(
                HTML.encode(),
                "text/html; charset=utf-8",
            )

            return

        if self.path == "/api/status":

            with STATE["lock"]:

                now = time.time()

                camera_age = (
                    None
                    if STATE["camera_last"] is None
                    else now - STATE["camera_last"]
                )

                trav_age = (
                    None
                    if STATE["trav_last"] is None
                    else now - STATE["trav_last"]
                )

                times = STATE["camera_times"]

                if len(times) >= 2:

                    duration = (
                        times[-1]
                        - times[0]
                    )

                    fps = (
                        (len(times) - 1) / duration
                        if duration > 0
                        else 0.0
                    )

                else:
                    fps = 0.0

                payload = {

                    "system_live": (
                        camera_age is not None
                        and camera_age < 3
                    ),

                    "camera": {

                        "live": (
                            camera_age is not None
                            and camera_age < 3
                        ),

                        "age": camera_age,

                        "frames": STATE["camera_count"],

                        "fps": fps,

                        "resolution": (
                            f'{STATE["camera_width"]} × '
                            f'{STATE["camera_height"]}'
                            if STATE["camera_width"]
                            else "—"
                        ),
                    },

                    "traversability": {

                        "live": (
                            trav_age is not None
                            and trav_age < 3
                        ),

                        "age": trav_age,

                        "frames": STATE["trav_count"],

                        "resolution": (
                            f'{STATE["trav_width"]} × '
                            f'{STATE["trav_height"]}'
                            if STATE["trav_width"]
                            else "—"
                        ),

                        "mean_score": STATE["mean_score"],
                    },

                    "topics": dict(STATE["topics"]),

                    "git": git_state(),
                }

            self.send_bytes(
                json.dumps(payload).encode(),
                "application/json",
            )

            return

        if self.path == "/camera.mjpg":
            self.stream("camera")
            return

        if self.path == "/trav.mjpg":
            self.stream("trav")
            return

        self.send_response(404)
        self.end_headers()

    def stream(self, stream_name):

        self.send_response(200)

        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )

        self.send_header(
            "Cache-Control",
            "no-cache, no-store",
        )

        self.end_headers()

        previous = None

        try:

            while True:

                with STATE["lock"]:
                    frame = STATE[stream_name]

                if frame is not None and frame is not previous:

                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: "
                        + str(len(frame)).encode()
                        + b"\r\n\r\n"
                    )

                    self.wfile.write(
                        header + frame + b"\r\n"
                    )

                    self.wfile.flush()

                    previous = frame

                time.sleep(0.03)

        except (
            BrokenPipeError,
            ConnectionResetError,
        ):
            pass


def main():

    rclpy.init()

    node = DashboardNode()

    server = ThreadingHTTPServer(
        (HOST, PORT),
        Handler,
    )

    thread = threading.Thread(
        target=server.serve_forever,
        daemon=True,
    )

    thread.start()

    print()
    print("============================================================")
    print(" SIH26126 LIVE DASHBOARD")
    print("============================================================")
    print()
    print("Server: http://0.0.0.0:8080")
    print("Browser: http://localhost:8080")
    print()
    print("Dashboard is READ-ONLY.")
    print("It does not control Gazebo or the UGV.")
    print()
    print("Keep this terminal running.")
    print("Press Ctrl+C here to stop the dashboard.")
    print()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        server.shutdown()
        server.server_close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
