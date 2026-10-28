#!/usr/bin/env python3
"""Managed Gazebo GUI streaming subprocess group.

Starts four child processes in order and supervises them:
  1. Xvfb      — virtual framebuffer on DISPLAY :99 (isolated from :0)
  2. gz sim -g — Gazebo GUI-only client on :99, connects to running server
  3. x11vnc    — VNC server capturing :99, listening on localhost only
  4. websockify — WebSocket-to-VNC bridge + noVNC static file server

All processes are confined to localhost.  No VNC port is exposed to the
network (x11vnc binds 127.0.0.1 only).

The module exposes a single ``GzGuiStream`` class with ``start()`` /
``stop()`` / ``is_alive()`` methods, designed to be driven from a ROS node
or the dashboard process.
"""

import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import List, Optional

# ── tuneable constants ──────────────────────────────────────────────────────
VIRTUAL_DISPLAY = ':99'
VNC_PORT = 5900           # x11vnc listens here (localhost only)
WEBSOCKET_PORT = 6080     # websockify / noVNC WebSocket endpoint
DISPLAY_RESOLUTION = '1280x800x24'
XVFB_READY_TIMEOUT_S = 6.0   # seconds to wait for Xvfb to be ready
GZ_START_DELAY_S = 3.0        # seconds after Xvfb before launching gz sim -g
VNC_READY_TIMEOUT_S = 10.0    # seconds to wait for x11vnc TCP port
WS_START_DELAY_S = 1.5        # seconds after x11vnc before launching websockify

# Locations to search for websockify binary (pip --user install lands in ~/.local/bin)
_WEBSOCKIFY_CANDIDATES = [
    shutil.which('websockify'),
    os.path.expanduser('~/.local/bin/websockify'),
    '/usr/bin/websockify',
    '/usr/local/bin/websockify',
]


def _find(binary: str) -> Optional[str]:
    """Return full path of *binary* or None if not found."""
    # Check PATH first
    found = shutil.which(binary)
    if found:
        return found
    # Special-case websockify which may be in ~/.local/bin (pip --user)
    if binary == 'websockify':
        for candidate in _WEBSOCKIFY_CANDIDATES:
            if candidate and os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
    return None


def _required(binary: str) -> str:
    """Return full path or raise RuntimeError with install hint."""
    path = _find(binary)
    if path is None:
        hints = {
            'Xvfb':       'sudo apt-get install -y xvfb',
            'x11vnc':     'sudo apt-get install -y x11vnc',
            'websockify': (
                'sudo apt-get install -y websockify  '
                'OR  pip3 install --user --break-system-packages websockify'
            ),
        }
        hint = hints.get(binary, f'install {binary}')
        raise RuntimeError(
            f'Required binary not found: {binary!r}.  '
            f'Install it with:  {hint}'
        )
    return path


def _novnc_static_dir() -> Optional[str]:
    """Return the noVNC static file directory if the package is installed."""
    candidates = [
        '/usr/share/novnc',
    ]
    for path in candidates:
        if os.path.isdir(path) and os.path.isfile(os.path.join(path, 'vnc_lite.html')):
            return path
    # Check python3-novnc wheel location
    try:
        import novnc  # type: ignore
        d = os.path.dirname(novnc.__file__)
        if os.path.isdir(d) and os.path.isfile(os.path.join(d, 'vnc_lite.html')):
            return d
    except ImportError:
        pass
    return None


def _port_in_use(port: int) -> bool:
    """Return True if something is already listening on localhost:port."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex(('127.0.0.1', port)) == 0


def _wait_for_port(port: int, timeout: float, interval: float = 0.3) -> bool:
    """Poll until localhost:port accepts connections or timeout expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _port_in_use(port):
            return True
        time.sleep(interval)
    return False


def _xvfb_display_exists(display: str) -> bool:
    """Return True if the X display is accepting connections."""
    # Use xdpyinfo if available; fall back to a raw socket check on
    # the abstract Unix socket that Xvfb creates.
    xdpyinfo = shutil.which('xdpyinfo')
    if xdpyinfo:
        result = subprocess.run(
            [xdpyinfo, '-display', display],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=3.0,
        )
        return result.returncode == 0
    # Fallback: check that Xvfb created its lock file
    disp_num = display.lstrip(':')
    return os.path.exists(f'/tmp/.X{disp_num}-lock')


class GzGuiStream:
    """Lifecycle manager for the virtual-display Gazebo GUI streaming stack."""

    def __init__(
        self,
        gz_binary: str = 'gz',
        virtual_display: str = VIRTUAL_DISPLAY,
        vnc_port: int = VNC_PORT,
        websocket_port: int = WEBSOCKET_PORT,
        resolution: str = DISPLAY_RESOLUTION,
        logger=None,
    ):
        self._gz_binary = gz_binary
        self._display = virtual_display
        self._vnc_port = vnc_port
        self._ws_port = websocket_port
        self._resolution = resolution
        self._log = logger or _default_logger()
        self._procs: List[subprocess.Popen] = []
        self._lock = threading.Lock()
        self._running = False
        self._monitor_thread: Optional[threading.Thread] = None

    # ── public API ──────────────────────────────────────────────────────────

    def start(self) -> None:
        """Start the streaming stack.  Raises RuntimeError if deps are missing."""
        with self._lock:
            if self._running:
                return
            self._validate_deps()
            self._check_port_conflicts()
            self._launch()
            self._running = True
            self._monitor_thread = threading.Thread(
                target=self._monitor, daemon=True)
            self._monitor_thread.start()

    def stop(self) -> None:
        """Stop all child processes gracefully, then forcefully."""
        with self._lock:
            self._running = False
            self._terminate_all()

    def is_alive(self) -> bool:
        """Return True if all expected child processes are still running."""
        with self._lock:
            return self._running and bool(self._procs) and all(
                p.poll() is None for p in self._procs)

    @property
    def websocket_port(self) -> int:
        return self._ws_port

    @property
    def vnc_port(self) -> int:
        return self._vnc_port

    # ── dependency validation ───────────────────────────────────────────────

    def _validate_deps(self) -> None:
        missing = []
        for binary in ('Xvfb', 'x11vnc', 'websockify'):
            if _find(binary) is None:
                missing.append(binary)
        if missing:
            raise RuntimeError(
                f'Gazebo GUI streaming requires these missing packages: '
                f'{", ".join(missing)}.  '
                f'Install with:  sudo apt-get install -y xvfb x11vnc websockify novnc'
                f'  OR for websockify only: '
                f'pip3 install --user --break-system-packages websockify'
            )

    # ── port conflict detection ─────────────────────────────────────────────

    def _check_port_conflicts(self) -> None:
        """Raise if either required port is already occupied."""
        if _port_in_use(self._vnc_port):
            raise RuntimeError(
                f'VNC port {self._vnc_port} is already in use.  '
                f'Pass a different vnc_port= to GzGuiStream, or stop the '
                f'conflicting process first.'
            )
        if _port_in_use(self._ws_port):
            raise RuntimeError(
                f'WebSocket port {self._ws_port} is already in use.  '
                f'Pass a different websocket_port= to GzGuiStream, or stop '
                f'the conflicting process first.'
            )

    # ── process launching ───────────────────────────────────────────────────

    def _launch(self) -> None:
        """Launch child processes in dependency order with readiness checks."""
        self._procs.clear()

        # 1. Xvfb ── virtual framebuffer
        self._log(f'[gz_gui_stream] Starting Xvfb on {self._display} ({self._resolution})')
        xvfb = _required('Xvfb')
        proc_xvfb = subprocess.Popen(
            [xvfb, self._display, '-screen', '0', self._resolution,
             '-ac', '+extension', 'GLX', '+render', '-noreset'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self._procs.append(proc_xvfb)

        # Wait until Xvfb is ready (lock file appears or xdpyinfo succeeds)
        deadline = time.monotonic() + XVFB_READY_TIMEOUT_S
        ready = False
        while time.monotonic() < deadline:
            if proc_xvfb.poll() is not None:
                raise RuntimeError(
                    f'Xvfb exited immediately (code={proc_xvfb.returncode}).  '
                    f'Display {self._display} may already be in use.'
                )
            if _xvfb_display_exists(self._display):
                ready = True
                break
            time.sleep(0.4)
        if not ready:
            self._log(
                f'[gz_gui_stream] WARNING: Xvfb readiness check timed out '
                f'({XVFB_READY_TIMEOUT_S}s); continuing anyway.')
        else:
            self._log(f'[gz_gui_stream] Xvfb ready on {self._display}')

        # 2. gz sim -g ── GUI-only client, attaches to the already-running server
        self._log('[gz_gui_stream] Starting gz sim -g on virtual display')
        gz_env = dict(os.environ)
        # Force X11 / Xvfb display — must be set before Qt initialises.
        gz_env['DISPLAY'] = self._display
        # Remove WAYLAND_DISPLAY so Qt does not attempt a Wayland session on
        # Xvfb.  Without this Qt/QPA picks the Wayland backend, which has no
        # compositor on display :99 and causes an immediate SIGABRT (code -6).
        gz_env.pop('WAYLAND_DISPLAY', None)
        # Force Qt to use the XCB (X11) platform plugin instead of
        # auto-detecting Wayland from the parent environment.
        gz_env['QT_QPA_PLATFORM'] = 'xcb'
        # Disable DRI3 — DRI3 over a software Mesa/Xvfb path causes the
        # GL context creation to fail in WSL2, triggering the texture-budget
        # stall and subsequent OGRE shutdown we see in ogre2.log.
        gz_env['LIBGL_DRI3_DISABLE'] = '1'
        # Select the classic OGRE (ogre1) render engine for the GUI only.
        # Verified via `gz sim --help`: the documented flag is
        #   --render-engine-gui [arg]   (default: ogre2)
        # GZ_SIM_RENDER_ENGINE is NOT listed as a supported env var and is
        # therefore NOT used here.  The ogre plugin is confirmed installed at:
        #   /opt/ros/jazzy/opt/gz_rendering_vendor/lib/gz-rendering-8/
        #   engine-plugins/libgz-rendering8-ogre.so
        # ogre (classic) uses the OGRE 1.x GL renderer, which is substantially
        # lighter on GPU memory than ogre2/GL3Plus and avoids the texture-budget
        # exhaustion and SIGABRT we observed in ogre2.log on WSL2/Xvfb.
        sim_share = _find_gz_sim_share()
        if sim_share:
            gz_env.setdefault('GZ_SIM_RESOURCE_PATH', f'{sim_share}/models')

        # Redirect GUI child's output to a log file so crashes are visible.
        # The file is opened in append mode so successive restarts accumulate.
        _gz_log_path = '/tmp/drishti_gz_gui.log'
        _gz_log_fd = open(_gz_log_path, 'a')  # noqa: WPS515 – lifecycle tied to proc
        self._log(f'[gz_gui_stream] gz sim -g stdout/stderr -> {_gz_log_path}')

        proc_gz = subprocess.Popen(
            [self._gz_binary, 'sim', '-g', '--render-engine-gui', 'ogre'],
            env=gz_env,
            stdout=_gz_log_fd,
            stderr=_gz_log_fd,
            start_new_session=True,
        )
        self._procs.append(proc_gz)
        # Allow gz to initialise its GUI before capturing
        time.sleep(GZ_START_DELAY_S)
        if proc_gz.poll() is not None:
            self._log(
                f'[gz_gui_stream] WARNING: gz sim -g exited quickly '
                f'(code={proc_gz.returncode}).  '
                f'The panel will show a blank screen until gz starts.')

        # 3. x11vnc ── capture virtual display, bind localhost only
        self._log(f'[gz_gui_stream] Starting x11vnc on localhost:{self._vnc_port}')
        x11vnc = _required('x11vnc')
        # Build x11vnc environment: copy parent env but:
        #  - Set DISPLAY to the virtual framebuffer (not the desktop)
        #  - Unset WAYLAND_DISPLAY so x11vnc doesn't refuse with
        #    "Wayland sessions are only supported via -rawfb"
        x11vnc_env = dict(os.environ)
        x11vnc_env['DISPLAY'] = self._display
        x11vnc_env.pop('WAYLAND_DISPLAY', None)
        proc_vnc = subprocess.Popen(
            [x11vnc,
             '-display', self._display,
             '-rfbport', str(self._vnc_port),
             '-listen', '127.0.0.1',   # bind localhost only — not network-exposed
             '-nopw',                   # no VNC password (only websockify bridge reaches it)
             '-forever',                # keep running after first disconnect
             '-shared',                 # allow multiple simultaneous clients
             '-noxdamage',              # more reliable on Xvfb
             '-noxfixes',
             '-quiet',
             # NOTE: do NOT use -bg here; daemonising changes the PID so the
             # monitor thread would see a false "process exited" and stop the stream.
             ],
            env=x11vnc_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self._procs.append(proc_vnc)

        # Wait until x11vnc is actually listening before starting websockify
        if not _wait_for_port(self._vnc_port, VNC_READY_TIMEOUT_S):
            self._log(
                f'[gz_gui_stream] WARNING: x11vnc did not open port '
                f'{self._vnc_port} within {VNC_READY_TIMEOUT_S}s; '
                f'websockify may fail to connect.')
        else:
            self._log(f'[gz_gui_stream] x11vnc listening on localhost:{self._vnc_port}')

        # 4. websockify ── WebSocket bridge + noVNC static files
        self._log(
            f'[gz_gui_stream] Starting websockify '
            f'localhost:{self._ws_port} -> 127.0.0.1:{self._vnc_port}')
        websockify = _required('websockify')
        novnc_dir = _novnc_static_dir()

        ws_cmd = [websockify, '--web', novnc_dir] if novnc_dir else [websockify]
        ws_cmd += [
            f'127.0.0.1:{self._ws_port}',  # bind localhost only — noVNC JS connects from browser via window.location.hostname
            f'127.0.0.1:{self._vnc_port}',
        ]

        proc_ws = subprocess.Popen(
            ws_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self._procs.append(proc_ws)
        time.sleep(WS_START_DELAY_S)

        if proc_ws.poll() is not None:
            raise RuntimeError(
                f'websockify exited immediately (code={proc_ws.returncode}).  '
                f'Check that port {self._ws_port} is free.'
            )

        # Verify websockify is accepting connections
        if _wait_for_port(self._ws_port, 3.0):
            self._log(
                f'[gz_gui_stream] Gazebo GUI stream ready.  '
                f'noVNC WebSocket: ws://localhost:{self._ws_port}  '
                f'noVNC UI: http://localhost:{self._ws_port}/vnc_lite.html'
            )
        else:
            self._log(
                f'[gz_gui_stream] WARNING: websockify port {self._ws_port} '
                f'not yet reachable; stream may not connect immediately.')

    # ── monitoring ──────────────────────────────────────────────────────────

    def _monitor(self) -> None:
        """Background thread: log and stop if a child exits unexpectedly."""
        _names = ['Xvfb', 'gz-sim-g', 'x11vnc', 'websockify']
        while True:
            time.sleep(5.0)
            with self._lock:
                if not self._running:
                    return
                dead = [(i, p) for i, p in enumerate(self._procs)
                        if p.poll() is not None]
            if dead:
                for i, p in dead:
                    name = _names[i] if i < len(_names) else f'proc[{i}]'
                    self._log(
                        f'[gz_gui_stream] WARNING: {name} exited unexpectedly '
                        f'(code={p.returncode}).  Stopping GUI stream.')
                self.stop()
                return

    # ── teardown ────────────────────────────────────────────────────────────

    def _terminate_all(self) -> None:
        """Send SIGTERM to the process group of each child, then SIGKILL."""
        for proc in reversed(self._procs):
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except (ProcessLookupError, PermissionError):
                    try:
                        proc.terminate()
                    except ProcessLookupError:
                        pass
        deadline = time.monotonic() + 4.0
        for proc in reversed(self._procs):
            remaining = max(0.0, deadline - time.monotonic())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    try:
                        proc.kill()
                    except ProcessLookupError:
                        pass
        self._procs.clear()


# ── helpers ─────────────────────────────────────────────────────────────────

def _default_logger():
    def _log(msg: str) -> None:
        print(msg, file=sys.stderr, flush=True)
    return _log


def _find_gz_sim_share() -> Optional[str]:
    """Best-effort lookup of the sih26126_sim share directory."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('sih26126_sim')
    except Exception:
        return None
