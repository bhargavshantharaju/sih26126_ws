#!/usr/bin/env bash
# Copyright 2026 DRISTHI Contributors
# Licensed under the Apache License, Version 2.0
#
# DRISTHI One-Command Launcher
#
# Normal usage:
#   cd ~/sih26126_ws && ./run_dristi.sh
#
# Options:
#   --gui         Show Gazebo GUI (default: headless)
#   --no-dashboard   Skip dashboard (default: start dashboard)
#   --rebuild     Force a clean rebuild before launch
#   --world PATH  Use custom Gazebo world SDF (default: showcase_terrain.sdf)
#

set -e  # Exit on any error

# ============================================================================
# Configuration
# ============================================================================
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="jazzy"
DEFAULT_WORLD="showcase_terrain"  # Default world (without path/extension)

# Launch parameters (overridable via command-line args)
SHOW_GUI="false"
START_DASHBOARD="true"
FORCE_REBUILD="false"
CUSTOM_WORLD=""

# ============================================================================
# Argument Parsing
# ============================================================================
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)
      SHOW_GUI="true"
      shift
      ;;
    --no-dashboard)
      START_DASHBOARD="false"
      shift
      ;;
    --rebuild)
      FORCE_REBUILD="true"
      shift
      ;;
    --world)
      CUSTOM_WORLD="$2"
      shift 2
      ;;
    --help|-h)
      cat <<EOF
DRISTHI One-Command Launcher

Usage:
  ./run_dristi.sh [OPTIONS]

Options:
  --gui              Show Gazebo GUI (default: headless server)
  --no-dashboard     Skip dashboard startup
  --rebuild          Force rebuild before launch
  --world PATH       Custom Gazebo world SDF path
  -h, --help         Show this help

Examples:
  ./run_dristi.sh
  ./run_dristi.sh --gui
  ./run_dristi.sh --world src/sih26126_sim/worlds/forest_hills.sdf

Environment:
  ROS 2 ${ROS_DISTRO} and Gazebo Harmonic required.
  WSL2 users: use headless mode (default).
EOF
      exit 0
      ;;
    *)
      echo "ERROR: Unknown argument '$1'. Use --help for usage." >&2
      exit 1
      ;;
  esac
done

# ============================================================================
# Pre-Flight Checks
# ============================================================================
echo "==> DRISTHI Launcher: Pre-flight checks"

# Check we're in the workspace root
if [[ ! -f "${WORKSPACE_ROOT}/src/sih26126_bringup/package.xml" ]]; then
  echo "ERROR: Must run from workspace root (~/sih26126_ws)" >&2
  echo "Current directory: ${WORKSPACE_ROOT}" >&2
  exit 1
fi

# Check ROS 2 Jazzy is available
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ERROR: ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}" >&2
  exit 1
fi

# Check Gazebo is available
if ! command -v gz &>/dev/null; then
  echo "ERROR: Gazebo 'gz' command not found. Install Gazebo Harmonic." >&2
  exit 1
fi

# ============================================================================
# Source ROS 2 and Workspace
# ============================================================================
echo "==> Sourcing ROS 2 ${ROS_DISTRO}"
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# ============================================================================
# Build Workspace (if needed or forced)
# ============================================================================
if [[ "${FORCE_REBUILD}" == "true" ]] || [[ ! -d "${WORKSPACE_ROOT}/install" ]]; then
  echo "==> Building workspace (this may take a few minutes)..."
  cd "${WORKSPACE_ROOT}"
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

  if [[ $? -ne 0 ]]; then
    echo "ERROR: Workspace build failed. Fix build errors before launching." >&2
    exit 1
  fi
  echo "==> Build complete"
else
  echo "==> Skipping build (install/ exists; use --rebuild to force)"
fi

# Source the workspace overlay
echo "==> Sourcing workspace overlay"
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"

# ============================================================================
# Prepare Launch Arguments
# ============================================================================
LAUNCH_ARGS=(
  "slam:=true"
  "fusion:=true"
  "gui:=${SHOW_GUI}"
  "start_dashboard:=${START_DASHBOARD}"
  "scenario:=showcase"
)

if [[ -n "${CUSTOM_WORLD}" ]]; then
  # Resolve to absolute path if relative
  if [[ "${CUSTOM_WORLD}" != /* ]]; then
    CUSTOM_WORLD="${WORKSPACE_ROOT}/${CUSTOM_WORLD}"
  fi
  
  if [[ ! -f "${CUSTOM_WORLD}" ]]; then
    echo "ERROR: Custom world file not found: ${CUSTOM_WORLD}" >&2
    exit 1
  fi
  
  LAUNCH_ARGS+=("world:=${CUSTOM_WORLD}")
  echo "==> Using custom world: ${CUSTOM_WORLD}"
fi

# ============================================================================
# Check for Conflicting Processes
# ============================================================================
echo "==> Checking for conflicting DRISTHI processes"

# Check if another full_stack.launch.py is already running
if pgrep -f "full_stack.launch.py" >/dev/null; then
  echo ""
  echo "WARNING: Another DRISTHI full_stack launch appears to be running."
  echo "         This may cause topic/service conflicts."
  echo ""
  # Detect whether we have a controlling terminal for interactive input.
  if [ -t 0 ]; then
    printf "Stop existing process and continue? [y/N] "
    read -r REPLY
  else
    # Non-TTY context (CI, subprocess, pipe): fail safe and abort.
    echo "Non-interactive context detected — aborting to avoid duplicate stacks."
    echo "Stop the existing process manually or use a different ROS_DOMAIN_ID."
    exit 1
  fi
  if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "==> Stopping existing full_stack processes..."
    pkill -SIGTERM -f "full_stack.launch.py" || true
    # Allow up to 8 seconds for Gazebo (a child of ros2 launch) to terminate.
    # Gazebo can take 5-10 seconds to exit cleanly; a 2-second wait was too
    # short and could leave a Gazebo instance running while a new one starts.
    echo "==> Waiting for processes to exit (up to 8 seconds)..."
    for i in {1..8}; do
      sleep 1
      if ! pgrep -f "gz sim" >/dev/null 2>&1; then
        break
      fi
    done
    # Force-kill any remaining Gazebo process that did not respond to SIGTERM.
    if pgrep -f "gz sim" >/dev/null 2>&1; then
      echo "==> Gazebo did not exit gracefully; sending SIGKILL..."
      pkill -SIGKILL -f "gz sim" || true
      sleep 1
    fi
  else
    echo "Aborting launch. Stop the existing process manually or use a different ROS_DOMAIN_ID."
    exit 1
  fi
fi

# ============================================================================
# Launch Full Stack
# ============================================================================
echo ""
echo "========================================================================"
echo "  DRISTHI Full Stack Launch"
echo "========================================================================"
echo "  Configuration:"
echo "    SLAM:       true (slam_toolbox + EKF sensor fusion)"
echo "    GUI:        ${SHOW_GUI}"
echo "    Dashboard:  ${START_DASHBOARD} (http://localhost:8080)"
echo "    World:      ${CUSTOM_WORLD:-Default showcase terrain}"
echo "========================================================================"
echo ""
echo "Starting in 2 seconds... (Ctrl+C to abort)"
sleep 2

cd "${WORKSPACE_ROOT}"

echo "==> Launching ROS 2 full stack..."
echo ""

# Launch with all output visible
# Ctrl+C will cleanly stop all nodes via launch system
exec ros2 launch sih26126_bringup full_stack.launch.py "${LAUNCH_ARGS[@]}"
