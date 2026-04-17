#!/usr/bin/env zsh
# kickstart.zsh — start the full pick-and-place stack.
#
# Opens (skips if already running):
#   1. micro-ROS agent        (ESP32 ↔ ROS 2 bridge — start first)
#   2. workspace_tracker      (AprilTag pose + red object detection)
#   3. camera_controller      (trajectory regulator + service servers)
#   4. motor_debug echo       (/motor_debug telemetry)
#   5. cmd_vel echo           (/cmd_vel verification)
#
# After launching, open FSM.slx in MATLAB, connect External mode,
# and toggle run_enable to begin the pick-and-place cycle.
#
# Usage:
#   ./kickstart.zsh           # default camera (device 0)
#   ./kickstart.zsh -c 1      # use /dev/video1

set -e

# ── Parse arguments ─────────────────────────────────────────────────────────
CAMERA=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--camera) CAMERA="$2"; shift 2 ;;
        *) echo "Usage: $0 [-c|--camera <device>]"; exit 1 ;;
    esac
done

ROS_SETUP="/opt/ros/jazzy/setup.zsh"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

PYTHON="${PROJECT_DIR}/.venv/bin/python3"
[[ -x "$PYTHON" ]] || PYTHON="python3"

# ── Helper ───────────────────────────────────────────────────────────────────
open_term() {
    local title="$1" pattern="$2" cmd="$3"
    if [[ -n "$pattern" ]] && pgrep -f "$pattern" > /dev/null 2>&1; then
        echo "  [skip] $title — already running"
        return
    fi
    gnome-terminal --title="$title" -- zsh -c \
        "source $ROS_SETUP 2>/dev/null; $cmd; exec zsh" &
    echo "  [open] $title"
}

# ── Launch ───────────────────────────────────────────────────────────────────
echo "=== Robot kickstart (camera: $CAMERA) ==="

# 1. micro-ROS agent — must come first so the ESP32 can connect
open_term "microROS Agent" \
    "micro_ros_agent udp4" \
    "cd ~/microros_ws && source install/local_setup.zsh && \
     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"

sleep 2
source "$ROS_SETUP" 2>/dev/null
ros2 daemon stop 2>/dev/null; ros2 daemon start 2>/dev/null

# 2. Vision node — AprilTag pose estimation + red object detection
open_term "Workspace Tracker" \
    "workspace_tracker.py" \
    "cd $PROJECT_DIR && $PYTHON workspace_tracker.py -c $CAMERA"

# 3. Trajectory regulator + pick/drop service servers (driven by Simulink)
open_term "Camera Controller" \
    "camera_controller_node.py" \
    "cd $PROJECT_DIR && $PYTHON camera_controller_node.py -c $CAMERA"

# 4. Motor debug telemetry
open_term "Motor Debug" \
    "topic echo /motor_debug" \
    "ros2 topic echo /motor_debug"

# 5. cmd_vel echo — verify commands are reaching the firmware
open_term "cmd_vel echo" \
    "topic echo /cmd_vel" \
    "ros2 topic echo /cmd_vel"

echo ""
echo "Done. Open FSM.slx in MATLAB → External mode → toggle run_enable."
echo ""
echo "Manual service calls:"
echo "  ros2 service call /start_motion std_srvs/srv/Trigger"
echo "  ros2 service call /stop_motion  std_srvs/srv/Trigger"
echo "  ros2 service call /pickup       std_srvs/srv/Trigger"
echo "  ros2 service call /drop         std_srvs/srv/Trigger"
