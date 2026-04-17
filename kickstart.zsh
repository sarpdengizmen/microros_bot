#!/usr/bin/env zsh
# kickstart.zsh — start all robot nodes in one shot.
#
# Opens (skips if already running):
#   1. micro-ROS agent        (ESP32 ↔ ROS 2 bridge)
#   2. Camera Controller      (pose tracking + cmd_vel)
#   3. Automation Node        (pick-and-place state machine)
#   4. Motor Debug echo       (/motor_debug)
#   5. cmd_vel echo           (/cmd_vel)
#   6. ESP32 Serial Monitor   (firmware logs)
#   7. Teleop Keyboard        (manual override — don't use while automation active)
#
# Usage:
#   chmod +x kickstart.zsh            # first time only
#   ./kickstart.zsh                   # default camera (device 0)
#   ./kickstart.zsh -c 1              # use /dev/video1
#   ./kickstart.zsh --camera 2        # use /dev/video2
#   ./kickstart.zsh -c /dev/video0    # explicit device path
#
#   Then start the automation cycle:
#   ros2 service call /start_auto std_srvs/srv/Trigger
#   ros2 service call /stop_auto  std_srvs/srv/Trigger

# ── Parse arguments ─────────────────────────────────────────────────────────────
CAMERA_DEVICE=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--camera)
            CAMERA_DEVICE="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [-c|--camera <device>]"
            exit 1
            ;;
    esac
done

ROS_SETUP="/opt/ros/jazzy/setup.zsh"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
MICROROS_BOT_DIR="$PROJECT_DIR/microROS_bot"
TERM_CMD="gnome-terminal"

# Use project venv (has pupil_apriltags, opencv, scipy + ROS via system-site-packages)
PYTHON="${PROJECT_DIR}/.venv/bin/python3"
[[ -x "$PYTHON" ]] || PYTHON="python3"

# ── Helper ─────────────────────────────────────────────────────────────────────
# open_term <title> <pgrep-pattern> <shell-command>
# Skips the terminal if <pgrep-pattern> already matches a running process.
# Pass "" as pattern to always open.
open_term() {
    local title="$1"
    local pattern="$2"
    local cmd="$3"

    if [[ -n "$pattern" ]] && pgrep -f "$pattern" > /dev/null 2>&1; then
        echo "  [skip] $title — already running"
        return
    fi

    $TERM_CMD --title="$title" -- zsh -c \
        "source $ROS_SETUP 2>/dev/null; $cmd; exec zsh" &
    echo "  [open] $title"
}

# ── Launch ─────────────────────────────────────────────────────────────────────
echo "=== Robot kickstart ==="
echo ""

# 1. micro-ROS agent — start first so the ESP32 can connect
open_term "microROS Agent" \
    "micro_ros_agent udp4" \
    "cd ~/microros_ws && source install/local_setup.zsh && \
     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"

# Give the agent a moment to come up, then refresh the ROS 2 daemon
sleep 2
source "$ROS_SETUP" 2>/dev/null
echo "  Refreshing ROS 2 daemon..."
ros2 daemon stop 2>/dev/null
ros2 daemon start 2>/dev/null

# 2. Camera controller — pose estimation + closed-loop cmd_vel
open_term "Camera Controller" \
    "camera_controller_node.py" \
    "cd $PROJECT_DIR && $PYTHON camera_controller_node.py $CAMERA_DEVICE"

# 3. Automation node — pick-and-place state machine
open_term "Automation Node" \
    "automation_node.py" \
    "cd $PROJECT_DIR && $PYTHON automation_node.py"

# 4. Motor debug echo
open_term "Motor Debug (/motor_debug)" \
    "topic echo /motor_debug" \
    "ros2 topic echo /motor_debug"

# 5. cmd_vel echo
open_term "cmd_vel echo (/cmd_vel)" \
    "topic echo /cmd_vel" \
    "ros2 topic echo /cmd_vel"

# 6. ESP32 serial monitor
open_term "ESP32 Serial Monitor" \
    "pio device monitor" \
    "cd $MICROROS_BOT_DIR && pio device monitor --baud 115200"

# 7. Teleop keyboard — manual override
#    WARNING: do not publish /cmd_vel via teleop while automation is active
open_term "Teleop Keyboard (manual)" \
    "teleop_twist_keyboard" \
    "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

echo ""
echo "Done. All terminals launched."
echo ""
echo "Service quick-reference:"
echo "  ros2 service call /start_auto   std_srvs/srv/Trigger   # begin automation"
echo "  ros2 service call /stop_auto    std_srvs/srv/Trigger   # halt automation"
echo "  ros2 service call /start_motion std_srvs/srv/Trigger   # manual: move to object"
echo "  ros2 service call /stop_motion  std_srvs/srv/Trigger   # manual: stop motion"
echo "  ros2 service call /pickup       std_srvs/srv/Trigger   # manual: run pickup"
echo "  ros2 service call /drop         std_srvs/srv/Trigger   # manual: run drop"
echo ""
echo "  WARNING: do not use Teleop while automation is active."
