#!/usr/bin/env zsh
# start_robot.zsh — firmware development environment.
# Opens terminals for testing the ESP32 in isolation (no vision, no Simulink).
#
# Opens (skips if already running):
#   1. micro-ROS agent        (ESP32 ↔ ROS 2 bridge)
#   2. ESP32 serial monitor   (firmware logs)
#   3. motor_debug echo       (/motor_debug telemetry)
#   4. cmd_vel echo           (/cmd_vel verification)
#   5. Teleop keyboard        (manual /cmd_vel for motor testing)

set -e

ROS_SETUP="/opt/ros/jazzy/setup.zsh"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

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
echo "=== Firmware dev environment ==="

# 1. micro-ROS agent — start first so the ESP32 can connect
open_term "microROS Agent" \
    "micro_ros_agent udp4" \
    "cd ~/microros_ws && source install/local_setup.zsh && \
     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"

sleep 2
source "$ROS_SETUP" 2>/dev/null
ros2 daemon stop 2>/dev/null; ros2 daemon start 2>/dev/null

# 2. Serial monitor — firmware connection status and debug output
open_term "ESP32 Serial Monitor" \
    "pio device monitor" \
    "cd $PROJECT_DIR && pio device monitor --baud 115200"

# 3. Motor debug telemetry
open_term "Motor Debug" \
    "topic echo /motor_debug" \
    "ros2 topic echo /motor_debug"

# 4. cmd_vel echo — verify commands reach the firmware
open_term "cmd_vel echo" \
    "topic echo /cmd_vel" \
    "ros2 topic echo /cmd_vel"

# 5. Teleop — manual motor testing
open_term "Teleop Keyboard" \
    "teleop_twist_keyboard" \
    "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

echo ""
echo "Done. Waiting for ESP32 to connect to the agent..."
echo "Build & flash: cd microROS_bot && pio run -t upload"
