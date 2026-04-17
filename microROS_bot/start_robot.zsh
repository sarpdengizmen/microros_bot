#!/usr/bin/env zsh
# start_robot.zsh — opens all robot dev terminals in one shot.
# Skips terminals whose backing process is already running.
# Assumes gnome-terminal. Change TERM_CMD if you use a different emulator.

ROS_SETUP="/opt/ros/jazzy/setup.zsh"
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
TERM_CMD="gnome-terminal"

# ── Helper ────────────────────────────────────────────────────────────────────
# open_term <window-title> <pgrep-pattern> <shell-command>
# If pgrep-pattern is non-empty and matches a running process, the terminal
# is skipped. Pass "" as pgrep-pattern to always open.
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

# ── Main ──────────────────────────────────────────────────────────────────────
echo "=== Robot dev environment ==="
echo ""

# 1. micro-ROS agent  (start first so the ESP32 can connect)
open_term "microROS Agent" \
    "micro_ros_agent udp4" \
    "cd ~/microros_ws && source install/local_setup.zsh && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"

# Give the agent a moment to come up, then refresh the ROS 2 daemon so that
# plain `ros2 topic list` (no --no-daemon) works correctly.
sleep 2
source "$ROS_SETUP" 2>/dev/null
echo "  Refreshing ROS 2 daemon..."
ros2 daemon stop 2>/dev/null
ros2 daemon start 2>/dev/null

# 2. Teleop keyboard  — sends /cmd_vel
open_term "Teleop Keyboard" \
    "teleop_twist_keyboard" \
    "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

# 3. Motor debug echo — /motor_debug Float32[6]:
#    [targetRPM_L, actualRPM_L, targetRPM_R, actualRPM_R, pid_out_L, pid_out_R]
open_term "Motor Debug (/motor_debug)" \
    "topic echo /motor_debug" \
    "ros2 topic echo /motor_debug"

# 4. cmd_vel echo — confirm teleop commands are reaching the agent
open_term "cmd_vel echo (/cmd_vel)" \
    "topic echo /cmd_vel" \
    "ros2 topic echo /cmd_vel"

# 5. ESP32 serial monitor — raw firmware output (connection status, RPM logs, etc.)
open_term "ESP32 Serial Monitor" \
    "pio device monitor" \
    "cd $PROJECT_DIR && pio device monitor --baud 115200"

echo ""
echo "Done. Waiting for ESP32 to connect to the agent..."
echo "Tip: run this script again to open any terminal that was closed."
