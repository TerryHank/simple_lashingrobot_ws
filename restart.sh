#!/bin/bash
set -euo pipefail

WORKSPACE_ROOT="/home/hyq-/simple_lashingrobot_ws"
LOG_DIR="${WORKSPACE_ROOT}/.runtime_logs"
mkdir -p "${LOG_DIR}"

source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"

if rosnode ping -c 1 /suoquNode >/dev/null 2>&1 && rosnode ping -c 1 /moduanNode >/dev/null 2>&1; then
  echo "driver stack already running"
  exit 0
fi

if pgrep -f "roslaunch tie_robot_bringup driver_stack.launch" >/dev/null 2>&1; then
  echo "driver_stack.launch already running"
  exit 0
fi

nohup roslaunch tie_robot_bringup driver_stack.launch \
  >>"${LOG_DIR}/driver_stack.launch.log" 2>&1 &

echo "started driver_stack.launch in background"
