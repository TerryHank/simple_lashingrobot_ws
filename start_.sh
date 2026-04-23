#!/bin/bash
set -euo pipefail

WORKSPACE_ROOT="/home/hyq-/simple_lashingrobot_ws"
LOG_DIR="${WORKSPACE_ROOT}/.runtime_logs"
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"
set -u

if pgrep -f "roslaunch tie_robot_bringup run.launch" >/dev/null 2>&1; then
  echo "run.launch already running"
  exit 0
fi

nohup roslaunch tie_robot_bringup run.launch start_workspace_picker_web:=true auto_open_workspace_picker:=false \
  >>"${LOG_DIR}/run.launch.log" 2>&1 &

echo "started run.launch in background"
