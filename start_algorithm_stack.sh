#!/bin/bash
set -euo pipefail

WORKSPACE_ROOT="/home/hyq-/simple_lashingrobot_ws"
LOG_DIR="${WORKSPACE_ROOT}/.runtime_logs"
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"
set -u

if rosnode ping -c 1 /pointAINode >/dev/null 2>&1 && rosnode ping -c 1 /stable_point_tf_broadcaster >/dev/null 2>&1; then
  echo "algorithm stack already running"
  exit 0
fi

if pgrep -f "roslaunch tie_robot_bringup algorithm_stack.launch" >/dev/null 2>&1; then
  echo "algorithm_stack.launch already running"
  exit 0
fi

nohup roslaunch tie_robot_bringup algorithm_stack.launch \
  >>"${LOG_DIR}/algorithm_stack.launch.log" 2>&1 &

echo "started algorithm_stack.launch in background"
