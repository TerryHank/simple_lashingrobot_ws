#!/bin/bash
set -euo pipefail

WORKSPACE_ROOT="/home/hyq-/simple_lashingrobot_ws"
LOG_DIR="${WORKSPACE_ROOT}/.runtime_logs"
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"
set -u

# 允许触发服务先返回，避免桥节点被自身动作抢断。
sleep 1

rosnode kill /pointAINode /stable_point_tf_broadcaster >/dev/null 2>&1 || true
pkill -f "roslaunch tie_robot_bringup algorithm_stack.launch" >/dev/null 2>&1 || true

if pgrep -f "roslaunch tie_robot_bringup run.launch" >/dev/null 2>&1; then
  echo "requested algorithm stack restart under run.launch"
  exit 0
fi

nohup roslaunch tie_robot_bringup algorithm_stack.launch \
  >>"${LOG_DIR}/algorithm_stack.launch.log" 2>&1 &

echo "restarted algorithm_stack.launch in background"
