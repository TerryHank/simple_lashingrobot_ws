#!/bin/bash
set -euo pipefail

WORKSPACE_ROOT="/home/hyq-/simple_lashingrobot_ws"
LOG_DIR="${WORKSPACE_ROOT}/.runtime_logs"
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"
set -u

ALGORITHM_NODES=(
  /pointAINode
  /bind_map_builder
  /global_bind_planner
  /cabin_motion_controller
  /moduan_motion_controller
  /bind_task_executor
)

if pgrep -f "roslaunch tie_robot_bringup algorithm_stack.launch" >/dev/null 2>&1; then
  echo "algorithm_stack.launch already running"
  exit 0
fi

LIVE_NODES="$(rosnode list 2>/dev/null || true)"
for node in "${ALGORITHM_NODES[@]}"; do
  if printf '%s\n' "${LIVE_NODES}" | grep -Fx -- "${node}" >/dev/null 2>&1; then
    echo "algorithm stack already has live node: ${node}"
    exit 0
  fi
done

nohup roslaunch tie_robot_bringup algorithm_stack.launch \
  >>"${LOG_DIR}/algorithm_stack.launch.log" 2>&1 &

echo "started algorithm_stack.launch in background"
