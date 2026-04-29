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

# Stop the standalone algorithm_stack launcher first, then ask ROS nodes to exit.
pkill -f "roslaunch tie_robot_bringup algorithm_stack.launch" >/dev/null 2>&1 || true

rosnode kill "${ALGORITHM_NODES[@]}" >/dev/null 2>&1 || true

echo "requested algorithm_stack stop"
