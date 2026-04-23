#!/bin/bash
set -euo pipefail

WORKSPACE_ROOT="/home/hyq-/simple_lashingrobot_ws"
LOG_DIR="${WORKSPACE_ROOT}/.runtime_logs"
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"
set -u

# Allow the Trigger response to return before the bridge node is torn down.
sleep 1

if command -v systemctl >/dev/null 2>&1 && sudo -n systemctl status start_ros.service >/dev/null 2>&1; then
  sudo -n systemctl restart start_ros.service
  echo "restarted ros runtime stack via systemd"
  exit 0
fi

pkill -f "roslaunch tie_robot_bringup run.launch" >/dev/null 2>&1 || true
pkill -f "roslaunch tie_robot_bringup driver_stack.launch" >/dev/null 2>&1 || true
pkill -f "roslaunch tie_robot_bringup algorithm_stack.launch" >/dev/null 2>&1 || true
pkill -f "roslaunch tie_robot_bringup api.launch" >/dev/null 2>&1 || true

rosnode kill /suoquNode /moduanNode /gripper_tf_broadcaster /pointAINode /stable_point_tf_broadcaster \
  /topicTransNode /tf2_web_republisher /system_log_mux /scepter_manager /scepter_world_coord_processor \
  >/dev/null 2>&1 || true

nohup roslaunch tie_robot_bringup run.launch start_workspace_picker_web:=true auto_open_workspace_picker:=false \
  >>"${LOG_DIR}/run.launch.log" 2>&1 &

echo "restarted ros runtime stack"
