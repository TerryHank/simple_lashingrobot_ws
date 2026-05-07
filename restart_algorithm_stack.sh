#!/bin/bash
set -euo pipefail

BACKEND_SERVICE="tie-robot-backend.service"
STANDALONE_PATTERN="roslaunch tie_robot_bringup algorithm_stack.launch"

# 允许触发服务先返回，避免桥节点被自身动作抢断。
sleep 1

if pgrep -f "${STANDALONE_PATTERN}" >/dev/null 2>&1; then
  echo "standalone algorithm_stack.launch is running; refusing to restart ${BACKEND_SERVICE} to avoid duplicate ROS node names"
  echo "please stop the standalone stack first, or use the full ROS stack restart"
  exit 1
fi

sudo -n systemctl restart "${BACKEND_SERVICE}"

echo "restarted ${BACKEND_SERVICE}"
