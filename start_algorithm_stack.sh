#!/bin/bash
set -euo pipefail

BACKEND_SERVICE="tie-robot-backend.service"
STANDALONE_PATTERN="roslaunch tie_robot_bringup algorithm_stack.launch"

if pgrep -f "${STANDALONE_PATTERN}" >/dev/null 2>&1; then
  echo "standalone algorithm_stack.launch is running; refusing to start ${BACKEND_SERVICE} to avoid duplicate ROS node names"
  echo "please stop the standalone stack first, or use the full ROS stack restart"
  exit 1
fi

sudo -n systemctl start "${BACKEND_SERVICE}"

echo "started ${BACKEND_SERVICE}"
