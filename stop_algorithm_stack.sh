#!/bin/bash
set -euo pipefail

BACKEND_SERVICE="tie-robot-backend.service"
STANDALONE_PATTERN="roslaunch tie_robot_bringup algorithm_stack.launch"

sudo -n systemctl stop "${BACKEND_SERVICE}" || true

# Clean up the old standalone launcher used by earlier UI actions.
pkill -INT -f "${STANDALONE_PATTERN}" >/dev/null 2>&1 || true
sleep 1
pkill -TERM -f "${STANDALONE_PATTERN}" >/dev/null 2>&1 || true

echo "requested ${BACKEND_SERVICE} and standalone algorithm_stack.launch stop"
