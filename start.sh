#!/bin/bash
set -euo pipefail

WORKSPACE_ROOT="/home/hyq-/simple_lashingrobot_ws"

set +u
source /opt/ros/noetic/setup.bash
source "${WORKSPACE_ROOT}/devel/setup.bash"
set -u

cd "${WORKSPACE_ROOT}"

exec roslaunch tie_robot_bringup run.launch \
  start_workspace_picker_web:=true \
  auto_open_workspace_picker:=false
