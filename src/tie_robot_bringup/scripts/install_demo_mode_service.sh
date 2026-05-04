#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${1:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
SERVICE_USER="${2:-${SUDO_USER:-${USER}}}"
DEFAULT_LEGACY_WORKSPACE="/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws"
DEFAULT_SCEPTER_ROS="/home/hyq-/ScepterSDK/3rd-PartyPlugin/ROS"
LEGACY_WORKSPACE="${3:-${DEFAULT_LEGACY_WORKSPACE}}"
SCEPTER_ROS="${4:-${DEFAULT_SCEPTER_ROS}}"
SERVICE_NAME="tie-robot-demo-show-full.service"
DEMO_ROSBRIDGE_SERVICE_NAME="tie-robot-demo-rosbridge.service"
SERVICE_TEMPLATE="${SCRIPT_DIR}/../systemd/${SERVICE_NAME}.in"
DEMO_ROSBRIDGE_SERVICE_TEMPLATE="${SCRIPT_DIR}/../systemd/${DEMO_ROSBRIDGE_SERVICE_NAME}.in"
SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"
DEMO_ROSBRIDGE_SERVICE_TARGET="/etc/systemd/system/${DEMO_ROSBRIDGE_SERVICE_NAME}"
TMP_SERVICE="$(mktemp)"
TMP_DEMO_ROSBRIDGE_SERVICE="$(mktemp)"

cleanup() {
  rm -f "${TMP_SERVICE}" "${TMP_DEMO_ROSBRIDGE_SERVICE}"
}
trap cleanup EXIT

if [[ ! -f "${LEGACY_WORKSPACE}/src/chassis_ctrl/launch/show_full.launch" ]]; then
  echo "旧工作目录 show_full.launch 不存在: ${LEGACY_WORKSPACE}" >&2
  exit 1
fi

if [[ ! -f "${SCEPTER_ROS}/devel/setup.bash" ]]; then
  echo "ScepterSDK ROS 环境不存在: ${SCEPTER_ROS}/devel/setup.bash" >&2
  exit 1
fi

sed \
  -e "s|@WORKSPACE@|${WORKSPACE}|g" \
  -e "s|@USER@|${SERVICE_USER}|g" \
  -e "s|@LEGACY_WORKSPACE@|${LEGACY_WORKSPACE}|g" \
  -e "s|@SCEPTER_ROS@|${SCEPTER_ROS}|g" \
  "${SERVICE_TEMPLATE}" > "${TMP_SERVICE}"

sed \
  -e "s|@WORKSPACE@|${WORKSPACE}|g" \
  -e "s|@USER@|${SERVICE_USER}|g" \
  "${DEMO_ROSBRIDGE_SERVICE_TEMPLATE}" > "${TMP_DEMO_ROSBRIDGE_SERVICE}"

sudo install -m 0644 "${TMP_DEMO_ROSBRIDGE_SERVICE}" "${DEMO_ROSBRIDGE_SERVICE_TARGET}"
sudo install -m 0644 "${TMP_SERVICE}" "${SERVICE_TARGET}"
sudo systemctl daemon-reload
sudo systemctl disable tie-robot-demo-rosbridge.service >/dev/null 2>&1 || true
sudo systemctl disable tie-robot-demo-show-full.service >/dev/null 2>&1 || true

if systemctl list-unit-files tie-robot-show-legacy-shared-driver-stack.service --no-pager --no-legend | grep -q '^tie-robot-show-legacy-shared-driver-stack.service'; then
  sudo systemctl disable --now tie-robot-show-legacy-shared-driver-stack.service || true
fi

echo "${DEMO_ROSBRIDGE_SERVICE_NAME} 与 ${SERVICE_NAME} 已安装为按需启动服务，不设置开机自启。"
echo "演示模式由新前端按钮启动/停止。"
echo "查看状态: systemctl status ${SERVICE_NAME}"
