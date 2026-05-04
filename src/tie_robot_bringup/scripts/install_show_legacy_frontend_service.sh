#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_LEGACY_DIST="/home/hyq-/simple_lashingrobot_show/simple_lashingrobot_ws20260403/simple_lashingrobot_ws/src/APP/dist"
LEGACY_DIST="${1:-${DEFAULT_LEGACY_DIST}}"
SERVICE_USER="${SUDO_USER:-${USER}}"
SERVICE_NAME="tie-robot-show-legacy-frontend.service"
SERVICE_TEMPLATE="${SCRIPT_DIR}/../systemd/${SERVICE_NAME}.in"
SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"
TMP_SERVICE="$(mktemp)"

cleanup() {
  rm -f "${TMP_SERVICE}"
}
trap cleanup EXIT

if [[ ! -d "${LEGACY_DIST}" ]]; then
  echo "旧展示前端 dist 不存在: ${LEGACY_DIST}" >&2
  exit 1
fi

sed \
  -e "s|@LEGACY_DIST@|${LEGACY_DIST}|g" \
  -e "s|@USER@|${SERVICE_USER}|g" \
  "${SERVICE_TEMPLATE}" > "${TMP_SERVICE}"

sudo install -m 0644 "${TMP_SERVICE}" "${SERVICE_TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"

echo "${SERVICE_NAME} 已安装并设置为开机启动。"
echo "访问地址: http://0.0.0.0:5173/"
echo "查看状态: systemctl status ${SERVICE_NAME}"
