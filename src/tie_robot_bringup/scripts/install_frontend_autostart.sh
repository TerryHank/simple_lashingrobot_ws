#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${1:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
SERVICE_USER="${SUDO_USER:-${USER}}"
SERVICE_NAME="tie-robot-frontend.service"
SERVICE_TEMPLATE="${SCRIPT_DIR}/../systemd/${SERVICE_NAME}.in"
SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"
ROSBRIDGE_INSTALLER="${SCRIPT_DIR}/install_rosbridge_service.sh"
DRIVER_INSTALLER="${SCRIPT_DIR}/install_driver_services.sh"
BACKEND_INSTALLER="${SCRIPT_DIR}/install_backend_service.sh"
LEGACY_FRONTEND_INSTALLER="${SCRIPT_DIR}/install_show_legacy_frontend_service.sh"
DEMO_MODE_INSTALLER="${SCRIPT_DIR}/install_demo_mode_service.sh"
TMP_SERVICE="$(mktemp)"

cleanup() {
  rm -f "${TMP_SERVICE}"
}
trap cleanup EXIT

sed \
  -e "s|@WORKSPACE@|${WORKSPACE}|g" \
  -e "s|@USER@|${SERVICE_USER}|g" \
  "${SERVICE_TEMPLATE}" > "${TMP_SERVICE}"

"${ROSBRIDGE_INSTALLER}" "${WORKSPACE}" "${SERVICE_USER}"
"${DRIVER_INSTALLER}" "${WORKSPACE}" "${SERVICE_USER}"
"${BACKEND_INSTALLER}" "${WORKSPACE}" "${SERVICE_USER}"
"${LEGACY_FRONTEND_INSTALLER}"
"${DEMO_MODE_INSTALLER}" "${WORKSPACE}" "${SERVICE_USER}"
sudo install -m 0644 "${TMP_SERVICE}" "${SERVICE_TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable tie-robot-frontend.service
sudo systemctl restart tie-robot-frontend.service

echo "tie-robot-frontend.service 已安装并设置为开机启动。"
echo "查看状态: systemctl status tie-robot-frontend.service"
