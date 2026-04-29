#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${1:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
SERVICE_USER="${2:-${SUDO_USER:-${USER}}}"
SERVICE_NAME="tie-robot-rosbridge.service"
SERVICE_TEMPLATE="${SCRIPT_DIR}/../systemd/${SERVICE_NAME}.in"
SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"
TMP_SERVICE="$(mktemp)"

cleanup() {
  rm -f "${TMP_SERVICE}"
}
trap cleanup EXIT

sed \
  -e "s|@WORKSPACE@|${WORKSPACE}|g" \
  -e "s|@USER@|${SERVICE_USER}|g" \
  "${SERVICE_TEMPLATE}" > "${TMP_SERVICE}"

sudo install -m 0644 "${TMP_SERVICE}" "${SERVICE_TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable tie-robot-rosbridge.service
sudo systemctl restart tie-robot-rosbridge.service

echo "tie-robot-rosbridge.service 已安装并设置为开机启动。"
echo "查看状态: systemctl status tie-robot-rosbridge.service"
