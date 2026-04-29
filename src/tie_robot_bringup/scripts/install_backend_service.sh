#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${1:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
SERVICE_USER="${2:-${SUDO_USER:-${USER}}}"
SERVICE_NAME="tie-robot-backend.service"
SUDOERS_NAME="tie-robot-backend-control"
SERVICE_TEMPLATE="${SCRIPT_DIR}/../systemd/${SERVICE_NAME}.in"
SUDOERS_TEMPLATE="${SCRIPT_DIR}/../systemd/${SUDOERS_NAME}.sudoers.in"
SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"
SUDOERS_TARGET="/etc/sudoers.d/${SUDOERS_NAME}"
TMP_SERVICE="$(mktemp)"
TMP_SUDOERS="$(mktemp)"

cleanup() {
  rm -f "${TMP_SERVICE}" "${TMP_SUDOERS}"
}
trap cleanup EXIT

sed \
  -e "s|@WORKSPACE@|${WORKSPACE}|g" \
  -e "s|@USER@|${SERVICE_USER}|g" \
  "${SERVICE_TEMPLATE}" > "${TMP_SERVICE}"

sed \
  -e "s|@USER@|${SERVICE_USER}|g" \
  "${SUDOERS_TEMPLATE}" > "${TMP_SUDOERS}"

sudo visudo -cf "${TMP_SUDOERS}"
sudo install -m 0644 "${TMP_SERVICE}" "${SERVICE_TARGET}"
sudo install -m 0440 "${TMP_SUDOERS}" "${SUDOERS_TARGET}"
sudo systemctl daemon-reload

echo "tie-robot-backend.service 已安装。"
echo "前端用户 ${SERVICE_USER} 已获得受限 systemctl start/stop/restart 权限。"
echo "查看状态: systemctl status tie-robot-backend.service"
