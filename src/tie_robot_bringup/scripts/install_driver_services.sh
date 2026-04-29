#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${1:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"
SERVICE_USER="${2:-${SUDO_USER:-${USER}}}"
SUDOERS_NAME="tie-robot-driver-control"
SUDOERS_TEMPLATE="${SCRIPT_DIR}/../systemd/${SUDOERS_NAME}.sudoers.in"
SUDOERS_TARGET="/etc/sudoers.d/${SUDOERS_NAME}"
SERVICES=(
  "tie-robot-driver-suoqu.service"
  "tie-robot-driver-moduan.service"
  "tie-robot-driver-camera.service"
)
TMP_SUDOERS="$(mktemp)"
TMP_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${TMP_DIR}" "${TMP_SUDOERS}"
}
trap cleanup EXIT

sed \
  -e "s|@USER@|${SERVICE_USER}|g" \
  "${SUDOERS_TEMPLATE}" > "${TMP_SUDOERS}"
sudo visudo -cf "${TMP_SUDOERS}"

for service_name in "${SERVICES[@]}"; do
  template="${SCRIPT_DIR}/../systemd/${service_name}.in"
  target="/etc/systemd/system/${service_name}"
  tmp_service="${TMP_DIR}/${service_name}"
  sed \
    -e "s|@WORKSPACE@|${WORKSPACE}|g" \
    -e "s|@USER@|${SERVICE_USER}|g" \
    "${template}" > "${tmp_service}"
  sudo install -m 0644 "${tmp_service}" "${target}"
done

sudo install -m 0440 "${TMP_SUDOERS}" "${SUDOERS_TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable tie-robot-driver-suoqu.service
sudo systemctl enable tie-robot-driver-moduan.service
sudo systemctl enable tie-robot-driver-camera.service

echo "驱动 systemd services 已安装并设置为开机启动。"
echo "查看状态: systemctl status tie-robot-driver-suoqu.service tie-robot-driver-moduan.service tie-robot-driver-camera.service"
