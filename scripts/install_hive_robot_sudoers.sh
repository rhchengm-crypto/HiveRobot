#!/usr/bin/env bash
set -euo pipefail

# Allow the web stream server to run specific arm-control scripts without an
# interactive sudo password. This intentionally grants only named scripts, not
# blanket passwordless sudo.

USER_NAME="${SUDO_USER:-${USER}}"
PYTHON_BIN="$(command -v python3)"
DEFAULT_DIR="/home/nvidia/hive_robot/DM_Control_Python"
SUDOERS_FILE="/etc/sudoers.d/hive-robot-left-arm-controller"

if [[ "$#" -gt 0 ]]; then
  SCRIPT_PATHS=("$@")
else
  SCRIPT_PATHS=(
    "${DEFAULT_DIR}/left_arm_controller.py"
    "${DEFAULT_DIR}/teach_left_arm.py"
  )
fi

for path in "${SCRIPT_PATHS[@]}"; do
  if [[ ! -f "${path}" ]]; then
    echo "controller script not found: ${path}" >&2
    echo "usage: sudo bash install_hive_robot_sudoers.sh [/absolute/path/to/script.py ...]" >&2
    exit 1
  fi
done

TMP_FILE="$(mktemp)"
{
  echo "# Installed by HiveRobot. Allows browser-triggered arm control without a sudo password."
  for path in "${SCRIPT_PATHS[@]}"; do
    echo "${USER_NAME} ALL=(root) NOPASSWD: ${PYTHON_BIN} ${path} *"
  done
} > "${TMP_FILE}"

visudo -cf "${TMP_FILE}" >/dev/null
install -m 0440 -o root -g root "${TMP_FILE}" "${SUDOERS_FILE}"
rm -f "${TMP_FILE}"

echo "installed ${SUDOERS_FILE}"
echo "allowed commands:"
for path in "${SCRIPT_PATHS[@]}"; do
  echo "  sudo -n ${PYTHON_BIN} ${path} ..."
done
