#!/usr/bin/env bash
set -euo pipefail

# Allow the web stream server to run the left-arm controller without an
# interactive sudo password. This intentionally grants only the controller
# script, not blanket passwordless sudo.

USER_NAME="${SUDO_USER:-${USER}}"
PYTHON_BIN="$(command -v python3)"
CONTROLLER_PATH="${1:-/home/nvidia/hive_robot/DM_Control_Python/left_arm_controller.py}"
SUDOERS_FILE="/etc/sudoers.d/hive-robot-left-arm-controller"

if [[ ! -f "${CONTROLLER_PATH}" ]]; then
  echo "controller not found: ${CONTROLLER_PATH}" >&2
  echo "usage: sudo bash install_hive_robot_sudoers.sh /absolute/path/to/left_arm_controller.py" >&2
  exit 1
fi

TMP_FILE="$(mktemp)"
cat > "${TMP_FILE}" <<EOF
# Installed by HiveRobot. Allows browser-triggered arm control without a sudo password.
${USER_NAME} ALL=(root) NOPASSWD: ${PYTHON_BIN} ${CONTROLLER_PATH} *
EOF

visudo -cf "${TMP_FILE}" >/dev/null
install -m 0440 -o root -g root "${TMP_FILE}" "${SUDOERS_FILE}"
rm -f "${TMP_FILE}"

echo "installed ${SUDOERS_FILE}"
echo "allowed command:"
echo "  sudo -n ${PYTHON_BIN} ${CONTROLLER_PATH} ..."
