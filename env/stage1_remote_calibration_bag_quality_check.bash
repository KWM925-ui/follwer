#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
BAG_PATH="${BAG_PATH:-}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/usb_cam/image_raw}"
BOARD_SPEC_YAML="${BOARD_SPEC_YAML:-${REMOTE_WORKSPACE_ROOT}/src/human_follow_fusion/config/calibration_board_spec.current.yaml}"
FRAME_STRIDE="${FRAME_STRIDE:-5}"
SUMMARY_JSON="${SUMMARY_JSON:-}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

if [ -z "${BAG_PATH}" ]; then
  echo "[FAIL] set BAG_PATH" >&2
  exit 1
fi

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u
python3 '${REMOTE_WORKSPACE_ROOT}/src/human_follow_bringup/scripts/calibration_bag_marker_audit.py' \
  --bag '${BAG_PATH}' \
  --image-topic '${IMAGE_TOPIC}' \
  --board-spec '${BOARD_SPEC_YAML}' \
  --frame-stride '${FRAME_STRIDE}' \
  ${SUMMARY_JSON:+--summary-json '${SUMMARY_JSON}'}
EOF
