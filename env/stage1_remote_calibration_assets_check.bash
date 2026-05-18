#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
INTRINSICS_YAML="${INTRINSICS_YAML:-}"
EXTRINSICS_YAML="${EXTRINSICS_YAML:-}"
CAMERA_BODY_YAML="${CAMERA_BODY_YAML:-}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

if [ -z "${INTRINSICS_YAML}" ] || [ -z "${EXTRINSICS_YAML}" ] || [ -z "${CAMERA_BODY_YAML}" ]; then
  echo "[FAIL] set INTRINSICS_YAML, EXTRINSICS_YAML, and CAMERA_BODY_YAML" >&2
  exit 1
fi

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u
python3 '${REMOTE_WORKSPACE_ROOT}/src/human_follow_fusion/scripts/calibration_asset_validator.py' \
  --intrinsics '${INTRINSICS_YAML}' \
  --extrinsics '${EXTRINSICS_YAML}' \
  --camera-body '${CAMERA_BODY_YAML}'
EOF
