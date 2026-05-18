#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
REMOTE_OUTPUT_DIR="${REMOTE_OUTPUT_DIR:-/home/nv/human_follow_ws/data/calibration/overlay_debug}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/usb_cam/image_raw}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/laserMapping/cloud_registered_body}"
INTRINSICS_YAML="${INTRINSICS_YAML:-}"
EXTRINSICS_YAML="${EXTRINSICS_YAML:-}"
SESSION_TAG="${SESSION_TAG:-$(date +%Y%m%d_%H%M%S)}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

if [ -z "${INTRINSICS_YAML}" ] || [ -z "${EXTRINSICS_YAML}" ]; then
  echo "[FAIL] set INTRINSICS_YAML and EXTRINSICS_YAML" >&2
  exit 1
fi

remote_snapshot="${REMOTE_OUTPUT_DIR}/projection_overlay_${SESSION_TAG}.png"
local_snapshot="${WORKSPACE_ROOT}/artifacts/projection_overlay/projection_overlay_${SESSION_TAG}.png"

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

mkdir -p '${REMOTE_OUTPUT_DIR}'
timeout 12s rosrun human_follow_fusion projection_overlay_debug_node.py \
  _image_topic:='${IMAGE_TOPIC}' \
  _pointcloud_topic:='${POINTCLOUD_TOPIC}' \
  _intrinsics_yaml:='${INTRINSICS_YAML}' \
  _extrinsics_yaml:='${EXTRINSICS_YAML}' \
  _snapshot_path:='${remote_snapshot}' \
  _publish_rate_hz:=3.0 \
  _max_points_to_process:=4000 || true

if [ ! -f '${remote_snapshot}' ]; then
  echo "[FAIL] projection overlay snapshot was not created" >&2
  exit 1
fi
EOF

mkdir -p "$(dirname "${local_snapshot}")"
scp "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}:${remote_snapshot}" "${local_snapshot}" >/dev/null
echo "[snapshot_local] ${local_snapshot}"
