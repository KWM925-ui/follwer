#!/usr/bin/env bash
set -euo pipefail

LOCAL_WORKSPACE_ROOT="${LOCAL_WORKSPACE_ROOT:-/home/coco/follwer_ws}"
REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
REMOTE_SRC_ROOT="${REMOTE_WORKSPACE_ROOT}/src"
REMOTE_BUILD="${REMOTE_BUILD:-0}"
ALLOW_WHILE_EGO_ACTIVE="${ALLOW_WHILE_EGO_ACTIVE:-0}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)
RSYNC_OPTS=(
  -a
  --delete
  --exclude=.git
  --exclude=.codex
  --exclude=build
  --exclude=devel
  --exclude=logs
  --exclude=.ros
  --exclude=data/samples/public_people_smoke.avi
)
HAS_VENDOR_LIVOX=0

if [ ! -d "${LOCAL_WORKSPACE_ROOT}/src" ]; then
  echo "[FAIL] Local workspace src missing: ${LOCAL_WORKSPACE_ROOT}/src" >&2
  exit 1
fi

if [ -d "${LOCAL_WORKSPACE_ROOT}/vendor/livox_ros_driver2" ]; then
  HAS_VENDOR_LIVOX=1
fi

ACTIVE_REMOTE_PROCS="$(ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'ps -eo pid=,args= | grep -E "(ego|faster_lio|livox|mavros|roslaunch|roscore)" | grep -v -E "(grep -E|stage1_remote_workspace_prepare|ps -eo pid=,args=)" || true')"

if [ -n "${ACTIVE_REMOTE_PROCS}" ] && [ "${ALLOW_WHILE_EGO_ACTIVE}" != "1" ]; then
  echo "[FAIL] Active onboard ROS or EGO processes detected; refusing to sync or build." >&2
  echo "[INFO] Remote processes:" >&2
  printf '%s\n' "${ACTIVE_REMOTE_PROCS}" >&2
  echo "[INFO] Re-run only after an exclusive window exists, or set ALLOW_WHILE_EGO_ACTIVE=1 if you intentionally accept interference risk." >&2
  exit 2
fi

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" "mkdir -p '${REMOTE_SRC_ROOT}'"

rsync "${RSYNC_OPTS[@]}" \
  "${LOCAL_WORKSPACE_ROOT}/src/" \
  "${REMOTE_SSH_TARGET}:${REMOTE_SRC_ROOT}/"

rsync "${RSYNC_OPTS[@]}" \
  "${LOCAL_WORKSPACE_ROOT}/env/" \
  "${REMOTE_SSH_TARGET}:${REMOTE_WORKSPACE_ROOT}/env/"

rsync "${RSYNC_OPTS[@]}" \
  "${LOCAL_WORKSPACE_ROOT}/FOLLOWER_ROUTE_MASTER.txt" \
  "${LOCAL_WORKSPACE_ROOT}/CAMERA_MID360_CALIBRATION_WORK_ORDER.txt" \
  "${REMOTE_SSH_TARGET}:${REMOTE_WORKSPACE_ROOT}/"

if [ "${HAS_VENDOR_LIVOX}" = "1" ]; then
  ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" "rm -rf '${REMOTE_SRC_ROOT}/livox_ros_driver2' && mkdir -p '${REMOTE_SRC_ROOT}/livox_ros_driver2'"
  rsync "${RSYNC_OPTS[@]}" \
    "${LOCAL_WORKSPACE_ROOT}/vendor/livox_ros_driver2/" \
    "${REMOTE_SSH_TARGET}:${REMOTE_SRC_ROOT}/livox_ros_driver2/"
fi

if [ "${REMOTE_BUILD}" = "1" ]; then
  ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
set -u
cd '${REMOTE_WORKSPACE_ROOT}'
catkin_make -DCATKIN_ENABLE_TESTING=OFF -DROS_EDITION=ROS1
EOF
fi

echo "[PASS] Remote follower workspace prepared at ${REMOTE_WORKSPACE_ROOT}"
