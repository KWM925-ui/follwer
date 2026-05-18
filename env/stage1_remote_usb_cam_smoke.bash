#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

roslaunch human_follow_bringup stage1_usb_cam.launch video_device:='${VIDEO_DEVICE}' >/tmp/stage1_usb_cam_smoke.log 2>&1 &
LAUNCH_PID=\$!
sleep 5
echo "[topic]"
timeout 8s rostopic echo -n 1 /camera/usb_cam/image_raw/header
echo "[encoding]"
timeout 8s rostopic echo -n 1 /camera/usb_cam/image_raw/encoding
kill \$LAUNCH_PID >/dev/null 2>&1 || true
wait \$LAUNCH_PID >/dev/null 2>&1 || true
EOF

echo "[PASS] remote usb_cam smoke passed"
