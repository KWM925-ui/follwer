#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
PROBE_SEC="${PROBE_SEC:-12.0}"
IMAGE_ROTATION_DEG="${IMAGE_ROTATION_DEG:-180}"
FLIP_HORIZONTAL="${FLIP_HORIZONTAL:-false}"
FLIP_VERTICAL="${FLIP_VERTICAL:-false}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail

cleanup() {
  kill "\${FOLLOW_PID:-}" "\${USB_PID:-}" "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
  wait "\${FOLLOW_PID:-}" >/dev/null 2>&1 || true
  wait "\${USB_PID:-}" >/dev/null 2>&1 || true
  wait "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

roscore >/tmp/stage1_live_target_presence_roscore.log 2>&1 &
ROSCORE_PID=\$!

for _ in \$(seq 1 20); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

if ! rosparam list >/dev/null 2>&1; then
  echo "[FAIL] roscore did not become ready" >&2
  exit 1
fi

roslaunch --wait human_follow_bringup stage1_usb_cam.launch video_device:='${VIDEO_DEVICE}' >/tmp/stage1_live_target_presence_usb.log 2>&1 &
USB_PID=\$!

roslaunch --wait human_follow_bringup stage1_live_core.launch image_topic:=/camera/usb_cam/image_raw require_image_heartbeat:=true image_rotation_deg:='${IMAGE_ROTATION_DEG}' flip_horizontal:='${FLIP_HORIZONTAL}' flip_vertical:='${FLIP_VERTICAL}' >/tmp/stage1_live_target_presence_follow.log 2>&1 &
FOLLOW_PID=\$!

python3 - <<'PY'
import sys
import time

import rospy
from human_follow_msgs.msg import FollowCommand, FollowState, Target2D

probe_sec = float("${PROBE_SEC}")
seen = {
    "detector_msgs": 0,
    "detector_valid": 0,
    "state_msgs": 0,
    "last_state": None,
    "command_msgs": 0,
    "command_valid": 0,
    "last_mode": None,
    "last_forward": None,
    "last_lateral": None,
    "last_yaw": None,
}

def det_cb(msg):
    seen["detector_msgs"] += 1
    if msg.valid:
        seen["detector_valid"] += 1

def state_cb(msg):
    seen["state_msgs"] += 1
    seen["last_state"] = msg.state_name

def cmd_cb(msg):
    seen["command_msgs"] += 1
    seen["last_mode"] = msg.mode
    seen["last_forward"] = msg.forward_mps
    seen["last_lateral"] = msg.lateral_mps
    seen["last_yaw"] = msg.yaw_rate_rps
    if msg.valid:
        seen["command_valid"] += 1

rospy.init_node("stage1_live_target_presence_probe", anonymous=True)
rospy.Subscriber("/follow/detector/person_target", Target2D, det_cb, queue_size=10)
rospy.Subscriber("/follow/state", FollowState, state_cb, queue_size=10)
rospy.Subscriber("/follow/control/cmd_body", FollowCommand, cmd_cb, queue_size=10)

deadline = time.time() + probe_sec
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    rate.sleep()

print(
    "[presence] detector_msgs=%d detector_valid=%d state_msgs=%d last_state=%s command_msgs=%d command_valid=%d last_mode=%s forward=%s lateral=%s yaw=%s"
    % (
        seen["detector_msgs"],
        seen["detector_valid"],
        seen["state_msgs"],
        seen["last_state"],
        seen["command_msgs"],
        seen["command_valid"],
        seen["last_mode"],
        seen["last_forward"],
        seen["last_lateral"],
        seen["last_yaw"],
    )
)

if seen["detector_valid"] > 0 and seen["command_valid"] > 0:
    print("[presence_ok] live target data available")
    sys.exit(0)

print("[presence_none] no valid live target observed in probe window")
sys.exit(0)
PY
EOF
