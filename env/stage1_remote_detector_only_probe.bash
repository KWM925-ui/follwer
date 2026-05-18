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
SAMPLE_LIMIT="${SAMPLE_LIMIT:-8}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail

cleanup() {
  kill "\${DET_PID:-}" "\${USB_PID:-}" "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
  wait "\${DET_PID:-}" >/dev/null 2>&1 || true
  wait "\${USB_PID:-}" >/dev/null 2>&1 || true
  wait "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

roscore >/tmp/stage1_detector_only_roscore.log 2>&1 &
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

roslaunch --wait human_follow_bringup stage1_usb_cam.launch video_device:='${VIDEO_DEVICE}' >/tmp/stage1_detector_only_usb.log 2>&1 &
USB_PID=\$!

roslaunch --wait human_follow_bringup stage1_live_detector_only.launch image_topic:=/camera/usb_cam/image_raw require_image_heartbeat:=true image_rotation_deg:='${IMAGE_ROTATION_DEG}' flip_horizontal:='${FLIP_HORIZONTAL}' flip_vertical:='${FLIP_VERTICAL}' >/tmp/stage1_detector_only_follow.log 2>&1 &
DET_PID=\$!

python3 - <<'PY'
import sys
import time

import rospy
from human_follow_msgs.msg import Target2D

probe_sec = float("${PROBE_SEC}")
sample_limit = int("${SAMPLE_LIMIT}")
seen = {
    "msgs": 0,
    "valid": 0,
    "samples": [],
}

def cb(msg):
    seen["msgs"] += 1
    if msg.valid:
        seen["valid"] += 1
        if len(seen["samples"]) < sample_limit:
            seen["samples"].append(
                (
                    float(msg.confidence),
                    float(msg.cx),
                    float(msg.cy),
                    float(msg.width),
                    float(msg.height),
                    msg.source,
                )
            )

rospy.init_node("stage1_detector_only_probe", anonymous=True)
rospy.Subscriber("/follow/detector/person_target", Target2D, cb, queue_size=10)

deadline = time.time() + probe_sec
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    rate.sleep()

print("[detector_probe] msgs=%d valid=%d" % (seen["msgs"], seen["valid"]))
for idx, sample in enumerate(seen["samples"]):
    print(
        "[detector_sample_%d] conf=%s cx=%s cy=%s width=%s height=%s src=%s"
        % ((idx,) + sample)
    )
PY
EOF
