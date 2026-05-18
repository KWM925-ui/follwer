#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
FCU_URL="${FCU_URL:-/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:921600}"
GCS_URL="${GCS_URL:-}"
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
  kill "\${FOLLOW_PID:-}" "\${MAV_PID:-}" "\${USB_PID:-}" "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
  wait "\${FOLLOW_PID:-}" >/dev/null 2>&1 || true
  wait "\${MAV_PID:-}" >/dev/null 2>&1 || true
  wait "\${USB_PID:-}" >/dev/null 2>&1 || true
  wait "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

roscore >/tmp/stage1_split_roscore.log 2>&1 &
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

roslaunch --wait human_follow_bringup stage1_usb_cam.launch video_device:='${VIDEO_DEVICE}' >/tmp/stage1_split_usb_cam.log 2>&1 &
USB_PID=\$!

python3 - <<'PY'
import os
import sys
import time

import rospy
from sensor_msgs.msg import Image

seen = {"image": 0}

def image_cb(_msg):
    seen["image"] += 1

rospy.init_node("stage1_split_image_waiter", anonymous=True)
sub = rospy.Subscriber("/camera/usb_cam/image_raw", Image, image_cb, queue_size=1)
deadline = time.time() + 12.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["image"] > 0:
        print("[image_ok] count=%d" % seen["image"])
        sys.exit(0)
    rate.sleep()
print("[FAIL] image topic did not publish in time", file=sys.stderr)
sys.exit(1)
PY

roslaunch --wait human_follow_bringup stage1_px4_mavros.launch fcu_url:='${FCU_URL}' gcs_url:='${GCS_URL}' >/tmp/stage1_split_mavros.log 2>&1 &
MAV_PID=\$!

python3 - <<'PY'
import sys
import time

import rospy
from mavros_msgs.msg import State

seen = {"count": 0, "connected_true": False, "last": None}

def state_cb(msg):
    seen["count"] += 1
    seen["last"] = msg.connected
    if msg.connected:
        seen["connected_true"] = True

rospy.init_node("stage1_split_mavros_waiter", anonymous=True)
sub = rospy.Subscriber("/mavros/state", State, state_cb, queue_size=10)
deadline = time.time() + 20.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["connected_true"]:
        print("[mavros_ok] count=%d last=%s" % (seen["count"], seen["last"]))
        sys.exit(0)
    rate.sleep()
print("[FAIL] mavros never reached connected=True count=%d last=%s" % (seen["count"], seen["last"]), file=sys.stderr)
sys.exit(1)
PY

roslaunch --wait human_follow_bringup stage1_live_px4_to_mavros.launch image_rotation_deg:='${IMAGE_ROTATION_DEG}' flip_horizontal:='${FLIP_HORIZONTAL}' flip_vertical:='${FLIP_VERTICAL}' >/tmp/stage1_split_follow.log 2>&1 &
FOLLOW_PID=\$!

python3 - <<'PY'
import sys
import time

import rospy
from human_follow_msgs.msg import FollowCommand
from mavros_msgs.msg import PositionTarget

seen = {
    "command": 0,
    "setpoint": 0,
    "last_mode": None,
    "last_vx": None,
    "last_vy": None,
    "last_yaw_rate": None,
}

def cmd_cb(msg):
    seen["command"] += 1
    seen["last_mode"] = msg.mode

def sp_cb(msg):
    seen["setpoint"] += 1
    seen["last_vx"] = msg.velocity.x
    seen["last_vy"] = msg.velocity.y
    seen["last_yaw_rate"] = msg.yaw_rate

rospy.init_node("stage1_split_follow_waiter", anonymous=True)
cmd_sub = rospy.Subscriber("/follow/control/cmd_body", FollowCommand, cmd_cb, queue_size=10)
sp_sub = rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, sp_cb, queue_size=10)
deadline = time.time() + 20.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["command"] >= 1 and seen["setpoint"] >= 3:
        print(
            "[follow_ok] command=%d setpoint=%d mode=%s vx=%s vy=%s yaw_rate=%s"
            % (
                seen["command"],
                seen["setpoint"],
                seen["last_mode"],
                seen["last_vx"],
                seen["last_vy"],
                seen["last_yaw_rate"],
            )
        )
        sys.exit(0)
    rate.sleep()
print(
    "[FAIL] follower chain incomplete command=%d setpoint=%d mode=%s"
    % (seen["command"], seen["setpoint"], seen["last_mode"]),
    file=sys.stderr,
)
sys.exit(1)
PY
EOF

echo "[PASS] remote split runtime smoke passed"
