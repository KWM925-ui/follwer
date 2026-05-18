#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
FCU_URL="${FCU_URL:-/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:921600}"
GCS_URL="${GCS_URL:-}"
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

roscore >/tmp/stage1_offboard_probe_roscore.log 2>&1 &
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

roslaunch --wait human_follow_bringup stage1_usb_cam.launch video_device:='${VIDEO_DEVICE}' >/tmp/stage1_offboard_probe_usb.log 2>&1 &
USB_PID=\$!
sleep 4

roslaunch --wait human_follow_bringup stage1_px4_mavros.launch fcu_url:='${FCU_URL}' gcs_url:='${GCS_URL}' >/tmp/stage1_offboard_probe_mavros.log 2>&1 &
MAV_PID=\$!

python3 - <<'PY'
import sys
import time

import rospy
from mavros_msgs.msg import State

seen = {"connected_true": False}

def cb(msg):
    if msg.connected:
        seen["connected_true"] = True

rospy.init_node("stage1_offboard_probe_connected_waiter", anonymous=True)
rospy.Subscriber("/mavros/state", State, cb, queue_size=10)
deadline = time.time() + 20.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["connected_true"]:
        print("[connected_ok]")
        sys.exit(0)
    rate.sleep()
print("[FAIL] mavros never reached connected=True", file=sys.stderr)
sys.exit(1)
PY

roslaunch --wait human_follow_bringup stage1_live_px4_to_mavros.launch >/tmp/stage1_offboard_probe_follow.log 2>&1 &
FOLLOW_PID=\$!

python3 - <<'PY'
import sys
import time

import rospy
from mavros_msgs.msg import PositionTarget

count = {"setpoint": 0}

def cb(_msg):
    count["setpoint"] += 1

rospy.init_node("stage1_offboard_probe_setpoint_waiter", anonymous=True)
rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, cb, queue_size=20)
deadline = time.time() + 15.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if count["setpoint"] >= 10:
        print("[setpoint_ok] count=%d" % count["setpoint"])
        sys.exit(0)
    rate.sleep()
print("[FAIL] insufficient raw setpoints count=%d" % count["setpoint"], file=sys.stderr)
sys.exit(1)
PY

echo "[set_mode]"
rosservice call /mavros/set_mode "{base_mode: 0, custom_mode: 'OFFBOARD'}"

python3 - <<'PY'
import sys
import time

import rospy
from mavros_msgs.msg import State

seen = {"mode": None, "offboard": False}

def cb(msg):
    seen["mode"] = msg.mode
    if msg.mode == "OFFBOARD":
        seen["offboard"] = True

rospy.init_node("stage1_offboard_probe_mode_waiter", anonymous=True)
rospy.Subscriber("/mavros/state", State, cb, queue_size=10)
deadline = time.time() + 10.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["offboard"]:
        print("[mode_ok] OFFBOARD")
        sys.exit(0)
    rate.sleep()
print("[FAIL] offboard mode not observed last_mode=%s" % seen["mode"], file=sys.stderr)
sys.exit(1)
PY
EOF

echo "[PASS] remote offboard mode probe passed"
