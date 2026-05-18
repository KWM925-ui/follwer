#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video0}"
FCU_URL="${FCU_URL:-/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:921600}"
GCS_URL="${GCS_URL:-}"
ODOM_LAUNCH="${ODOM_LAUNCH:-stage1_mid360_faster_lio_hw.launch}"
TARGET_TIMEOUT_SEC="${TARGET_TIMEOUT_SEC:-30.0}"
OFFBOARD_TIMEOUT_SEC="${OFFBOARD_TIMEOUT_SEC:-12.0}"
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
  rosservice call /mavros/cmd/arming "{value: false}" >/dev/null 2>&1 || true
  kill "\${FOLLOW_PID:-}" "\${ODOM_PID:-}" "\${MAV_PID:-}" "\${USB_PID:-}" "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
  wait "\${FOLLOW_PID:-}" >/dev/null 2>&1 || true
  wait "\${ODOM_PID:-}" >/dev/null 2>&1 || true
  wait "\${MAV_PID:-}" >/dev/null 2>&1 || true
  wait "\${USB_PID:-}" >/dev/null 2>&1 || true
  wait "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

roscore >/tmp/stage1_live_cutin_roscore.log 2>&1 &
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

roslaunch --wait human_follow_bringup stage1_usb_cam.launch video_device:='${VIDEO_DEVICE}' >/tmp/stage1_live_cutin_usb.log 2>&1 &
USB_PID=\$!

python3 - <<'PY'
import sys
import time

import rospy
from sensor_msgs.msg import Image

seen = {"image": 0}

def cb(_msg):
    seen["image"] += 1

rospy.init_node("stage1_live_cutin_image_waiter", anonymous=True)
rospy.Subscriber("/camera/usb_cam/image_raw", Image, cb, queue_size=1)
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

roslaunch --wait human_follow_bringup stage1_px4_mavros.launch fcu_url:='${FCU_URL}' gcs_url:='${GCS_URL}' >/tmp/stage1_live_cutin_mavros.log 2>&1 &
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

rospy.init_node("stage1_live_cutin_connected_waiter", anonymous=True)
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

roslaunch --wait human_follow_bringup ${ODOM_LAUNCH} >/tmp/stage1_live_cutin_odom.log 2>&1 &
ODOM_PID=\$!
sleep 5

python3 - <<'PY'
import sys
import time

import rospy
from mavros_msgs.msg import EstimatorStatus

seen = {"valid": False, "last": None}

def cb(msg):
    seen["last"] = (
        msg.velocity_horiz_status_flag,
        msg.velocity_vert_status_flag,
        msg.pos_horiz_rel_status_flag,
        msg.pos_vert_abs_status_flag,
    )
    if all(seen["last"]):
        seen["valid"] = True

rospy.init_node("stage1_live_cutin_estimator_waiter", anonymous=True)
rospy.Subscriber("/mavros/estimator_status", EstimatorStatus, cb, queue_size=10)
deadline = time.time() + 12.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["valid"]:
        print("[estimator_ok] %s" % (seen["last"],))
        sys.exit(0)
    rate.sleep()
print("[FAIL] estimator flags never became valid last=%s" % (seen["last"],), file=sys.stderr)
sys.exit(1)
PY

roslaunch --wait human_follow_bringup stage1_live_px4_to_mavros.launch image_rotation_deg:='${IMAGE_ROTATION_DEG}' flip_horizontal:='${FLIP_HORIZONTAL}' flip_vertical:='${FLIP_VERTICAL}' >/tmp/stage1_live_cutin_follow.log 2>&1 &
FOLLOW_PID=\$!

echo "[set_mode] STABILIZED"
rosservice call /mavros/set_mode "{base_mode: 0, custom_mode: 'STABILIZED'}" >/dev/null

echo "[arm] STABILIZED"
rosservice call /mavros/cmd/arming "{value: true}" >/dev/null

python3 - <<'PY'
import sys
import time

import rospy
from mavros_msgs.msg import State

seen = {"armed": False, "mode": None, "last_armed": None}

def cb(msg):
    seen["mode"] = msg.mode
    seen["last_armed"] = msg.armed
    if msg.armed:
        seen["armed"] = True

rospy.init_node("stage1_live_cutin_arm_waiter", anonymous=True)
rospy.Subscriber("/mavros/state", State, cb, queue_size=10)
deadline = time.time() + 6.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["armed"]:
        print("[armed_ok] mode=%s" % seen["mode"])
        sys.exit(0)
    rate.sleep()
print("[FAIL] armed state not observed last_mode=%s last_armed=%s" % (seen["mode"], seen["last_armed"]), file=sys.stderr)
sys.exit(1)
PY

python3 - <<'PY'
import math
import sys
import time

import rospy
from human_follow_msgs.msg import FollowCommand, FollowState
from mavros_msgs.msg import State

target_timeout_sec = float("${TARGET_TIMEOUT_SEC}")
offboard_timeout_sec = float("${OFFBOARD_TIMEOUT_SEC}")
deadline = time.time() + max(target_timeout_sec, offboard_timeout_sec)

seen = {
    "command_count": 0,
    "state_count": 0,
    "last_mode": None,
    "last_valid": None,
    "last_forward": None,
    "last_lateral": None,
    "last_yaw_rate": None,
    "last_state": None,
    "last_px4_mode": None,
    "last_px4_armed": None,
    "target_ready": False,
    "target_ready_time": None,
    "offboard_armed": False,
    "offboard_armed_time": None,
}

def cmd_cb(msg):
    now = time.time()
    seen["command_count"] += 1
    seen["last_mode"] = msg.mode
    seen["last_valid"] = msg.valid
    seen["last_forward"] = msg.forward_mps
    seen["last_lateral"] = msg.lateral_mps
    seen["last_yaw_rate"] = msg.yaw_rate_rps
    if (
        not seen["target_ready"]
        and msg.valid
        and msg.mode == "body_velocity_yaw_rate"
        and (abs(float(msg.forward_mps)) + abs(float(msg.lateral_mps)) + abs(float(msg.yaw_rate_rps))) >= 0.05
    ):
        seen["target_ready"] = True
        seen["target_ready_time"] = now
        print(
            "[target_ok] command=%d state=%s mode=%s valid=%s forward=%s lateral=%s yaw_rate=%s"
            % (
                seen["command_count"],
                seen["last_state"],
                seen["last_mode"],
                seen["last_valid"],
                seen["last_forward"],
                seen["last_lateral"],
                seen["last_yaw_rate"],
            )
        )

def follow_state_cb(msg):
    seen["state_count"] += 1
    seen["last_state"] = msg.state_name

def px4_state_cb(msg):
    now = time.time()
    seen["last_px4_mode"] = msg.mode
    seen["last_px4_armed"] = msg.armed
    if not seen["offboard_armed"] and msg.mode == "OFFBOARD" and msg.armed:
        seen["offboard_armed"] = True
        seen["offboard_armed_time"] = now
        print("[offboard_ok] mode=%s armed=%s" % (msg.mode, msg.armed))

rospy.init_node("stage1_live_cutin_combined_waiter", anonymous=True)
rospy.Subscriber("/follow/control/cmd_body", FollowCommand, cmd_cb, queue_size=10)
rospy.Subscriber("/follow/state", FollowState, follow_state_cb, queue_size=10)
rospy.Subscriber("/mavros/state", State, px4_state_cb, queue_size=10)
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["target_ready"] and seen["offboard_armed"]:
        delta = seen["offboard_armed_time"] - seen["target_ready_time"]
        print(
            "[cutin_ok] target_to_offboard_sec=%.3f final_mode=%s final_armed=%s"
            % (delta, seen["last_px4_mode"], seen["last_px4_armed"])
        )
        sys.exit(0)
    rate.sleep()

if not seen["target_ready"]:
    print(
        "[FAIL] target-acquired follow command not observed state=%s mode=%s valid=%s forward=%s lateral=%s yaw_rate=%s"
        % (
            seen["last_state"],
            seen["last_mode"],
            seen["last_valid"],
            seen["last_forward"],
            seen["last_lateral"],
            seen["last_yaw_rate"],
        ),
        file=sys.stderr,
    )
    sys.exit(1)

print(
    "[FAIL] offboard armed not observed after target_ready last_mode=%s last_armed=%s target_state=%s cmd_mode=%s cmd_valid=%s"
    % (
        seen["last_px4_mode"],
        seen["last_px4_armed"],
        seen["last_state"],
        seen["last_mode"],
        seen["last_valid"],
    ),
    file=sys.stderr,
)
sys.exit(1)
PY
EOF

echo "[PASS] remote live follow cut-in probe passed"
