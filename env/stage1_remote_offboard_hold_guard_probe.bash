#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
FCU_URL="${FCU_URL:-/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:921600}"
GCS_URL="${GCS_URL:-}"
ODOM_LAUNCH="${ODOM_LAUNCH:-stage1_mid360_faster_lio_hw.launch}"
OBSERVE_SEC="${OBSERVE_SEC:-8.0}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail

cleanup() {
  rosservice call /mavros/cmd/arming "{value: false}" >/dev/null 2>&1 || true
  kill "\${FOLLOW_PID:-}" "\${ODOM_PID:-}" "\${MAV_PID:-}" "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
  wait "\${FOLLOW_PID:-}" >/dev/null 2>&1 || true
  wait "\${ODOM_PID:-}" >/dev/null 2>&1 || true
  wait "\${MAV_PID:-}" >/dev/null 2>&1 || true
  wait "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

roscore >/tmp/stage1_offboard_hold_guard_roscore.log 2>&1 &
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

roslaunch --wait human_follow_bringup stage1_px4_mavros.launch fcu_url:='${FCU_URL}' gcs_url:='${GCS_URL}' >/tmp/stage1_offboard_hold_guard_mavros.log 2>&1 &
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

rospy.init_node("stage1_offboard_hold_guard_connected_waiter", anonymous=True)
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

roslaunch --wait human_follow_bringup ${ODOM_LAUNCH} >/tmp/stage1_offboard_hold_guard_odom.log 2>&1 &
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

rospy.init_node("stage1_offboard_hold_guard_estimator_waiter", anonymous=True)
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

roslaunch --wait human_follow_bringup stage1_live_px4_to_mavros.launch >/tmp/stage1_offboard_hold_guard_follow.log 2>&1 &
FOLLOW_PID=\$!

python3 - <<'PY'
import math
import sys
import time

import rospy
from human_follow_msgs.msg import FollowCommand
from mavros_msgs.msg import PositionTarget

seen = {
    "command_count": 0,
    "setpoint_count": 0,
    "last_mode": None,
    "last_valid": None,
    "last_forward": None,
    "last_lateral": None,
    "last_yaw_cmd": None,
    "last_vx": None,
    "last_vy": None,
    "last_vz": None,
    "last_yaw_rate": None,
}

def close_zero(value):
    return math.isclose(float(value), 0.0, abs_tol=1e-5)

def cmd_cb(msg):
    seen["command_count"] += 1
    seen["last_mode"] = msg.mode
    seen["last_valid"] = msg.valid
    seen["last_forward"] = msg.forward_mps
    seen["last_lateral"] = msg.lateral_mps
    seen["last_yaw_cmd"] = msg.yaw_rate_rps

def sp_cb(msg):
    seen["setpoint_count"] += 1
    seen["last_vx"] = msg.velocity.x
    seen["last_vy"] = msg.velocity.y
    seen["last_vz"] = msg.velocity.z
    seen["last_yaw_rate"] = msg.yaw_rate

rospy.init_node("stage1_offboard_hold_guard_hold_waiter", anonymous=True)
rospy.Subscriber("/follow/control/cmd_body", FollowCommand, cmd_cb, queue_size=10)
rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, sp_cb, queue_size=10)
deadline = time.time() + 12.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["command_count"] >= 3 and seen["setpoint_count"] >= 5:
        cmd_ok = (
            seen["last_mode"] == "hold"
            and seen["last_valid"] is False
            and close_zero(seen["last_forward"])
            and close_zero(seen["last_lateral"])
            and close_zero(seen["last_yaw_cmd"])
        )
        sp_ok = (
            close_zero(seen["last_vx"])
            and close_zero(seen["last_vy"])
            and close_zero(seen["last_vz"])
            and close_zero(seen["last_yaw_rate"])
        )
        if cmd_ok and sp_ok:
            print(
                "[hold_ok] command=%d setpoint=%d mode=%s valid=%s vx=%s vy=%s yaw_rate=%s"
                % (
                    seen["command_count"],
                    seen["setpoint_count"],
                    seen["last_mode"],
                    seen["last_valid"],
                    seen["last_vx"],
                    seen["last_vy"],
                    seen["last_yaw_rate"],
                )
            )
            sys.exit(0)
    rate.sleep()

print(
    "[FAIL] hold path not stable command=%d setpoint=%d mode=%s valid=%s vx=%s vy=%s yaw_rate=%s"
    % (
        seen["command_count"],
        seen["setpoint_count"],
        seen["last_mode"],
        seen["last_valid"],
        seen["last_vx"],
        seen["last_vy"],
        seen["last_yaw_rate"],
    ),
    file=sys.stderr,
)
sys.exit(1)
PY

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
    if msg.armed and msg.mode == "STABILIZED":
        seen["armed"] = True

rospy.init_node("stage1_offboard_hold_guard_arm_waiter", anonymous=True)
rospy.Subscriber("/mavros/state", State, cb, queue_size=10)
deadline = time.time() + 6.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["armed"]:
        print("[armed_ok] mode=%s" % seen["mode"])
        sys.exit(0)
    rate.sleep()
print("[FAIL] stabilized arm not observed last_mode=%s last_armed=%s" % (seen["mode"], seen["last_armed"]), file=sys.stderr)
sys.exit(1)
PY

python3 - <<'PY'
import sys
import time

import rospy
from mavros_msgs.msg import State

observe_sec = float("${OBSERVE_SEC}")
seen = {"entered_offboard": False, "last_mode": None, "last_armed": None}

def cb(msg):
    seen["last_mode"] = msg.mode
    seen["last_armed"] = msg.armed
    if msg.mode == "OFFBOARD":
        seen["entered_offboard"] = True

rospy.init_node("stage1_offboard_hold_guard_mode_waiter", anonymous=True)
rospy.Subscriber("/mavros/state", State, cb, queue_size=10)
deadline = time.time() + observe_sec
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["entered_offboard"]:
        print("[FAIL] offboard entered unexpectedly last_mode=%s last_armed=%s" % (seen["last_mode"], seen["last_armed"]), file=sys.stderr)
        sys.exit(1)
    rate.sleep()

print("[hold_guard_ok] observe_sec=%.1f last_mode=%s armed=%s" % (observe_sec, seen["last_mode"], seen["last_armed"]))
sys.exit(0)
PY

if grep -q "offboard_mode_gate request #" /tmp/stage1_offboard_hold_guard_follow.log; then
  echo "[FAIL] gate emitted OFFBOARD request during hold guard" >&2
  exit 1
fi
EOF

echo "[PASS] remote offboard hold guard probe passed"
