#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
FCU_URL="${FCU_URL:-/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:921600}"
GCS_URL="${GCS_URL:-}"
ODOM_LAUNCH="${ODOM_LAUNCH:-stage1_mid360_faster_lio_hw.launch}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail

cleanup() {
  rosservice call /mavros/cmd/arming "{value: false}" >/dev/null 2>&1 || true
  kill "\${BENCH_PID:-}" "\${ODOM_PID:-}" "\${MAV_PID:-}" "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
  wait "\${BENCH_PID:-}" >/dev/null 2>&1 || true
  wait "\${ODOM_PID:-}" >/dev/null 2>&1 || true
  wait "\${MAV_PID:-}" >/dev/null 2>&1 || true
  wait "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

roscore >/tmp/stage1_offboard_gate_roscore.log 2>&1 &
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

roslaunch --wait human_follow_bringup stage1_px4_mavros.launch fcu_url:='${FCU_URL}' gcs_url:='${GCS_URL}' >/tmp/stage1_offboard_gate_mavros.log 2>&1 &
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

rospy.init_node("stage1_offboard_gate_connected_waiter", anonymous=True)
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

roslaunch --wait human_follow_bringup ${ODOM_LAUNCH} >/tmp/stage1_offboard_gate_odom.log 2>&1 &
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

rospy.init_node("stage1_offboard_gate_estimator_waiter", anonymous=True)
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

roslaunch --wait human_follow_bringup stage1_offboard_gate_bench.launch >/tmp/stage1_offboard_gate_bench.log 2>&1 &
BENCH_PID=\$!

echo "[set_mode] STABILIZED"
rosservice call /mavros/set_mode "{base_mode: 0, custom_mode: 'STABILIZED'}"

echo "[arm] STABILIZED"
rosservice call /mavros/cmd/arming "{value: true}"

python3 - <<'PY'
import sys
import time

import rospy
from mavros_msgs.msg import State

seen = {"armed": False, "mode": None, "last": None}

def cb(msg):
    seen["mode"] = msg.mode
    seen["last"] = msg.armed
    if msg.armed and msg.mode == "STABILIZED":
        seen["armed"] = True

rospy.init_node("stage1_offboard_gate_arm_waiter", anonymous=True)
rospy.Subscriber("/mavros/state", State, cb, queue_size=10)
deadline = time.time() + 6.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["armed"]:
        print("[armed_ok] mode=%s" % seen["mode"])
        sys.exit(0)
    rate.sleep()
print("[FAIL] stabilized arm not observed last_mode=%s last_armed=%s" % (seen["mode"], seen["last"]), file=sys.stderr)
sys.exit(1)
PY

python3 - <<'PY'
import sys
import time

import rospy
from mavros_msgs.msg import State

seen = {"offboard": False, "mode": None, "armed": None}

def cb(msg):
    seen["mode"] = msg.mode
    seen["armed"] = msg.armed
    if msg.mode == "OFFBOARD" and msg.armed:
        seen["offboard"] = True

rospy.init_node("stage1_offboard_gate_mode_waiter", anonymous=True)
rospy.Subscriber("/mavros/state", State, cb, queue_size=10)
deadline = time.time() + 10.0
rate = rospy.Rate(20)
while time.time() < deadline and not rospy.is_shutdown():
    if seen["offboard"]:
        print("[offboard_ok] mode=%s armed=%s" % (seen["mode"], seen["armed"]))
        sys.exit(0)
    rate.sleep()
print("[FAIL] offboard mode not observed last_mode=%s last_armed=%s" % (seen["mode"], seen["armed"]), file=sys.stderr)
sys.exit(1)
PY
EOF

echo "[PASS] remote offboard gate probe passed"
