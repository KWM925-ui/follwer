#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
ODOM_LAUNCH="${ODOM_LAUNCH:-stage1_mid360_faster_lio_hw.launch}"
LIO_ODOM_TOPIC="${LIO_ODOM_TOPIC:-/laserMapping/odometry}"
FOLLOW_ODOM_TOPIC="${FOLLOW_ODOM_TOPIC:-/follow/lio/odom}"
MAVROS_ODOM_TOPIC="${MAVROS_ODOM_TOPIC:-/mavros/odometry/out}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail

cleanup() {
  kill "\${LIO_PID:-}" "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
  wait "\${LIO_PID:-}" >/dev/null 2>&1 || true
  wait "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

roscore >/tmp/stage1_real_odom_smoke_roscore.log 2>&1 &
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

roslaunch --wait human_follow_bringup ${ODOM_LAUNCH} >/tmp/stage1_real_odom_smoke.log 2>&1 &
LIO_PID=\$!

for topic in '${LIO_ODOM_TOPIC}' '${FOLLOW_ODOM_TOPIC}' '${MAVROS_ODOM_TOPIC}'; do
  echo "[WAIT] \$topic"
  timeout 30s rostopic echo -n 1 "\$topic" >/dev/null
done
EOF

echo "[PASS] remote real-odom smoke passed"
