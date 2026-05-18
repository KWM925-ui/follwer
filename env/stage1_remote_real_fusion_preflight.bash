#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/laserMapping/cloud_registered_body}"
INTRINSICS_YAML="${INTRINSICS_YAML:-}"
EXTRINSICS_YAML="${EXTRINSICS_YAML:-}"
CAMERA_BODY_YAML="${CAMERA_BODY_YAML:-}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

if [ -z "${INTRINSICS_YAML}" ] || [ -z "${EXTRINSICS_YAML}" ] || [ -z "${CAMERA_BODY_YAML}" ]; then
  echo "[FAIL] set INTRINSICS_YAML, EXTRINSICS_YAML, and CAMERA_BODY_YAML before real-fusion preflight" >&2
  exit 1
fi

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

for path in '${INTRINSICS_YAML}' '${EXTRINSICS_YAML}' '${CAMERA_BODY_YAML}'; do
  if [ ! -f "\$path" ]; then
    echo "[FAIL] missing calibration file \$path" >&2
    exit 1
  fi
done

python3 '${REMOTE_WORKSPACE_ROOT}/src/human_follow_fusion/scripts/calibration_asset_validator.py' \
  --intrinsics '${INTRINSICS_YAML}' \
  --extrinsics '${EXTRINSICS_YAML}' \
  --camera-body '${CAMERA_BODY_YAML}'

roscore >/tmp/stage1_real_fusion_preflight_roscore.log 2>&1 &
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

roslaunch --wait human_follow_bringup stage1_mid360_faster_lio_hw.launch >/tmp/stage1_real_fusion_preflight_lio.log 2>&1 &
LIO_PID=\$!
sleep 8

topic_type="\$(rostopic type '${POINTCLOUD_TOPIC}' || true)"
echo "[pointcloud_topic] ${POINTCLOUD_TOPIC}"
echo "[pointcloud_type] \$topic_type"
if [ "\$topic_type" != "sensor_msgs/PointCloud2" ]; then
  echo "[FAIL] ${POINTCLOUD_TOPIC} is not sensor_msgs/PointCloud2" >&2
  exit 1
fi

timeout 10s rostopic echo -n 1 '${POINTCLOUD_TOPIC}/header' >/dev/null
echo "[pointcloud_ok] ${POINTCLOUD_TOPIC}"

roslaunch --nodes human_follow_bringup stage1_live_px4_to_mavros_real_fusion.launch \
  intrinsics_yaml:='${INTRINSICS_YAML}' \
  extrinsics_yaml:='${EXTRINSICS_YAML}' \
  camera_body_yaml:='${CAMERA_BODY_YAML}' \
  pointcloud_topic:='${POINTCLOUD_TOPIC}' >/dev/null
echo "[launch_parse_ok] stage1_live_px4_to_mavros_real_fusion.launch"
EOF

echo "[PASS] remote real fusion preflight passed"
