#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/usb_cam/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/usb_cam/camera_info}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/laserMapping/cloud_registered_body}"
RAW_LIDAR_TOPIC="${RAW_LIDAR_TOPIC:-/livox/lidar}"
IMU_TOPIC="${IMU_TOPIC:-/livox/imu}"
TF_STATIC_TOPIC="${TF_STATIC_TOPIC:-/tf_static}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail

cleanup() {
  kill "\${USB_PID:-}" "\${LIO_PID:-}" "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
  wait "\${USB_PID:-}" >/dev/null 2>&1 || true
  wait "\${LIO_PID:-}" >/dev/null 2>&1 || true
  wait "\${ROSCORE_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

roscore >/tmp/stage1_calibration_capture_smoke_roscore.log 2>&1 &
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

roslaunch --wait human_follow_bringup stage1_usb_cam.launch >/tmp/stage1_calibration_capture_smoke_usb.log 2>&1 &
USB_PID=\$!
roslaunch --wait human_follow_bringup stage1_mid360_faster_lio_hw.launch >/tmp/stage1_calibration_capture_smoke_lio.log 2>&1 &
LIO_PID=\$!
sleep 10

timeout 20s rosrun human_follow_bringup calibration_record_monitor_node.py \
  _image_topic:='${IMAGE_TOPIC}' \
  _camera_info_topic:='${CAMERA_INFO_TOPIC}' \
  _pointcloud_topic:='${POINTCLOUD_TOPIC}' \
  _raw_lidar_topic:='${RAW_LIDAR_TOPIC}' \
  _imu_topic:='${IMU_TOPIC}' \
  _tf_static_topic:='${TF_STATIC_TOPIC}' \
  _output_manifest:=/tmp/stage1_calibration_capture_smoke_manifest.txt

if ! grep -qx 'result=pass' /tmp/stage1_calibration_capture_smoke_manifest.txt; then
  echo "[FAIL] calibration capture smoke precheck did not pass" >&2
  cat /tmp/stage1_calibration_capture_smoke_manifest.txt >&2 || true
  exit 1
fi

echo "[PASS] remote calibration capture smoke passed"
EOF
