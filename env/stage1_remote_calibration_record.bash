#!/usr/bin/env bash
set -euo pipefail

REMOTE_USER="${REMOTE_USER:-nv}"
REMOTE_HOST="${REMOTE_HOST:-192.168.5.10}"
REMOTE_SSH_TARGET="${REMOTE_USER}@${REMOTE_HOST}"
REMOTE_WORKSPACE_ROOT="${REMOTE_WORKSPACE_ROOT:-/home/nv/human_follow_ws}"
REMOTE_BAG_ROOT="${REMOTE_BAG_ROOT:-/home/nv/human_follow_ws/data/calibration}"
SESSION_TAG="${SESSION_TAG:-$(date +%Y%m%d_%H%M%S)}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/usb_cam/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/usb_cam/camera_info}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/laserMapping/cloud_registered_body}"
RAW_LIDAR_TOPIC="${RAW_LIDAR_TOPIC:-/livox/lidar}"
IMU_TOPIC="${IMU_TOPIC:-/livox/imu}"
TF_STATIC_TOPIC="${TF_STATIC_TOPIC:-/tf_static}"
ODOM_TOPIC="${ODOM_TOPIC:-/laserMapping/odometry}"
DURATION_SEC="${DURATION_SEC:-0}"
SSH_OPTS=(
  -o BatchMode=yes
  -o ConnectTimeout=5
)

ssh "${SSH_OPTS[@]}" "${REMOTE_SSH_TARGET}" 'bash -s' <<EOF
set -euo pipefail
set +u
source /opt/ros/noetic/setup.bash
source '${REMOTE_WORKSPACE_ROOT}/devel/setup.bash'
set -u

for topic in '${IMAGE_TOPIC}' '${CAMERA_INFO_TOPIC}' '${POINTCLOUD_TOPIC}' '${RAW_LIDAR_TOPIC}' '${IMU_TOPIC}'; do
  if ! timeout 8s rostopic type "\$topic" >/dev/null 2>&1; then
    echo "[FAIL] topic not available: \$topic" >&2
    exit 1
  fi
done

bag_dir='${REMOTE_BAG_ROOT}/${SESSION_TAG}'
mkdir -p "\$bag_dir"
manifest="\$bag_dir/manifest.txt"
precheck_manifest="\$bag_dir/precheck_manifest.txt"
{
  echo "session_tag=${SESSION_TAG}"
  echo "record_start_utc=\$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "image_topic=${IMAGE_TOPIC}"
  echo "camera_info_topic=${CAMERA_INFO_TOPIC}"
  echo "pointcloud_topic=${POINTCLOUD_TOPIC}"
  echo "raw_lidar_topic=${RAW_LIDAR_TOPIC}"
  echo "imu_topic=${IMU_TOPIC}"
  echo "tf_static_topic=${TF_STATIC_TOPIC}"
  echo "odom_topic=${ODOM_TOPIC}"
  echo "image_type=\$(rostopic type '${IMAGE_TOPIC}')"
  echo "camera_info_type=\$(rostopic type '${CAMERA_INFO_TOPIC}')"
  echo "pointcloud_type=\$(rostopic type '${POINTCLOUD_TOPIC}')"
  echo "raw_lidar_type=\$(rostopic type '${RAW_LIDAR_TOPIC}')"
  echo "imu_type=\$(rostopic type '${IMU_TOPIC}')"
  echo "tf_static_type=\$(rostopic type '${TF_STATIC_TOPIC}')"
  echo "odom_type=\$(rostopic type '${ODOM_TOPIC}' || true)"
} > "\$manifest"

timeout 20s rosrun human_follow_bringup calibration_record_monitor_node.py \
  _image_topic:='${IMAGE_TOPIC}' \
  _camera_info_topic:='${CAMERA_INFO_TOPIC}' \
  _pointcloud_topic:='${POINTCLOUD_TOPIC}' \
  _raw_lidar_topic:='${RAW_LIDAR_TOPIC}' \
  _imu_topic:='${IMU_TOPIC}' \
  _tf_static_topic:='${TF_STATIC_TOPIC}' \
  _output_manifest:="\$precheck_manifest"

if ! grep -qx 'result=pass' "\$precheck_manifest"; then
  echo "[FAIL] calibration precheck did not pass" >&2
  cat "\$precheck_manifest" >&2 || true
  exit 1
fi

bag_path="\$bag_dir/stage1_calibration_record.bag"
topics=(
  '${IMAGE_TOPIC}'
  '${CAMERA_INFO_TOPIC}'
  '${POINTCLOUD_TOPIC}'
  '${RAW_LIDAR_TOPIC}'
  '${IMU_TOPIC}'
  '${TF_STATIC_TOPIC}'
  '${ODOM_TOPIC}'
)

if [ '${DURATION_SEC}' = '0' ]; then
  echo "[recording] bag_path=\$bag_path"
  exec rosbag record --lz4 -O "\$bag_path" "\${topics[@]}"
fi

echo "[recording] bag_path=\$bag_path duration_sec=${DURATION_SEC}"
timeout '${DURATION_SEC}' rosbag record --lz4 -O "\$bag_path" "\${topics[@]}" || true
echo "[done] bag_path=\$bag_path"
EOF
