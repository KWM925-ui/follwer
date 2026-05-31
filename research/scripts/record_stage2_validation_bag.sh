#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR="${1:-research/runs/stage2_rosbags}"
RUN_NAME="${2:-stage2_validation_$(date +%Y%m%d_%H%M%S)}"

if ! command -v rosbag >/dev/null 2>&1; then
  echo "rosbag is not available. Source Ubuntu 20.04 ROS1/catkin setup first." >&2
  exit 2
fi

mkdir -p "${OUTPUT_DIR}"

exec rosbag record \
  -O "${OUTPUT_DIR}/${RUN_NAME}.bag" \
  /follow/fusion/target_world \
  /follow/lio/odom \
  /follow/stage2/state \
  /follow/stage2/goal \
  /follow/stage2/debug_goal_path \
  /move_base_simple/goal \
  /follow/stage2/ego_position_cmd \
  /follow/stage2/offboard/setpoint \
  /mavros/setpoint_raw/local
