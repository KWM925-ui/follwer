#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-${DEFAULT_WORKSPACE_ROOT}}"
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/noetic/setup.bash}"
MAVROS_OVERLAY_PREFIX_DEFAULT="${MAVROS_OVERLAY_PREFIX:-/home/coco/.local/ros_noetic_overlay/opt/ros/noetic}"

if [ ! -f "${ROS_DISTRO_SETUP}" ]; then
  echo "[FAIL] ROS setup not found: ${ROS_DISTRO_SETUP}" >&2
  exit 1
fi

if [ ! -f "${WORKSPACE_ROOT}/devel/setup.bash" ]; then
  echo "[FAIL] Workspace devel setup missing: ${WORKSPACE_ROOT}/devel/setup.bash" >&2
  exit 1
fi

set +u
source "${ROS_DISTRO_SETUP}"
source "${WORKSPACE_ROOT}/devel/setup.bash"
set -u

if [ -f "${MAVROS_OVERLAY_PREFIX_DEFAULT}/share/mavros_msgs/package.xml" ] && [ -f "${WORKSPACE_ROOT}/env/source_local_mavros_overlay.bash" ]; then
  # shellcheck disable=SC1090
  set +u
  source "${WORKSPACE_ROOT}/env/source_local_mavros_overlay.bash"
  set -u
elif [ -f "${MAVROS_OVERLAY_PREFIX_DEFAULT}/share/mavros_msgs/package.xml" ]; then
  export MAVROS_OVERLAY_PREFIX="${MAVROS_OVERLAY_PREFIX_DEFAULT}"
  export CMAKE_PREFIX_PATH="${MAVROS_OVERLAY_PREFIX}:${CMAKE_PREFIX_PATH:-}"
  export ROS_PACKAGE_PATH="${MAVROS_OVERLAY_PREFIX}/share:${ROS_PACKAGE_PATH:-}"
  export PYTHONPATH="${MAVROS_OVERLAY_PREFIX}/lib/python3/dist-packages:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="${MAVROS_OVERLAY_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  export PKG_CONFIG_PATH="${MAVROS_OVERLAY_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
fi

echo "[INFO] rospack human_follow_px4_bridge"
rospack find human_follow_px4_bridge >/dev/null

echo "[INFO] rospack mavros_msgs"
rospack find mavros_msgs >/dev/null

echo "[INFO] rosmsg human_follow_msgs/FollowCommand"
rosmsg show human_follow_msgs/FollowCommand >/dev/null

echo "[INFO] rosmsg mavros_msgs/PositionTarget"
rosmsg show mavros_msgs/PositionTarget >/dev/null

echo "[INFO] roslaunch parse stage1_live_px4.launch"
roslaunch --nodes human_follow_bringup stage1_live_px4.launch >/dev/null

echo "[INFO] roslaunch parse stage1_offline_px4.launch"
roslaunch --nodes human_follow_bringup stage1_offline_px4.launch >/dev/null

echo "[INFO] roslaunch parse stage1_offline_video_px4.launch"
roslaunch --nodes human_follow_bringup stage1_offline_video_px4.launch >/dev/null

echo "[INFO] roslaunch parse stage1_offline_video_px4_smoke.launch"
roslaunch --nodes human_follow_bringup stage1_offline_video_px4_smoke.launch >/dev/null

echo "[INFO] roslaunch parse stage1_live_px4_idle_regression.launch"
roslaunch --nodes human_follow_bringup stage1_live_px4_idle_regression.launch >/dev/null

echo "[INFO] roslaunch parse stage1_offboard_gate_regression.launch"
roslaunch --nodes human_follow_bringup stage1_offboard_gate_regression.launch >/dev/null

echo "[INFO] roslaunch parse stage1_px4_mavros_sitl.launch"
roslaunch --nodes human_follow_bringup stage1_px4_mavros_sitl.launch >/dev/null

echo "[INFO] roslaunch parse stage1_sitl_synthetic_follow.launch"
roslaunch --nodes human_follow_bringup stage1_sitl_synthetic_follow.launch >/dev/null

echo "[PASS] stage1 PX4 software preflight passed"
