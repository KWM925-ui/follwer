#!/usr/bin/env bash

# Source this before ROS tools that need rospack/rosmsg visibility for the
# locally installed MAVROS overlay.

if [ -z "${BASH_VERSION:-}" ]; then
  echo "source_local_mavros_overlay.bash must be sourced from bash" >&2
  return 1 2>/dev/null || exit 1
fi

if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/noetic/setup.bash
fi

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

OVERLAY_PREFIX="${MAVROS_OVERLAY_PREFIX:-/home/coco/.local/ros_noetic_overlay/opt/ros/noetic}"
if [ ! -f "${OVERLAY_PREFIX}/share/mavros_msgs/package.xml" ]; then
  echo "MAVROS overlay not found at ${OVERLAY_PREFIX}" >&2
  return 1 2>/dev/null || exit 1
fi
OVERLAY_ROOT="$(cd "${OVERLAY_PREFIX}/../../.." && pwd)"
OVERLAY_SUPPORT_LIB="${OVERLAY_ROOT}/usr/lib/x86_64-linux-gnu"

export MAVROS_OVERLAY_PREFIX="${OVERLAY_PREFIX}"
export CMAKE_PREFIX_PATH="${OVERLAY_PREFIX}:${CMAKE_PREFIX_PATH:-}"
export ROS_PACKAGE_PATH="${OVERLAY_PREFIX}/share:${ROS_PACKAGE_PATH:-}"
export PYTHONPATH="${OVERLAY_PREFIX}/lib/python3/dist-packages:${PYTHONPATH:-}"
if [ -d "${OVERLAY_SUPPORT_LIB}" ]; then
  export LD_LIBRARY_PATH="${OVERLAY_PREFIX}/lib:${OVERLAY_SUPPORT_LIB}:${LD_LIBRARY_PATH:-}"
else
  export LD_LIBRARY_PATH="${OVERLAY_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
fi
export PKG_CONFIG_PATH="${OVERLAY_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"

if [ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]; then
  # Keep the local workspace visible for rospack/roslaunch while preserving the
  # explicit MAVROS overlay in the inherited environment.
  source "${WORKSPACE_ROOT}/devel/setup.bash"
fi
