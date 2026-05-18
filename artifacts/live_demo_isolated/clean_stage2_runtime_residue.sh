#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/coco/follwer_ws"
DEMO_ROOT="$ROOT/artifacts/live_demo_isolated"
PID_DIR="$DEMO_ROOT/pids"
ACTIVE_ROS_HOMES=(
  "$DEMO_ROOT/ros_home"
  "$DEMO_ROOT/ros_home_verify6"
)
OBSOLETE_ROS_HOMES=(
  "$DEMO_ROOT/ros_home_verify"
  "$DEMO_ROOT/ros_home_verify2"
  "$DEMO_ROOT/ros_home_verify3"
  "$DEMO_ROOT/ros_home_verify4"
  "$DEMO_ROOT/ros_home_verify5"
)

prune_dead_pid_file() {
  local file="$1"
  [[ -f "$file" ]] || return 0
  local pid=""
  pid="$(tr -d '[:space:]' < "$file" 2>/dev/null || true)"
  if [[ -z "$pid" ]] || ! [[ "$pid" =~ ^[0-9]+$ ]] || ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$file"
  fi
}

mkdir -p "$PID_DIR"
for dir in "${ACTIVE_ROS_HOMES[@]}"; do
  mkdir -p "$dir"
done

while IFS= read -r file; do
  prune_dead_pid_file "$file"
done < <(
  {
    find "$PID_DIR" -maxdepth 1 -type f -name '*.pid' 2>/dev/null
    for dir in "${ACTIVE_ROS_HOMES[@]}"; do
      find "$dir" -maxdepth 1 -type f -name '*.pid' 2>/dev/null
    done
  } | sort -u
)

for dir in "${ACTIVE_ROS_HOMES[@]}"; do
  rm -f "$dir"/rospack_cache_* 2>/dev/null || true
done

for dir in "${OBSOLETE_ROS_HOMES[@]}"; do
  rm -rf "$dir"
done

echo "stage2 runtime residue cleaned"
