#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/coco/follwer_ws"
RUN_ROOT="$ROOT/artifacts/live_demo_isolated"
PID_DIR="$RUN_ROOT/pids"
ROS_HOME_DIR="$RUN_ROOT/ros_home_verify6"
MASTER_URI="http://127.0.0.1:11412"
ROS_HOST="127.0.0.1"
CLEAN_SCRIPT="$RUN_ROOT/clean_stage2_runtime_residue.sh"

LAUNCH_PID_FILE="$PID_DIR/stage2_manual_replan_showcase.pid"
RVIZ_PID_FILE="$PID_DIR/stage2_manual_replan_showcase_rviz.pid"
BAG_PID_FILE="$PID_DIR/stage2_manual_replan_showcase_bag.pid"

kill_pid_hard() {
  local pid="$1"
  [[ -z "$pid" ]] && return 0
  kill -INT "$pid" 2>/dev/null || true
  sleep 0.5
  kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
  sleep 0.5
  kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
}

is_self_or_ancestor() {
  local pid="$1"
  local cursor="$$"
  while [[ -n "$cursor" ]] && [[ "$cursor" =~ ^[0-9]+$ ]] && [[ "$cursor" -gt 1 ]]; do
    [[ "$pid" == "$cursor" ]] && return 0
    cursor="$(ps -o ppid= -p "$cursor" 2>/dev/null | tr -d '[:space:]' || true)"
  done
  return 1
}

kill_pattern_until_gone() {
  local pattern="$1"
  local rounds=0
  while true; do
    local pids
    pids="$(
      ps -eo pid=,comm=,args= \
        | awk -v pat="$pattern" '
            $2 ~ /^(bash|timeout|awk|rg|pgrep|ps|sed)$/ { next }
            $0 ~ pat { print $1 }
          ' \
        || true
    )"
    [[ -z "$pids" ]] && break
    while read -r pid; do
      if [[ -n "$pid" ]] && ! is_self_or_ancestor "$pid"; then
        kill_pid_hard "$pid"
      fi
    done <<< "$pids"
    rounds=$((rounds + 1))
    [[ "$rounds" -ge 6 ]] && break
    sleep 0.5
  done
}

kill_rosnodes_on_master() {
  env \
    ROS_MASTER_URI="$MASTER_URI" \
    ROS_IP="$ROS_HOST" \
    ROS_HOSTNAME="$ROS_HOST" \
    ROS_HOME="$ROS_HOME_DIR" \
    bash -lc "source /opt/ros/noetic/setup.bash && rosnode list 2>/dev/null | grep -v '^/rosout$' | while read -r node; do [[ -n \"\$node\" ]] && rosnode kill \"\$node\" >/dev/null 2>&1 || true; done" \
    >/dev/null 2>&1 || true
}

for file in "$BAG_PID_FILE" "$RVIZ_PID_FILE" "$LAUNCH_PID_FILE"; do
  if [[ -f "$file" ]]; then
    pid="$(cat "$file")"
    if kill -0 "$pid" 2>/dev/null; then
      kill_pid_hard "$pid"
    fi
    rm -f "$file"
  fi
done

kill_rosnodes_on_master
kill_pattern_until_gone "roslaunch /home/coco/follwer_ws/src/human_follow_bringup/launch/stage2_real_ego_visual_demo_manual_replan_showcase.launch"
kill_pattern_until_gone "rosmaster --core -p 11412"
kill_pattern_until_gone "${ROS_HOME_DIR}/log/"
kill_pattern_until_gone "/opt/ros/noetic/lib/rosout/rosout .*${ROS_HOME_DIR}/log/"
kill_pattern_until_gone "rviz .*stage2_real_ego_integrated.rviz"
kill_pattern_until_gone "rosbag record -O /home/coco/follwer_ws/artifacts/live_demo_isolated/recordings/stage2_acceptance_"

rm -f "$PID_DIR"/stage2_manual_replan_showcase*.pid 2>/dev/null || true
"$CLEAN_SCRIPT" >/dev/null 2>&1 || true

echo "stage2 manual replan showcase stopped"
