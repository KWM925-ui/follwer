#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/coco/follwer_ws"
DEMO_ROOT="$ROOT/artifacts/live_demo_isolated"
PID_DIR="$DEMO_ROOT/pids"
ROS_HOME_DIR="$DEMO_ROOT/ros_home"
MASTER_URI="http://127.0.0.1:11411"
ROS_HOST="127.0.0.1"
CLEAN_SCRIPT="$DEMO_ROOT/clean_stage2_runtime_residue.sh"

MASTER_PID_FILE="$PID_DIR/rosmaster.pid"
LAUNCH_PID_FILE="$PID_DIR/stage2_real_ego_visual_demo.pid"
AUTOPLAY_LAUNCH_PID_FILE="$PID_DIR/stage2_real_ego_visual_demo_autoplay.pid"
REPLAN_SHOWCASE_LAUNCH_PID_FILE="$PID_DIR/stage2_real_ego_visual_demo_replan_showcase.pid"
MASTER_CMD_PATTERN="rosmaster --core -p 11411"
LAUNCH_CMD_PATTERN="roslaunch -p 11411 human_follow_bringup stage2_real_ego_visual_demo.launch"
AUTOPLAY_LAUNCH_CMD_PATTERN="roslaunch -p 11411 human_follow_bringup stage2_real_ego_visual_demo_autoplay.launch"
REPLAN_SHOWCASE_LAUNCH_CMD_PATTERN="roslaunch -p 11411 human_follow_bringup stage2_real_ego_visual_demo_replan_showcase.launch"

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

for file in "$LAUNCH_PID_FILE" "$AUTOPLAY_LAUNCH_PID_FILE" "$REPLAN_SHOWCASE_LAUNCH_PID_FILE" "$MASTER_PID_FILE"; do
  if [[ -f "$file" ]]; then
    pid="$(cat "$file")"
    if kill -0 "$pid" 2>/dev/null; then
      kill_pid_hard "$pid"
    fi
    rm -f "$file"
  fi
done

kill_rosnodes_on_master
for pattern in "$REPLAN_SHOWCASE_LAUNCH_CMD_PATTERN" "$AUTOPLAY_LAUNCH_CMD_PATTERN" "$LAUNCH_CMD_PATTERN" "$MASTER_CMD_PATTERN"; do
  kill_pattern_until_gone "$pattern"
done

kill_pattern_until_gone "/opt/ros/noetic/lib/rosout/rosout .*${ROS_HOME_DIR}/log/"
kill_pattern_until_gone "stage2_real_ego_visual_demo_rviz"

rm -f "$PID_DIR"/*.pid 2>/dev/null || true
"$CLEAN_SCRIPT" >/dev/null 2>&1 || true

echo "stage2 real ego visual demo stopped"
