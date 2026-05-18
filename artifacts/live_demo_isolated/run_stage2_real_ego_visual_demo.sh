#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/coco/follwer_ws"
DEMO_ROOT="$ROOT/artifacts/live_demo_isolated"
PID_DIR="$DEMO_ROOT/pids"
ROS_HOME_DIR="$DEMO_ROOT/ros_home"
LOG_DIR="$DEMO_ROOT/logs"
MASTER_PORT="11411"
MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
ROS_HOST="127.0.0.1"
MASTER_CMD_PATTERN="rosmaster --core -p ${MASTER_PORT}"
LAUNCH_CMD_PATTERN="roslaunch -p ${MASTER_PORT} human_follow_bringup stage2_real_ego_visual_demo.launch"
STOP_SCRIPT="$DEMO_ROOT/stop_stage2_real_ego_visual_demo.sh"
CLEAN_SCRIPT="$DEMO_ROOT/clean_stage2_runtime_residue.sh"
CLEAN_START="${HF_DEMO_CLEAN_START:-1}"
EXTRA_LAUNCH_ARGS=("$@")
TOPIC_CHECK_REGEX='^/follow/stage2/state$|^/ego_planner_node/optimal_list$|^/grid_map/cloud$|^/follow/sim/truth_phase$'

mkdir -p "$PID_DIR" "$ROS_HOME_DIR" "$LOG_DIR"

if [[ "$CLEAN_START" == "1" ]]; then
  "$STOP_SCRIPT" >/dev/null 2>&1 || true
  sleep 1
  "$CLEAN_SCRIPT" >/dev/null 2>&1 || true
fi

LAUNCH_PID_FILE="$PID_DIR/stage2_real_ego_visual_demo.pid"
LAUNCH_LOG="$LOG_DIR/stage2_real_ego_visual_demo.log"

find_running_pid() {
  local pattern="$1"
  ps -eo pid=,comm=,args= \
    | awk -v pat="$pattern" '
        $2 ~ /^(bash|timeout|awk|rg|pgrep|ps|sed)$/ { next }
        $0 ~ pat { print $1; exit }
      ' \
    || true
}

wait_for_pid() {
  local pid_file="$1"
  local pattern="$2"
  local pid_value=""
  local attempt=0
  for attempt in $(seq 1 40); do
    pid_value="$(find_running_pid "$pattern")"
    if [[ -n "$pid_value" ]] && kill -0 "$pid_value" 2>/dev/null; then
      echo "$pid_value" > "$pid_file"
      return 0
    fi
    sleep 0.25
  done
  return 1
}

if launch_pid="$(find_running_pid "$LAUNCH_CMD_PATTERN")"; [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
  echo "$launch_pid" > "$LAUNCH_PID_FILE"
  echo "demo launch already running pid=$launch_pid"
else
  setsid env \
    ROS_MASTER_URI="$MASTER_URI" \
    ROS_IP="$ROS_HOST" \
    ROS_HOSTNAME="$ROS_HOST" \
    ROS_HOME="$ROS_HOME_DIR" \
    bash -lc "source /opt/ros/noetic/setup.bash && source '$ROOT/devel/setup.bash' && exec roslaunch -p ${MASTER_PORT} human_follow_bringup stage2_real_ego_visual_demo.launch ${EXTRA_LAUNCH_ARGS[*]-}" \
    >"$LAUNCH_LOG" 2>&1 < /dev/null &
  wait_for_pid "$LAUNCH_PID_FILE" "$LAUNCH_CMD_PATTERN"
fi

MASTER_PID_FILE="$PID_DIR/rosmaster.pid"
wait_for_pid "$MASTER_PID_FILE" "$MASTER_CMD_PATTERN"

env \
  ROS_MASTER_URI="$MASTER_URI" \
  ROS_IP="$ROS_HOST" \
  ROS_HOSTNAME="$ROS_HOST" \
  ROS_HOME="$ROS_HOME_DIR" \
  bash -lc "source /opt/ros/noetic/setup.bash && source '$ROOT/devel/setup.bash' && for i in \$(seq 1 80); do rostopic list >/dev/null 2>&1 || { sleep 0.5; continue; }; if rostopic list | rg -q \"$TOPIC_CHECK_REGEX\"; then exit 0; fi; sleep 0.5; done; exit 1"

echo "ROS_MASTER_URI=$MASTER_URI"
echo "ROS_IP=$ROS_HOST"
echo "ROS_HOSTNAME=$ROS_HOST"
echo "ROS_HOME=$ROS_HOME_DIR"
echo "rosmaster_pid=$(cat "$MASTER_PID_FILE")"
echo "launch_pid=$(cat "$LAUNCH_PID_FILE")"
echo "launch_log=$LAUNCH_LOG"
