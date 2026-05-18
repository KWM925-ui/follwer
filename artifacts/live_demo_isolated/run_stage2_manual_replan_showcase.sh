#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/coco/follwer_ws"
RUN_ROOT="$ROOT/artifacts/live_demo_isolated"
PID_DIR="$RUN_ROOT/pids"
LOG_DIR="$RUN_ROOT/logs"
REC_DIR="$RUN_ROOT/recordings"
ROS_HOME_DIR="$RUN_ROOT/ros_home_verify6"
MASTER_PORT="11412"
MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
ROS_HOST="127.0.0.1"
STOP_SCRIPT="$RUN_ROOT/stop_stage2_manual_replan_showcase.sh"
CLEAN_SCRIPT="$RUN_ROOT/clean_stage2_runtime_residue.sh"
CLEAN_START="${HF_DEMO_CLEAN_START:-1}"
ENABLE_RECORD="${HF_SHOWCASE_RECORD:-0}"
EXTRA_LAUNCH_ARGS=("$@")
FILTERED_LAUNCH_ARGS=()
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="$LOG_DIR/stage2_manual_replan_showcase_${TIMESTAMP}.log"
LAUNCH_FILE="$ROOT/src/human_follow_bringup/launch/stage2_real_ego_visual_demo_manual_replan_showcase.launch"
LAUNCH_PATTERN="roslaunch ${LAUNCH_FILE}"
RVIZ_PATTERN="rviz .*stage2_real_ego_integrated.rviz"
BAG_PREFIX="$REC_DIR/stage2_acceptance_${TIMESTAMP}"
BAG_LOG="$LOG_DIR/stage2_acceptance_bag_${TIMESTAMP}.log"
TOPIC_CHECK_REGEX='^/follow/stage2/state$|^/ego_planner_node/optimal_list$|^/grid_map/cloud$'

mkdir -p "$PID_DIR" "$LOG_DIR" "$REC_DIR" "$ROS_HOME_DIR"

if [[ "$CLEAN_START" == "1" ]]; then
  "$STOP_SCRIPT" >/dev/null 2>&1 || true
  sleep 1
  "$CLEAN_SCRIPT" >/dev/null 2>&1 || true
fi

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
  local pattern="$1"
  local timeout_loops="${2:-80}"
  local pid_value=""
  local loop_index=0
  for loop_index in $(seq 1 "$timeout_loops"); do
    pid_value="$(find_running_pid "$pattern")"
    if [[ -n "$pid_value" ]] && kill -0 "$pid_value" 2>/dev/null; then
      echo "$pid_value"
      return 0
    fi
    sleep 0.25
  done
  return 1
}

launch_arg_value() {
  local key="$1"
  local arg
  for arg in "${EXTRA_LAUNCH_ARGS[@]}"; do
    if [[ "$arg" == "${key}:="* ]]; then
      echo "${arg#${key}:=}"
      return 0
    fi
  done
  return 1
}

RVIZ_ENABLED="true"
if launch_arg_value "rviz" >/dev/null; then
  RVIZ_ENABLED="$(launch_arg_value "rviz")"
fi

for arg in "${EXTRA_LAUNCH_ARGS[@]}"; do
  if [[ "$arg" == rviz:=* ]]; then
    continue
  fi
  FILTERED_LAUNCH_ARGS+=("$arg")
done

setsid env \
  ROS_MASTER_URI="$MASTER_URI" \
  ROS_IP="$ROS_HOST" \
  ROS_HOSTNAME="$ROS_HOST" \
  ROS_HOME="$ROS_HOME_DIR" \
  bash -lc "source /opt/ros/noetic/setup.bash && source '$ROOT/devel/setup.bash' && exec roslaunch '$LAUNCH_FILE' rviz:=$RVIZ_ENABLED open_image_view:=false enable_image_debug:=false ${FILTERED_LAUNCH_ARGS[*]-}" \
  >"$LAUNCH_LOG" 2>&1 < /dev/null &

LAUNCH_PID="$(wait_for_pid "$LAUNCH_PATTERN" 80)"
echo "$LAUNCH_PID" > "$PID_DIR/stage2_manual_replan_showcase.pid"

env \
  ROS_MASTER_URI="$MASTER_URI" \
  ROS_IP="$ROS_HOST" \
  ROS_HOSTNAME="$ROS_HOST" \
  ROS_HOME="$ROS_HOME_DIR" \
  bash -lc "source /opt/ros/noetic/setup.bash && source '$ROOT/devel/setup.bash' && for i in \$(seq 1 60); do rostopic list >/dev/null 2>&1 || { sleep 0.5; continue; }; if rostopic list | rg -q \"$TOPIC_CHECK_REGEX\"; then exit 0; fi; sleep 0.5; done; exit 1"

if [[ "$RVIZ_ENABLED" == "true" ]]; then
  RVIZ_PID="$(wait_for_pid "$RVIZ_PATTERN" 80)"
  echo "$RVIZ_PID" > "$PID_DIR/stage2_manual_replan_showcase_rviz.pid"
else
  RVIZ_PID=""
fi

if [[ "$ENABLE_RECORD" == "1" ]]; then
  setsid env \
    ROS_MASTER_URI="$MASTER_URI" \
    ROS_IP="$ROS_HOST" \
    ROS_HOSTNAME="$ROS_HOST" \
    ROS_HOME="$ROS_HOME_DIR" \
    bash -lc "source /opt/ros/noetic/setup.bash && source '$ROOT/devel/setup.bash' && exec rosbag record -O '$BAG_PREFIX' \
      /follow/stage2/state \
      /follow/stage2/debug_goal_path \
      /ego_planner_node/goal_point \
      /ego_planner_node/optimal_list \
      /planning/bspline \
      /grid_map/cloud \
      /grid_map/occupancy_inflate \
      /follow/fusion/target_world \
      /follow/sim/vehicle_odom \
      /follow/stage2/ego_position_cmd \
      /follow/stage2/ego_position_cmd_face_target \
      /move_base_simple/goal" \
    >"$BAG_LOG" 2>&1 < /dev/null &
  BAG_PID="$(wait_for_pid "rosbag record -O ${BAG_PREFIX}" 80)"
  echo "$BAG_PID" > "$PID_DIR/stage2_manual_replan_showcase_bag.pid"
fi

sleep 2

kill -0 "$LAUNCH_PID" 2>/dev/null
if [[ "$RVIZ_ENABLED" == "true" ]]; then
  kill -0 "$RVIZ_PID" 2>/dev/null
fi
if [[ "$ENABLE_RECORD" == "1" ]]; then
  kill -0 "$BAG_PID" 2>/dev/null
fi

echo "READY"
echo "ROS_MASTER_URI=$MASTER_URI"
echo "ROS_IP=$ROS_HOST"
echo "ROS_HOSTNAME=$ROS_HOST"
echo "ROS_HOME=$ROS_HOME_DIR"
echo "launch_pid=$LAUNCH_PID"
if [[ "$RVIZ_ENABLED" == "true" ]]; then
  echo "rviz_pid=$RVIZ_PID"
fi
echo "launch_log=$LAUNCH_LOG"
if [[ "$ENABLE_RECORD" == "1" ]]; then
  echo "bag_pid=$BAG_PID"
  echo "bag_file=${BAG_PREFIX}.bag"
  echo "bag_log=$BAG_LOG"
fi
