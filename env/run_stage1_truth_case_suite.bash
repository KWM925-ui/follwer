#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/coco/follwer_ws}"
RUN_TAG="${RUN_TAG:-$(date +%Y%m%d_%H%M%S)}"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${WORKSPACE_ROOT}/.codex/artifacts/stage1_truth_case_suite/${RUN_TAG}}"
STEP_LOG_DIR="${ARTIFACT_ROOT}/step_logs"

mkdir -p "${ARTIFACT_ROOT}/ros_home" "${ARTIFACT_ROOT}/ros_logs" "${STEP_LOG_DIR}"
export ROS_HOME="${ARTIFACT_ROOT}/ros_home"
export ROS_LOG_DIR="${ARTIFACT_ROOT}/ros_logs"
export ROS_IP="127.0.0.1"
export ROS_HOSTNAME="127.0.0.1"

source "${WORKSPACE_ROOT}/devel/setup.bash" 2>/dev/null || true
source "${WORKSPACE_ROOT}/env/source_local_mavros_overlay.bash"

echo "[INFO] stage1 truth case suite artifact root: ${ARTIFACT_ROOT}"

shutdown_launch() {
  local launch_pid="$1"
  local int_deadline=$((SECONDS + 5))

  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    while kill -0 "${launch_pid}" 2>/dev/null; do
      if (( SECONDS >= int_deadline )); then
        local term_deadline=$((SECONDS + 2))
        kill -TERM "${launch_pid}" 2>/dev/null || true
        while kill -0 "${launch_pid}" 2>/dev/null; do
          if (( SECONDS >= term_deadline )); then
            kill -KILL "${launch_pid}" 2>/dev/null || true
            break
          fi
          sleep 0.2
        done
        break
      fi
      sleep 0.2
    done
  fi
}

wait_launch_status() {
  local launch_pid="$1"
  local launch_status=0

  set +e
  wait "${launch_pid}"
  launch_status="$?"
  set -e

  echo "${launch_status}"
}

wait_launch_natural_exit() {
  local launch_pid="$1"
  local timeout_sec="${2:-5}"
  local deadline=$((SECONDS + timeout_sec))

  while kill -0 "${launch_pid}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 0.2
  done

  return 0
}

run_case() {
  local case_name="$1"
  local log_path="${STEP_LOG_DIR}/${case_name}.log"
  local launch_pid=""
  local pass_line=""
  local fail_line=""
  local deadline=$((SECONDS + 22))
  local ros_port=""

  ros_port="$(python3 - <<'PY'
import socket
s = socket.socket()
s.bind(("127.0.0.1", 0))
print(s.getsockname()[1])
s.close()
PY
)"

  echo "[CASE] ${case_name}"
  : >"${log_path}"
  ROS_MASTER_URI="http://127.0.0.1:${ros_port}" roslaunch human_follow_bringup stage1_truth_fusion_controller_regression.launch \
    loop:=true \
    rviz:=false \
    open_image_view:=false \
    enable_monitor:=false \
    motion_mode:="${case_name}" \
    enable_case_validation:=true \
    validation_case:="${case_name}" \
    monitor_required:=true \
    >"${log_path}" 2>&1 &
  launch_pid="$!"

  while true; do
    fail_line="$(grep -m1 "truth_case_validation FAIL" "${log_path}" || true)"
    if [[ -n "${fail_line}" ]]; then
      shutdown_launch "${launch_pid}"
      wait_launch_status "${launch_pid}" >/dev/null || true
      tail -n 120 "${log_path}" >&2 || true
      return 1
    fi

    pass_line="$(grep -m1 "truth_case_validation PASS" "${log_path}" || true)"
    if [[ -n "${pass_line}" ]]; then
      wait_launch_natural_exit "${launch_pid}" 5 || shutdown_launch "${launch_pid}"
      wait_launch_status "${launch_pid}" >/dev/null || true
      echo "  [PASS] ${case_name}"
      return 0
    fi

    if ! kill -0 "${launch_pid}" 2>/dev/null; then
      local status
      status="$(wait_launch_status "${launch_pid}")"
      tail -n 120 "${log_path}" >&2 || true
      return "${status}"
    fi

    if (( SECONDS >= deadline )); then
      shutdown_launch "${launch_pid}"
      wait_launch_status "${launch_pid}" >/dev/null || true
      tail -n 120 "${log_path}" >&2 || true
      return 124
    fi

    sleep 0.2
  done
}

CASES=(
  acquire_center
  search_reacquire_right
  search_reacquire_left
  person_approach_retreat
  person_depart_follow
  lateral_left_track
  lateral_right_track
)

for case_name in "${CASES[@]}"; do
  run_case "${case_name}"
done

if [[ -x "${WORKSPACE_ROOT}/env/sanitize_ros_master_logs.bash" ]]; then
  "${WORKSPACE_ROOT}/env/sanitize_ros_master_logs.bash" "${ROS_LOG_DIR}"
fi

echo "[PASS] stage1 truth case suite completed"
