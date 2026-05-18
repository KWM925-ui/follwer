#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/coco/follwer_ws}"
RUN_TAG="${RUN_TAG:-$(date +%Y%m%d_%H%M%S)}"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${WORKSPACE_ROOT}/.codex/artifacts/nohardware_acceptance/${RUN_TAG}}"
STEP_LOG_DIR="${ARTIFACT_ROOT}/step_logs"

mkdir -p "${ARTIFACT_ROOT}/ros_home" "${ARTIFACT_ROOT}/ros_logs" "${STEP_LOG_DIR}"
export ROS_HOME="${ARTIFACT_ROOT}/ros_home"
export ROS_LOG_DIR="${ARTIFACT_ROOT}/ros_logs"
export ROS_IP="127.0.0.1"
export ROS_HOSTNAME="127.0.0.1"

source "${WORKSPACE_ROOT}/devel/setup.bash" 2>/dev/null || true
source "${WORKSPACE_ROOT}/env/source_local_mavros_overlay.bash"

echo "[INFO] nohardware acceptance artifact root: ${ARTIFACT_ROOT}"

echo "[STEP] prepare public people smoke video"
"${WORKSPACE_ROOT}/env/prepare_public_people_smoke_video.bash"

echo "[STEP] python compile"
python3 -m compileall -q "${WORKSPACE_ROOT}/src"

echo "[STEP] catkin_make"
catkin_log="${STEP_LOG_DIR}/catkin_make.log"
if ! (
  cd "${WORKSPACE_ROOT}"
  catkin_make -DCATKIN_ENABLE_TESTING=OFF
) >"${catkin_log}" 2>&1; then
  tail -n 120 "${catkin_log}" >&2 || true
  exit 1
fi
echo "  [PASS] catkin_make -> ${catkin_log}"

source "${WORKSPACE_ROOT}/devel/setup.bash"
source "${WORKSPACE_ROOT}/env/source_local_mavros_overlay.bash"

echo "[STEP] parse stage1 launch files"
while IFS= read -r launch_file; do
  launch_name="$(basename "${launch_file}")"
  case "${launch_name}" in
    *_hw.launch|*sitl*.launch)
      echo "  [SKIP] ${launch_name} (hardware or simulation only)"
      continue
      ;;
  esac
  echo "  [PARSE] ${launch_name}"
  roslaunch --nodes human_follow_bringup "${launch_name}" >/dev/null
done < <(find "${WORKSPACE_ROOT}/src/human_follow_bringup/launch" -maxdepth 1 -type f -name 'stage1*.launch' | sort)

echo "[STEP] stage1 px4 software preflight"
"${WORKSPACE_ROOT}/env/stage1_px4_preflight_check.bash"

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

run_launch() {
  local timeout_sec="$1"
  shift
  local package_name="$1"
  local launch_name="$2"
  shift 2
  local step_name="${launch_name%.launch}"
  local log_path="${STEP_LOG_DIR}/${step_name}.log"
  local timeout_value="${timeout_sec%s}"
  local launch_pid=""
  local status=0
  local pass_line=""
  local fail_line=""
  local deadline=$((SECONDS + timeout_value))
  local ros_port=""
  local ros_step_root="${ARTIFACT_ROOT}/roslaunch_runs/${step_name}"
  local step_ros_home="${ros_step_root}/ros_home"
  local step_ros_logs="${ros_step_root}/ros_logs"
  mkdir -p "${step_ros_home}" "${step_ros_logs}"
  ros_port="$(python3 - <<'PY'
import socket
s = socket.socket()
s.bind(("127.0.0.1", 0))
print(s.getsockname()[1])
s.close()
PY
)"
  echo "  [RUN] ${package_name} ${launch_name}"
  ROS_MASTER_URI="http://127.0.0.1:${ros_port}" \
  ROS_HOME="${step_ros_home}" \
  ROS_LOG_DIR="${step_ros_logs}" \
  roslaunch "${package_name}" "${launch_name}" monitor_required:=false "$@" >"${log_path}" 2>&1 &
  launch_pid="$!"

  while true; do
    fail_line="$(grep -m1 " FAIL " "${log_path}" || true)"
    if [[ -n "${fail_line}" ]]; then
      shutdown_launch "${launch_pid}"
      status="$(wait_launch_status "${launch_pid}")"
      tail -n 120 "${log_path}" >&2 || true
      return 1
    fi

    pass_line="$(grep -m1 " PASS " "${log_path}" || true)"
    if [[ -n "${pass_line}" ]]; then
      shutdown_launch "${launch_pid}"
      status="$(wait_launch_status "${launch_pid}")"
      echo "    ${pass_line}"
      return 0
    fi

    if ! kill -0 "${launch_pid}" 2>/dev/null; then
      status="$(wait_launch_status "${launch_pid}")"
      fail_line="$(grep -m1 " FAIL " "${log_path}" || true)"
      pass_line="$(grep -m1 " PASS " "${log_path}" || true)"
      if [[ -n "${pass_line}" && "${status}" -eq 0 ]]; then
        echo "    ${pass_line}"
        return 0
      fi
      tail -n 120 "${log_path}" >&2 || true
      if [[ -n "${fail_line}" ]]; then
        return 1
      fi
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

echo "[STEP] synthetic and regression launches"
run_launch 20s human_follow_bringup stage1_projection_sanity.launch
run_launch 20s human_follow_bringup stage1_association_sanity.launch
run_launch 20s human_follow_bringup stage1_target_fusion_synthetic.launch
run_launch 20s human_follow_bringup stage1_target_fusion_body_synthetic.launch
run_launch 30s human_follow_bringup stage1_tracker_sequence_regression.launch
run_launch 20s human_follow_bringup stage1_px4_bridge_regression.launch
run_launch 20s human_follow_bringup stage1_offboard_gate_regression.launch
run_launch 30s human_follow_bringup stage1_controller_px4_chain_regression.launch
run_launch 30s human_follow_bringup stage1_offline_video_px4_smoke.launch video_path:="${WORKSPACE_ROOT}/data/samples/public_people_smoke.avi"
run_launch 20s human_follow_bringup stage1_live_px4_idle_regression.launch

echo "[STEP] stage1 truth demo smoke launches"
RUN_SECONDS=4 VARIANT_TAG=builtin WORKSPACE_ROOT="${WORKSPACE_ROOT}" "${WORKSPACE_ROOT}/env/stage1_truth_demo_smoke.bash" stage1_truth_fusion_controller_regression.launch loop:=true enable_monitor:=false

echo "[STEP] stage1 truth case suite"
WORKSPACE_ROOT="${WORKSPACE_ROOT}" "${WORKSPACE_ROOT}/env/run_stage1_truth_case_suite.bash"

if [[ -x "${WORKSPACE_ROOT}/env/sanitize_ros_master_logs.bash" ]]; then
  "${WORKSPACE_ROOT}/env/sanitize_ros_master_logs.bash" "${ROS_LOG_DIR}"
fi

echo "[PASS] stage1 no-hardware acceptance completed"
