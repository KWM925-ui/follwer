#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/coco/follwer_ws}"
LAUNCH_PACKAGE="${LAUNCH_PACKAGE:-human_follow_bringup}"
LAUNCH_NAME="${1:-stage1_truth_visual_demo.launch}"
if [[ $# -gt 0 ]]; then
  shift
fi

RUN_SECONDS="${RUN_SECONDS:-6}"
STARTUP_TIMEOUT_SEC="${STARTUP_TIMEOUT_SEC:-12}"
TOPIC_ECHO_TIMEOUT_SEC="${TOPIC_ECHO_TIMEOUT_SEC:-10}"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${WORKSPACE_ROOT}/.codex/artifacts/stage1_demo_smoke}"
VARIANT_TAG="${VARIANT_TAG:-}"

if [[ -n "${EXPECT_TOPICS:-}" ]]; then
  read -r -a EXPECTED_TOPICS <<<"${EXPECT_TOPICS}"
else
  EXPECTED_TOPICS=(
    /follow/viz/markers
    /follow/viz/truth_path
    /follow/viz/fusion_path
    /follow/viz/vehicle_path
    /follow/lidar/points
    /follow/control/cmd_body
    /follow/sim/vehicle_odom
    /follow/sim/human_truth_world
  )
fi

source "${WORKSPACE_ROOT}/devel/setup.bash" 2>/dev/null || true
source "${WORKSPACE_ROOT}/env/source_local_mavros_overlay.bash"

mkdir -p "${ARTIFACT_ROOT}"
run_tag="$(date +%Y%m%d_%H%M%S)_${LAUNCH_NAME%.launch}"
if [[ -n "${VARIANT_TAG}" ]]; then
  sanitized_variant="$(printf '%s' "${VARIANT_TAG}" | tr ' /:=' '____' | tr -cd 'A-Za-z0-9._-')"
  run_tag="${run_tag}_${sanitized_variant}"
fi
run_dir="${ARTIFACT_ROOT}/${run_tag}"
mkdir -p "${run_dir}/ros_home" "${run_dir}/ros_logs"

if [[ -z "${ROS_PORT:-}" || "${ROS_PORT}" == "0" ]]; then
  ROS_PORT="$(python3 - <<'PY'
import socket
s = socket.socket()
s.bind(("127.0.0.1", 0))
print(s.getsockname()[1])
s.close()
PY
)"
fi

export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_HOME="${run_dir}/ros_home"
export ROS_LOG_DIR="${run_dir}/ros_logs"

launch_log="${run_dir}/roslaunch_stdout.log"
probe_log="${run_dir}/topic_probe.log"
node_log="${run_dir}/rosnodes.txt"

launch_pid=""

cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "[INFO] launch=${LAUNCH_NAME} run_dir=${run_dir} master=${ROS_MASTER_URI}" | tee "${probe_log}"
roslaunch "${LAUNCH_PACKAGE}" "${LAUNCH_NAME}" rviz:=false open_image_view:=false "$@" >"${launch_log}" 2>&1 &
launch_pid="$!"

startup_deadline="$(python3 - <<PY
import time
print(time.time() + float("${STARTUP_TIMEOUT_SEC}"))
PY
)"

while true; do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "[FAIL] roslaunch exited before startup completed" | tee -a "${probe_log}"
    tail -n 120 "${launch_log}" >&2 || true
    exit 1
  fi
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  now_epoch="$(python3 - <<'PY'
import time
print(time.time())
PY
)"
  if python3 - <<PY
startup_deadline = float("${startup_deadline}")
now_epoch = float("${now_epoch}")
raise SystemExit(0 if now_epoch <= startup_deadline else 1)
PY
  then
    sleep 0.25
  else
    echo "[FAIL] roscore did not come up within ${STARTUP_TIMEOUT_SEC}s" | tee -a "${probe_log}"
    tail -n 120 "${launch_log}" >&2 || true
    exit 1
  fi
done

for topic_name in "${EXPECTED_TOPICS[@]}"; do
  echo "[PROBE] ${topic_name}" | tee -a "${probe_log}"
  if ! timeout -s INT "${TOPIC_ECHO_TIMEOUT_SEC}" rostopic echo -n1 "${topic_name}" >/dev/null 2>&1; then
    echo "[FAIL] topic probe timed out: ${topic_name}" | tee -a "${probe_log}"
    tail -n 120 "${launch_log}" >&2 || true
    exit 1
  fi
done

rosnode list >"${node_log}" 2>/dev/null || true

sleep "${RUN_SECONDS}"

kill -INT "${launch_pid}" 2>/dev/null || true
set +e
wait "${launch_pid}"
launch_status="$?"
set -e
launch_pid=""

if [[ -x "${WORKSPACE_ROOT}/env/sanitize_ros_master_logs.bash" ]]; then
  "${WORKSPACE_ROOT}/env/sanitize_ros_master_logs.bash" "${run_dir}/ros_logs"
fi

if grep -Eq 'Traceback|RuntimeError|RLException|new node registered with same name|\[FATAL\]|\[ERROR\]' "${launch_log}"; then
  echo "[FAIL] error pattern detected in ${launch_log}" | tee -a "${probe_log}"
  tail -n 120 "${launch_log}" >&2 || true
  exit 1
fi

if [[ "${launch_status}" -ne 0 && "${launch_status}" -ne 130 ]]; then
  echo "[FAIL] roslaunch exit status=${launch_status}" | tee -a "${probe_log}"
  tail -n 120 "${launch_log}" >&2 || true
  exit 1
fi

echo "[PASS] demo smoke launch=${LAUNCH_NAME} run_dir=${run_dir}" | tee -a "${probe_log}"
