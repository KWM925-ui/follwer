#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/home/coco/follwer_ws}"
EXTERNAL_CONTROLLER_TYPE="${1:-control_algorithm_template_node.py}"
EXTERNAL_CONTROLLER_PKG="${2:-human_follow_control}"
if [[ $# -ge 2 ]]; then
  shift 2
else
  shift || true
fi

REQUIRE_PLANNING_PATH="${REQUIRE_PLANNING_PATH:-false}"
if [[ "${EXTERNAL_CONTROLLER_TYPE}" == *"planning"* ]]; then
  REQUIRE_PLANNING_PATH="true"
fi

source "${WORKSPACE_ROOT}/env/source_local_mavros_overlay.bash"
source "${WORKSPACE_ROOT}/devel/setup.bash"

LOG_DIR="${WORKSPACE_ROOT}/.codex/artifacts/stage1_external_ingress_smoke"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/$(date +%Y%m%d_%H%M%S)_${EXTERNAL_CONTROLLER_PKG}_${EXTERNAL_CONTROLLER_TYPE}.log"

pick_free_ros_master_port() {
  local port
  for port in $(seq 11331 11380); do
    if ! ss -ltn "( sport = :${port} )" | tail -n +2 | grep -q .; then
      echo "${port}"
      return 0
    fi
  done
  return 1
}

cleanup() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

ROS_MASTER_PORT="$(pick_free_ros_master_port)"
if [[ -z "${ROS_MASTER_PORT}" ]]; then
  echo "failed to pick free ROS master port" >&2
  exit 2
fi
export ROS_MASTER_URI="http://127.0.0.1:${ROS_MASTER_PORT}"

roslaunch human_follow_bringup stage1_external_ingress_regression.launch \
  external_controller_pkg:="${EXTERNAL_CONTROLLER_PKG}" \
  external_controller_type:="${EXTERNAL_CONTROLLER_TYPE}" \
  require_planning_path:="${REQUIRE_PLANNING_PATH}" \
  monitor_required:=false \
  "$@" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!
trap 'cleanup "${LAUNCH_PID}"' EXIT

DEADLINE=$((SECONDS + 90))
while (( SECONDS < DEADLINE )); do
  if [[ -f "${LOG_FILE}" ]] && grep -q "external ingress PASS" "${LOG_FILE}"; then
    cleanup "${LAUNCH_PID}"
    trap - EXIT
    grep "external ingress PASS" "${LOG_FILE}" | tail -n 1
    exit 0
  fi
  if [[ -f "${LOG_FILE}" ]] && grep -q "external ingress FAIL" "${LOG_FILE}"; then
    cleanup "${LAUNCH_PID}"
    trap - EXIT
    grep "external ingress FAIL" "${LOG_FILE}" | tail -n 1 >&2
    exit 1
  fi
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    wait "${LAUNCH_PID}" || true
    trap - EXIT
    tail -n 80 "${LOG_FILE}" >&2
    exit 1
  fi
  sleep 1
done

cleanup "${LAUNCH_PID}"
trap - EXIT
echo "external ingress smoke timeout: ${EXTERNAL_CONTROLLER_PKG}/${EXTERNAL_CONTROLLER_TYPE}" >&2
tail -n 80 "${LOG_FILE}" >&2
exit 124
