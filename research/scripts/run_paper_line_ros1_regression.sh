#!/usr/bin/env bash
set -euo pipefail

RUN_COUNT="${1:-${RUN_COUNT:-3}}"
if ! [[ "${RUN_COUNT}" =~ ^[0-9]+$ ]] || [[ "${RUN_COUNT}" -lt 1 ]]; then
  echo "RUN_COUNT must be a positive integer" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

if [[ ! -f devel/setup.bash ]]; then
  echo "devel/setup.bash not found; run catkin_make first" >&2
  exit 2
fi

set +u
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
MAVROS_OVERLAY_PREFIX="${MAVROS_OVERLAY_PREFIX:-/home/coco/.local/ros_noetic_overlay/opt/ros/noetic}"
if [[ -f "${MAVROS_OVERLAY_PREFIX}/setup.bash" ]]; then
  source "${MAVROS_OVERLAY_PREFIX}/setup.bash"
  export MAVROS_OVERLAY_PREFIX
fi
source devel/setup.bash
set -u

DATE_TAG="$(date +%Y%m%d)"
RUN_TAG="$(date +%H%M%S)"
ARTIFACT_ROOT="${ARTIFACT_ROOT:-${REPO_ROOT}/.codex/artifacts/paper_line_ros1_adapter_${DATE_TAG}/formal_real_ego_regression_${RUN_TAG}}"
LAUNCH_TIMEOUT_SEC="${LAUNCH_TIMEOUT_SEC:-35}"
MONITOR_DURATION_SEC="${MONITOR_DURATION_SEC:-18.0}"
SAMPLE_TIMEOUT_SEC="${SAMPLE_TIMEOUT_SEC:-16}"
ROS_PORT_BASE="${ROS_PORT_BASE:-12400}"

mkdir -p "${ARTIFACT_ROOT}"

git rev-parse --abbrev-ref HEAD > "${ARTIFACT_ROOT}/git_branch.txt"
git rev-parse HEAD > "${ARTIFACT_ROOT}/git_commit.txt"
git status --short --branch > "${ARTIFACT_ROOT}/git_status.txt"

terminate_group() {
  local pgid="$1"
  if ps -o pid= -g "${pgid}" >/dev/null 2>&1; then
    kill -TERM "-${pgid}" >/dev/null 2>&1 || true
    sleep 2
  fi
  if ps -o pid= -g "${pgid}" >/dev/null 2>&1; then
    kill -KILL "-${pgid}" >/dev/null 2>&1 || true
  fi
}

sample_topic() {
  local topic="$1"
  local out_file="$2"
  local count="${3:-1}"
  (
    timeout --kill-after=2s "${SAMPLE_TIMEOUT_SEC}" bash -c '
      topic="$1"
      count="$2"
      until rostopic list >/dev/null 2>&1; do
        sleep 0.2
      done
      rostopic echo -n "${count}" "${topic}"
    ' _ "${topic}" "${count}"
  ) > "${out_file}" 2>&1 &
  SAMPLER_PIDS+=("$!")
}

sample_ros_list() {
  local out_file="$1"
  local command_name="$2"
  (
    timeout --kill-after=2s "${SAMPLE_TIMEOUT_SEC}" bash -c '
      command_name="$1"
      until rostopic list >/dev/null 2>&1; do
        sleep 0.2
      done
      "${command_name}" list
    ' _ "${command_name}"
  ) > "${out_file}" 2>&1 &
  SAMPLER_PIDS+=("$!")
}

for run_idx in $(seq 1 "${RUN_COUNT}"); do
  RUN_DIR="${ARTIFACT_ROOT}/run_${run_idx}"
  mkdir -p "${RUN_DIR}/ros_home" "${RUN_DIR}/ros_log"

  export ROS_MASTER_URI="http://127.0.0.1:$((ROS_PORT_BASE + run_idx))"
  export ROS_IP="127.0.0.1"
  unset ROS_HOSTNAME || true
  export ROS_HOME="${RUN_DIR}/ros_home"
  export ROS_LOG_DIR="${RUN_DIR}/ros_log"

  printf 'ROS_MASTER_URI=%s\n' "${ROS_MASTER_URI}" > "${RUN_DIR}/environment.txt"
  printf 'roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch max_duration_sec:=%s\n' "${MONITOR_DURATION_SEC}" > "${RUN_DIR}/command.txt"

  setsid roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch \
    max_duration_sec:="${MONITOR_DURATION_SEC}" \
    > "${RUN_DIR}/roslaunch.log" 2>&1 &
  LAUNCH_PID="$!"
  echo "${LAUNCH_PID}" > "${RUN_DIR}/roslaunch.pid"

  SAMPLER_PIDS=()
  sleep 1
  sample_ros_list "${RUN_DIR}/topics.txt" rostopic
  sample_ros_list "${RUN_DIR}/nodes.txt" rosnode
  sample_topic "/follow/stage2/state" "${RUN_DIR}/stage2_state.txt" 2
  sample_topic "/follow/stage2/goal" "${RUN_DIR}/stage2_goal.txt" 1
  sample_topic "/follow/stage2/debug_goal_path" "${RUN_DIR}/debug_goal_path.txt" 1
  sample_topic "/move_base_simple/goal" "${RUN_DIR}/ego_goal.txt" 1
  sample_topic "/waypoint_generator/waypoints" "${RUN_DIR}/waypoints.txt" 1
  sample_topic "/planning/bspline" "${RUN_DIR}/bspline.txt" 1
  sample_topic "/follow/stage2/ego_position_cmd" "${RUN_DIR}/ego_position_cmd.txt" 1
  sample_topic "/follow/stage2/offboard/setpoint" "${RUN_DIR}/offboard_setpoint.txt" 1
  sample_topic "/mavros/setpoint_raw/local" "${RUN_DIR}/mavros_setpoint.txt" 1

  START_SEC="$(date +%s)"
  TIMED_OUT=0
  while kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; do
    NOW_SEC="$(date +%s)"
    if (( NOW_SEC - START_SEC >= LAUNCH_TIMEOUT_SEC )); then
      TIMED_OUT=1
      echo "timeout" > "${RUN_DIR}/result.txt"
      terminate_group "${LAUNCH_PID}"
      break
    fi
    sleep 1
  done

  set +e
  wait "${LAUNCH_PID}"
  LAUNCH_RC="$?"
  set -e

  for sampler_pid in "${SAMPLER_PIDS[@]}"; do
    wait "${sampler_pid}" >/dev/null 2>&1 || true
  done

  terminate_group "${LAUNCH_PID}"
  LEFTOVERS="$(ps -o pid= -g "${LAUNCH_PID}" 2>/dev/null | tr '\n' ' ' | xargs || true)"
  if [[ -z "${LEFTOVERS}" ]]; then
    PROCESS_CLEAN="true"
  else
    PROCESS_CLEAN="false"
    printf '%s\n' "${LEFTOVERS}" > "${RUN_DIR}/leftover_pids.txt"
  fi

  python3 - "${RUN_DIR}" "${LAUNCH_RC}" "${TIMED_OUT}" "${PROCESS_CLEAN}" <<'PY'
import json
import re
import sys
from pathlib import Path

run_dir = Path(sys.argv[1])
launch_rc = int(sys.argv[2])
timed_out = bool(int(sys.argv[3]))
process_clean = sys.argv[4].lower() == "true"
log_text = (run_dir / "roslaunch.log").read_text(errors="replace") if (run_dir / "roslaunch.log").exists() else ""

pass_match = re.search(
    r"stage2 paper-line real ego regression PASS .*?distinct_goals=(\d+) distinct_cmds=(\d+) request_count=(\d+)",
    log_text,
    re.S,
)
fail_match = re.search(r"stage2 paper-line real ego regression FAIL ([^\n\r]*)", log_text)
final_plan = re.findall(r"final_plan_success=([01])", log_text)

def read(name):
    path = run_dir / name
    return path.read_text(errors="replace") if path.exists() else ""

topics_text = read("topics.txt")
setpoint_contract = bool(pass_match) and "offboard_mode_gate state=offboard_active" in log_text
sample_checks = {
    "/planning/bspline": "pos_pts:" in read("bspline.txt"),
    "/follow/stage2/ego_position_cmd": "position:" in read("ego_position_cmd.txt"),
    "/follow/stage2/goal": "pose:" in read("stage2_goal.txt"),
    "/follow/stage2/debug_goal_path": "poses:" in read("debug_goal_path.txt"),
    "/move_base_simple/goal": "pose:" in read("ego_goal.txt"),
    "/waypoint_generator/waypoints": "poses:" in read("waypoints.txt"),
    "/follow/stage2/offboard/setpoint": "position:" in read("offboard_setpoint.txt")
    or ("/follow/stage2/offboard/setpoint" in topics_text and setpoint_contract),
    "/mavros/setpoint_raw/local": "position:" in read("mavros_setpoint.txt")
    or ("/mavros/setpoint_raw/local" in topics_text and setpoint_contract),
}

error_lines = []
for line in log_text.splitlines():
    lower = line.lower()
    if "traceback" in lower or "fatal" in lower or "in obstacle" in lower or "[error]" in lower or " error " in lower:
        error_lines.append(line)

summary = {
    "run_dir": str(run_dir),
    "passed": bool(pass_match) and not timed_out,
    "launch_rc": launch_rc,
    "timed_out": timed_out,
    "process_clean": process_clean,
    "distinct_goals": int(pass_match.group(1)) if pass_match else 0,
    "distinct_cmds": int(pass_match.group(2)) if pass_match else 0,
    "request_count": int(pass_match.group(3)) if pass_match else 0,
    "failure_reason": fail_match.group(1).strip() if fail_match else "",
    "sampled_topics": sample_checks,
    "setpoint_contract_confirmed_by_monitor": setpoint_contract,
    "final_plan_success": {
        "0": final_plan.count("0"),
        "1": final_plan.count("1"),
    },
    "error_keyword_count": len(error_lines),
    "error_keyword_lines": error_lines[:40],
}
(run_dir / "run_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
print(json.dumps(summary, sort_keys=True))
PY
done

python3 - "${ARTIFACT_ROOT}" "${RUN_COUNT}" <<'PY'
import json
import sys
from pathlib import Path

artifact_root = Path(sys.argv[1])
run_count = int(sys.argv[2])
runs = []
for idx in range(1, run_count + 1):
    summary_path = artifact_root / ("run_%d" % idx) / "run_summary.json"
    runs.append(json.loads(summary_path.read_text()))

branch = (artifact_root / "git_branch.txt").read_text().strip()
commit = (artifact_root / "git_commit.txt").read_text().strip()
status = (artifact_root / "git_status.txt").read_text().splitlines()
passed = [run for run in runs if run["passed"]]
summary = {
    "artifact_root": str(artifact_root),
    "git_branch": branch,
    "git_commit": commit,
    "git_status": status,
    "launch": "human_follow_bringup stage2_paper_line_real_ego_regression.launch",
    "run_count": run_count,
    "pass_count": len(passed),
    "all_passed": len(passed) == run_count,
    "runs": runs,
    "claim_boundary": "ROS1 repeatable regression evidence only; not hardware readiness or robustness completion.",
}
(artifact_root / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

lines = [
    "# Paper-Line ROS1 Regression Summary",
    "",
    "- branch: `%s`" % branch,
    "- commit: `%s`" % commit[:12],
    "- launch: `stage2_paper_line_real_ego_regression.launch`",
    "- pass_count: `%d/%d`" % (len(passed), run_count),
    "",
    "| run | pass | distinct_goals | distinct_cmds | request_count | bspline | ego_cmd | final_plan_success=0 | final_plan_success=1 | errors | clean |",
    "|---:|:---:|---:|---:|---:|:---:|:---:|---:|---:|---:|:---:|",
]
for idx, run in enumerate(runs, 1):
    samples = run["sampled_topics"]
    lines.append(
        "| %d | %s | %d | %d | %d | %s | %s | %d | %d | %d | %s |"
        % (
            idx,
            "PASS" if run["passed"] else "FAIL",
            run["distinct_goals"],
            run["distinct_cmds"],
            run["request_count"],
            "yes" if samples.get("/planning/bspline") else "no",
            "yes" if samples.get("/follow/stage2/ego_position_cmd") else "no",
            run["final_plan_success"]["0"],
            run["final_plan_success"]["1"],
            run["error_keyword_count"],
            "yes" if run["process_clean"] else "no",
        )
    )
lines.extend([
    "",
    "Boundary: this is repeatable ROS1 regression evidence only; it is not hardware readiness or a robustness-complete claim.",
])
(artifact_root / "summary.md").write_text("\n".join(lines) + "\n")

print(json.dumps({"artifact_root": str(artifact_root), "pass_count": len(passed), "run_count": run_count, "all_passed": len(passed) == run_count}, sort_keys=True))
sys.exit(0 if len(passed) == run_count else 1)
PY
