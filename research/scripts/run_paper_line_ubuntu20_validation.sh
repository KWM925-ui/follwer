#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

RUN_COUNT="${RUN_COUNT:-3}"
SEARCH_DURATION_SEC="${SEARCH_DURATION_SEC:-9.5}"
EXPERIMENT_SEEDS="${EXPERIMENT_SEEDS:-1:5}"
SKIP_CATKIN="${SKIP_CATKIN:-0}"
SKIP_ROS="${SKIP_ROS:-0}"

log_step() {
  printf '\n[paper-line validation] %s\n' "$1"
}

log_step "static Python checks"
python3 -m py_compile \
  src/human_follow_user/scripts/user_stage2_goal_node.py \
  src/human_follow_bringup/scripts/stage2_paper_line_regression_monitor_node.py \
  src/human_follow_bringup/scripts/stage2_paper_line_search_regression_monitor_node.py \
  research/scripts/smoke_ros1_stage2_adapter_core.py \
  research/scripts/smoke_ros1_stage2_adapter_scenarios.py \
  research/scripts/run_stage2_adapter_experiments.py \
  research/scripts/analyze_stage2_rosbag_metrics.py

log_step "offline decision-core smoke"
python3 research/scripts/smoke_ros1_stage2_adapter_core.py
python3 research/scripts/smoke_ros1_stage2_adapter_scenarios.py

log_step "offline baseline/ablation diagnostics"
python3 research/scripts/run_stage2_adapter_experiments.py --seeds "${EXPERIMENT_SEEDS}"

if [[ "${SKIP_CATKIN}" != "1" ]]; then
  log_step "catkin build"
  catkin_make
fi

if [[ -f devel/setup.bash ]]; then
  # shellcheck disable=SC1091
  source devel/setup.bash
fi

if [[ "${SKIP_ROS}" == "1" ]]; then
  log_step "ROS regression skipped"
  exit 0
fi

if ! command -v roslaunch >/dev/null 2>&1; then
  echo "roslaunch is not available; source ROS1/catkin setup or set SKIP_ROS=1" >&2
  exit 2
fi

log_step "normal paper-line ROS1/EGO regression"
bash research/scripts/run_paper_line_ros1_regression.sh "${RUN_COUNT}"

log_step "target-loss/search paper-line ROS1/EGO regression"
MONITOR_DURATION_SEC="${SEARCH_DURATION_SEC}" LAUNCH_TIMEOUT_SEC="${LAUNCH_TIMEOUT_SEC:-25}" \
bash research/scripts/run_paper_line_ros1_regression.sh 1 \
  fixture_scenario_yaml:="$(pwd)/src/human_follow_bringup/config/paper_line_stage2_target_loss.yaml" \
  monitor_full_duration_evidence:=true \
  monitor_required_state_names:=follow,predict_hold,search_safe_viewpoint \
  monitor_required_phase_labels:=loss_target_visible_start,short_dropout_predict_hold,reacquired_after_short_loss,long_dropout_search,reacquired_after_search

log_step "PASS"
