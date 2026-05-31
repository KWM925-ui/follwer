# 2026-06-01 Ubuntu20 ROS1 Validation Log

Scope: paper-line only.

## Purpose

Validate the paper-line ROS1 adapter on the target Ubuntu 20.04 + ROS Noetic
software stack after the Windows/MATLAB algorithm refresh.

This is ROS1/EGO software-chain evidence. It is not real hardware flight
evidence and does not prove full-system superiority.

## Repository State

- Branch: `paper-line`
- Commit: `bc684c537198cd3ea77ea877bd9bb590e190476d`
- Remote: local branch matched `origin/paper-line` before validation.
- Runtime: Ubuntu 20.04.6 LTS, Python 3.8.10, ROS Noetic.
- `catkin_make` and `roslaunch` were available from `/opt/ros/noetic/bin`.

## Phase 0 And Phase 1

Command:

```bash
SKIP_ROS=1 bash research/scripts/run_paper_line_ubuntu20_validation.sh
```

Result: PASS.

What passed:

- Python syntax checks;
- offline decision-core smoke;
- offline scenario smoke;
- offline baseline/ablation diagnostics;
- `catkin_make`.

Offline diagnostics output:

- `research/runs/stage2_adapter_experiments/20260601_002224`
- 225 runs.
- Script summary: `stage2 adapter experiments PASS ... best=fixed_behind success=1.000`

Build notes:

- `catkin_make` completed successfully on this Ubuntu20 runtime.
- CMake/PCL/VTK emitted warnings, but there was no build failure.

## Phase 2: Single ROS1/EGO Regressions

Normal regression command:

```bash
RUN_COUNT=1 LAUNCH_TIMEOUT_SEC=35 MONITOR_DURATION_SEC=18.0 \
bash research/scripts/run_paper_line_ros1_regression.sh 1
```

Result: PASS.

Artifact:

- `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002359`

Key evidence:

- `distinct_goals=4`
- `distinct_cmds=3`
- `request_count=1`
- `final_plan_success`: `0=0`, `1=20`
- sampled topics all true:
  `/follow/stage2/state`, `/follow/stage2/goal`,
  `/follow/stage2/debug_goal_path`, `/move_base_simple/goal`,
  `/waypoint_generator/waypoints`, `/planning/bspline`,
  `/follow/stage2/ego_position_cmd`, `/follow/stage2/offboard/setpoint`,
  `/mavros/setpoint_raw/local`

Target-loss/search regression command:

```bash
MONITOR_DURATION_SEC=9.5 LAUNCH_TIMEOUT_SEC=35 \
bash research/scripts/run_paper_line_ros1_regression.sh 1 \
  fixture_scenario_yaml:=$(pwd)/src/human_follow_bringup/config/paper_line_stage2_target_loss.yaml \
  monitor_full_duration_evidence:=true \
  monitor_required_state_names:=follow,predict_hold,search_safe_viewpoint \
  monitor_required_phase_labels:=loss_target_visible_start,short_dropout_predict_hold,reacquired_after_short_loss,long_dropout_search,reacquired_after_search
```

Result: PASS.

Artifact:

- `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002421`

Key evidence:

- `distinct_goals=30`
- `distinct_cmds=22`
- `request_count=1`
- `final_plan_success`: `0=27`, `1=99`
- sampled topics all true.

Interpretation:

- The target-loss/search case exercised harder planner windows because
  `final_plan_success=0` appeared alongside successful commands.
- This supports state-sequence execution through ROS1/EGO, not robust planner
  recovery proof.

## Phase 3: Repeated Regression

Command:

```bash
bash research/scripts/run_paper_line_ros1_regression.sh 5
```

Result: PASS, 5/5.

Artifact:

- `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002516`

Per-run summary:

| run | distinct goals | distinct cmds | final plan success 0 | final plan success 1 | sampled required topics | timeout |
|---:|---:|---:|---:|---:|---|---|
| 1 | 3 | 3 | 0 | 21 | true | false |
| 2 | 3 | 3 | 0 | 23 | true | false |
| 3 | 3 | 3 | 0 | 20 | true | false |
| 4 | 4 | 3 | 0 | 21 | true | false |
| 5 | 3 | 3 | 0 | 22 | true | false |

No `ERROR`, `FATAL`, `Traceback`, `in obstacle`, or timeout was counted by the
runner summaries. Process cleanup was clean.

## Phase 4: Rosbag Metrics

Normal validation bag:

- bag:
  `research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation.bag`
- metrics:
  `research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation_metrics`
- command:

```bash
python3 research/scripts/analyze_stage2_rosbag_metrics.py \
  research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation.bag \
  --validate
```

Result: PASS.

Key metrics:

- `bag_duration_sec=11.4229`
- `goal_count=171`
- `ego_cmd_count=1133`
- `goal_rate_hz=15.0000`
- `ego_cmd_rate_hz=99.9999`
- `goal_max_gap_sec=0.0703`
- `ego_cmd_max_gap_sec=0.0101`
- `bridge_echo_distance_mean_m=0.0072`
- states seen: `follow`

Target-loss/search validation bag:

- bag:
  `research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation.bag`
- metrics:
  `research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation_metrics`
- command:

```bash
python3 research/scripts/analyze_stage2_rosbag_metrics.py \
  research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation.bag \
  --validate \
  --required-states follow,predict_hold,search_safe_viewpoint
```

Result: PASS.

Key metrics:

- `bag_duration_sec=8.8151`
- `goal_count=127`
- `ego_cmd_count=873`
- `goal_rate_hz=14.4832`
- `ego_cmd_rate_hz=100.0000`
- `goal_max_gap_sec=0.1129`
- `ego_cmd_max_gap_sec=0.0101`
- `bridge_echo_distance_mean_m=0.0061`
- `search_count=16`
- state durations:
  `follow=5.714 sec`, `predict_hold=1.887 sec`,
  `search_safe_viewpoint=1.081 sec`

## Conclusion

The paper-line adapter is executable in the Ubuntu20 ROS1/EGO software chain:

- `/follow/stage2/state` and `/follow/stage2/goal` are published;
- EGO ingress receives `/move_base_simple/goal`;
- EGO publishes `/follow/stage2/ego_position_cmd`;
- bridge/fake-MAVROS setpoint chain is active through
  `/follow/stage2/offboard/setpoint` and `/mavros/setpoint_raw/local`;
- target-loss/search states `follow`, `predict_hold`, and
  `search_safe_viewpoint` are observed in ROS bag metrics;
- repeated normal regression passed 5/5.

Supported now:

- ROS1/EGO software-chain executability of the paper-line adapter;
- dynamic safety margin and hard safety filtering remain supported by MATLAB
  and offline diagnostic evidence;
- planner feedback/cooldown remains supported by MATLAB/offline diagnostic
  evidence;
- target-loss/search state execution is now supported at ROS1/EGO software
  level.

Still not supported:

- real-flight safety;
- strict Gazebo/RViz full-chain validation;
- full proposed-method superiority over all baselines;
- prediction, occlusion score, visibility score, or FSM recovery as independent
  main contributions.

## Next Action

Move from validation plumbing to paper/patent evidence packaging:

1. write the method section around dynamic margin, hard filtering, planner
   feedback, failed-candidate cooldown, and failure burst;
2. prepare tables from the Windows/MATLAB and Ubuntu20 logs;
3. add scenario-specific ROS1 fixtures only if they answer a concrete claim,
   especially obstacle-near, planner-blocked, fixed-behind baseline, and
   no-feedback ablation cases;
4. keep Gazebo + RViz strict full-chain simulation as later system-level
   validation, not as the blocker for the current paper-line result package.
