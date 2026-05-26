# Paper-Line ROS1 Stage2 Adapter

This file records the current deployment bridge for paper-line.

## Scope

The project runtime remains ROS1 / catkin. MATLAB is used only for research,
prototype validation, and experiment design. The deployed ROS node does not
depend on MATLAB.

The adapter is implemented at:

- `src/human_follow_user/scripts/user_stage2_goal_node.py`

It uses the existing external Stage2 goal-provider slot and does not modify
EGO-Planner, PX4 bridge code, fusion code, or launch topology.

## Interfaces

Inputs:

- `/follow/fusion/target_world`
- `/follow/lio/odom`
- `/grid_map/occupancy_inflate` when obstacle safety filtering is available
- `/follow/stage2/ego_position_cmd` as optional downstream planner feedback

Outputs:

- `/follow/stage2/goal`
- `/follow/stage2/debug_goal_path`
- `/follow/stage2/state`

## Implemented Logic

- CV-KF target state estimate and short-horizon prediction.
- Dynamic safety margin using vehicle speed and prediction covariance.
- Hard safety filtering before candidate scoring.
- Multi-candidate viewpoint generation:
  `behind`, `left`, `right`, `far_safe`, and `search_reacquire_*`.
- Weighted quality scoring after hard filtering.
- Deterministic states:
  `idle`, `follow`, `predict_hold`, `search_safe_viewpoint`,
  `hold_safe`, `failsafe`.
- Optional planner command feedback and failed-candidate cooldown.

## Launch Entry

For the existing Stage2 placeholder path:

```bash
roslaunch human_follow_bringup stage2_placeholder.launch \
  stage2_enabled:=true \
  goal_provider:=external \
  external_goal_pkg:=human_follow_user \
  external_goal_type:=user_stage2_goal_node.py
```

For the existing Stage2/EGO path, keep the existing launch and override only:

```bash
goal_provider:=external \
external_goal_pkg:=human_follow_user \
external_goal_type:=user_stage2_goal_node.py
```

For the repeatable paper-line real-EGO regression entry:

```bash
roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch
```

This launch keeps the baseline regression launch untouched and starts:

- `stage2_goal_input_fixture_node.py`
- `stage2_real_ego.launch` with `goal_provider:=external`
- `human_follow_user/user_stage2_goal_node.py`
- `fake_mavros_gate_harness_node.py`
- `stage2_paper_line_regression_monitor_node.py`

The monitor checks the external-provider chain by topic contract, not by
baseline fixed geometry:

- `/follow/stage2/goal`
- `/move_base_simple/goal`
- `/waypoint_generator/waypoints`
- `/planning/bspline`
- `/follow/stage2/ego_position_cmd`
- `/follow/stage2/offboard/setpoint`
- `/mavros/setpoint_raw/local`
- one `OFFBOARD` request
- `/follow/stage2/state` with paper-line candidate detail
- `/follow/stage2/debug_goal_path`

For repeated local evidence:

```bash
research/scripts/run_paper_line_ros1_regression.sh 5
```

The runner writes ignored artifacts under:

```text
.codex/artifacts/paper_line_ros1_adapter_<date>/formal_real_ego_regression_<time>/
```

## Verification Status

Completed in the current workspace on `paper-line` commit `daaf3bb`:

```bash
python3 -m py_compile src/human_follow_user/scripts/user_stage2_goal_node.py
python3 -m py_compile research/scripts/smoke_ros1_stage2_adapter_core.py
python3 -m py_compile research/scripts/smoke_ros1_stage2_adapter_scenarios.py
python3 -m py_compile src/human_follow_bringup/scripts/stage2_paper_line_regression_monitor_node.py
```

Offline core smoke:

```text
offline smoke PASS candidate=behind score=0.806 margin=0.846
```

Offline scenario smoke:

```text
scenario smoke PASS normal=behind short_loss=predict_hold long_loss=search_safe_viewpoint expired_loss=hold_safe hard_filter=behind_rejected cooldown=blocked_then_expired
```

Catkin build:

- `catkin_make` failed on this host before package configuration because
  CMake 4.2 rejects the system `/usr/src/googletest` minimum-version policy.
- `catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5` passed.

Formal real-EGO regression:

- Artifact:
  `.codex/artifacts/paper_line_ros1_adapter_20260526/formal_real_ego_regression_234357`
- Result: 5/5 PASS.
- Each run had `distinct_goals=3`, `distinct_cmds=3`, and `request_count=1`.
- Each run sampled `/planning/bspline` and `/follow/stage2/ego_position_cmd`.
- `/follow/stage2/offboard/setpoint` and `/mavros/setpoint_raw/local` were
  confirmed by the monitor contract and `offboard_mode_gate` logs.
- Fresh formal runs had `final_plan_success=0` count `0`; previous manual
  smoke had startup `final_plan_success=0` transients followed by
  `final_plan_success=1`, so this remains a fact to keep watching.
- No `ERROR`, `FATAL`, `Traceback`, or `in obstacle` lines were counted by the
  runner in these five formal runs.
- Process cleanup was clean in all five runs.

## Experiment Data Interface

Each ROS1 regression run should write one ignored artifact directory containing
raw logs, sampled topics, and `run_summary.json`. A batch directory should also
write `summary.json` and `summary.md`.

Current `run_summary.json` fields:

- `passed`
- `distinct_goals`
- `distinct_cmds`
- `request_count`
- `sampled_topics`
- `setpoint_contract_confirmed_by_monitor`
- `final_plan_success`
- `error_keyword_count`
- `process_clean`

Required baselines for later paper-line experiments:

- original baseline Stage2 goal generator
- paper-line adapter without prediction
- paper-line adapter without dynamic margin
- paper-line adapter without hard filter
- paper-line full adapter

Required metrics for later experiment expansion:

- follow distance error
- view angle error
- minimum obstacle distance
- safety-shell violation count
- target-loss recovery time
- planner-failure recovery time
- trajectory smoothness
- goal update latency
- EGO command continuity

The current ROS1 regression is a repeatable chain check plus limited offline
decision-layer smoke. It is not yet a full baseline/ablation experiment.

## Claim Boundary

The current evidence supports saying:

- the paper-line adapter has a formal ROS1 real-EGO regression launch;
- the launch can repeatedly exercise the external Stage2 goal-provider chain;
- core decision-layer scenario smoke covers normal follow, target-loss states,
  hard filtering, and failed-candidate cooldown at offline logic level.

Do not claim from this evidence:

- hardware readiness;
- robust behavior across scenes;
- strict obstacle avoidance;
- strict target-loss recovery;
- strict planner-failure protection;
- paper contribution superiority over all baselines;
- readiness to merge into the hardware mainline.
