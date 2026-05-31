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

Ubuntu20 acceptance checklist:

- `docs/PAPER_LINE_UBUNTU20_VALIDATION_MATRIX.md`

Preferred paper-line regression entry:

```bash
roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch
```

Repeated regression runner:

```bash
bash research/scripts/run_paper_line_ros1_regression.sh 5
```

Offline baseline/ablation runner:

```bash
python3 research/scripts/run_stage2_adapter_experiments.py --seeds 1:5
```

The offline runner uses the adapter's decision core without ROS master. Outputs
are written under `research/runs/` and include per-run CSV, condition summaries,
scenario-condition summaries, a manifest, and a short markdown report.

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

The runner also accepts extra `roslaunch` arguments after the run count. This is
used for scenario-specific evidence, for example:

```bash
research/scripts/run_paper_line_ros1_regression.sh 1 \
  fixture_scenario_yaml:=$(pwd)/src/human_follow_bringup/config/paper_line_stage2_target_loss.yaml \
  monitor_full_duration_evidence:=true
```

## Verification Status

Run these checks after pulling the branch on Ubuntu 20.04 + ROS1:

```bash
python3 -m py_compile src/human_follow_user/scripts/user_stage2_goal_node.py
python3 -m py_compile research/scripts/smoke_ros1_stage2_adapter_core.py
python3 -m py_compile research/scripts/smoke_ros1_stage2_adapter_scenarios.py
python3 -m py_compile src/human_follow_bringup/scripts/stage2_paper_line_regression_monitor_node.py
python3 research/scripts/smoke_ros1_stage2_adapter_core.py
python3 research/scripts/smoke_ros1_stage2_adapter_scenarios.py
python3 research/scripts/run_stage2_adapter_experiments.py --seeds 1:5
catkin_make
roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch
bash research/scripts/run_paper_line_ros1_regression.sh 5
```

Known evidence from the paper-line preparation branch:

- Offline core smoke has passed with `behind` as the open-scene candidate.
- Offline scenario smoke covers open follow, behind-blocked side selection,
  switching bias, target-loss state transitions, hard filtering, and
  failed-candidate cooldown.
- MATLAB-free offline experiments have run diagnostic baseline/ablation
  variants. The latest diagnostic result was:

```text
offline smoke PASS candidate=behind score=0.806 margin=0.846
offline scenario smoke PASS open_follow:behind:0.806:1.126 behind_blocked:left:0.755:1.126 switching_bias:left:0.810:1.126
stage2 adapter experiments PASS runs=175 ... best=fixed_behind success=1.000
```

Current diagnostic result boundary:

- `no_hard_filter` produces safety-shell violations, supporting the hard-filter
  design.
- `no_planner_feedback` produces repeated planner failures in the blocked
  planner scenario, supporting feedback/cooldown.
- `paper_line_full` is not yet dominant over all baselines in the offline
  diagnostic suite, so it must not be claimed as globally superior.
- Plain `catkin_make` on the WSL2 Ubuntu 22.04 preparation host hit a CMake
  4.2 `/usr/src/googletest` policy issue before package configuration;
  `catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5` passed there. The target
  runtime remains Ubuntu 20.04 + ROS1.
- A formal real-EGO regression batch previously passed 5/5 with sampled
  `/planning/bspline`, `/follow/stage2/ego_position_cmd`, bridge setpoints,
  and fake MAVROS setpoints.
- Scenario widening added YAML-driven fixtures:
  `src/human_follow_bringup/config/paper_line_stage2_normal.yaml` and
  `src/human_follow_bringup/config/paper_line_stage2_target_loss.yaml`.
- Target-loss full-duration evidence has observed `follow`, `predict_hold`,
  and `search_safe_viewpoint` plus `search_reacquire_*` candidates. Harder
  target-loss windows can still produce many `final_plan_success=0` lines, so
  this is state-sequence evidence, not robust recovery proof.

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
