# Paper-Line Experiment Matrix

Status: Ubuntu20 ROS1 evidence refreshed, 2026-06-01.

Purpose: turn paper-line work into paper/patent evidence, not only a chain-alive
ROS smoke test.

For the Windows/MATLAB execution plan, read:

- `docs/PAPER_LINE_MATLAB_EXPERIMENT_PLAN_CN.md`

For the current evidence package, read:

- `docs/PAPER_LINE_EVIDENCE_PACKAGE_CN.md`
- `research/notes/2026-05-31_windows_matlab_refresh_log.md`
- `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`

## Claim Boundary

Current ROS1 evidence supports these claims:

- the paper-line external Stage2 goal provider can drive the real EGO chain in
  a repeatable ROS1 regression;
- the fixture can now load scenario definitions from YAML;
- a full-duration target-loss fixture can exercise `follow`, `predict_hold`,
  and `search_safe_viewpoint` states while the EGO chain remains connected.
- normal ROS1/EGO repeated regression passed 5/5 on Ubuntu20;
- normal and target-loss/search rosbag metrics passed.

Do not claim:

- hardware readiness;
- collision-free behavior;
- robust target-loss recovery;
- planner-failure protection completeness;
- superiority over all baselines.

## Evidence Levels

| level | purpose | current entry |
|---|---|---|
| offline core smoke | decision-layer sanity without ROS graph | `research/scripts/smoke_ros1_stage2_adapter_core.py` |
| offline scenario smoke | state/filter/cooldown unit-like scenarios | `research/scripts/smoke_ros1_stage2_adapter_scenarios.py` |
| ROS chain regression | Stage2 goal to EGO/PX4-bridge topic contract | `stage2_paper_line_real_ego_regression.launch` |
| ROS full-duration scenario evidence | run scenario long enough to observe required states/phases | same launch with `monitor_full_duration_evidence:=true` |
| MATLAB batch/stress | broad baseline and ablation tables | Windows-side MATLAB only |

## ROS1 Scenario Fixtures

Scenario YAML files live under `src/human_follow_bringup/config/`.

Current fixtures:

- `paper_line_stage2_normal.yaml`
  - three visible-target phases;
  - intended for stable full-chain regression compatibility.
- `paper_line_stage2_target_loss.yaml`
  - short target dropout for `predict_hold`;
  - longer dropout for `search_safe_viewpoint`;
  - reacquisition phases before and after loss windows.

The Stage2 input fixture accepts:

```bash
fixture_scenario_yaml:=/path/to/scenario.yaml
```

## Current ROS Evidence

Latest Ubuntu20 ROS1/EGO evidence:

- normal single regression:
  `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002359`
- target-loss/search single regression:
  `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002421`
- normal repeated regression:
  `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002516`
- normal bag metrics:
  `research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation_metrics`
- target-loss/search bag metrics:
  `research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation_metrics`

Summary:

- normal repeated regression: 5/5 PASS;
- normal bag: `goal_count=171`, `ego_cmd_count=1133`;
- target-loss/search bag: `goal_count=127`, `ego_cmd_count=873`;
- target-loss/search bag observed `follow`, `predict_hold`,
  `search_safe_viewpoint`.

Baseline-compatible YAML fixture:

```bash
research/scripts/run_paper_line_ros1_regression.sh 1
```

Latest local result:

- artifact: `.codex/artifacts/paper_line_ros1_adapter_20260527/yaml_fixture_regression_010441`
- result: 1/1 PASS
- `distinct_goals=3`
- `distinct_cmds=3`
- `request_count=1`
- `/planning/bspline` and `/follow/stage2/ego_position_cmd` sampled

Target-loss full-duration evidence:

```bash
MONITOR_DURATION_SEC=9.5 LAUNCH_TIMEOUT_SEC=25 \
research/scripts/run_paper_line_ros1_regression.sh 1 \
  fixture_scenario_yaml:=$(pwd)/src/human_follow_bringup/config/paper_line_stage2_target_loss.yaml \
  monitor_full_duration_evidence:=true \
  monitor_required_state_names:=follow,predict_hold,search_safe_viewpoint \
  monitor_required_phase_labels:=loss_target_visible_start,short_dropout_predict_hold,reacquired_after_short_loss,long_dropout_search,reacquired_after_search
```

Latest local result:

- artifact: `.codex/artifacts/paper_line_ros1_adapter_20260527/target_loss_full_duration_010748`
- result: 1/1 PASS
- observed states: `follow`, `predict_hold`, `search_safe_viewpoint`
- observed candidates: `behind`, `left`, `right`, `search_reacquire_1`, `search_reacquire_5`
- observed all five target-loss phase labels
- `distinct_goals=32`, `distinct_cmds=23`, `request_count=1`

Important limitation:

- this run counted many `final_plan_success=0` lines during the harder
  target-loss/search window. Treat that as a planning stress observation, not as
  proof of robust recovery.

## Required Paper/Patent Baselines

Keep MATLAB as the primary broad-evaluation tool:

- full proposed paper-line adapter;
- fixed safety margin;
- no planner feedback/cooldown;
- no prediction covariance margin;
- no prediction placement;
- no recovery FSM;
- behind-only candidate set;
- nearest-feasible candidate set.

ROS1 launch-level baselines should be added only where they produce stronger
evidence than MATLAB diagnostics. Prioritize:

1. full paper-line adapter;
2. no planner feedback/cooldown;
3. fixed safety margin;
4. no prediction covariance margin;
5. behind-only candidate set, if implemented cleanly.

## Patent-Relevant Effects To Measure

The most defensible effects remain:

- reduced near-miss or safety-shell violations from dynamic safety margin;
- reduced repeated infeasible command bursts from planner feedback and failed
  candidate cooldown;
- auditable transition from follow to prediction-hold to search viewpoint under
  target loss.

The current ROS evidence only supports the third item as an exercised state
sequence. The first two still need scenario-specific metrics or MATLAB/ROS
ablation evidence before they are patent-claim material.
