# 2026-05-31 Windows MATLAB Refresh Log

Scope: paper-line only.

## Purpose

Refresh the algorithm-only MATLAB evidence after the branch was handed back to
the Windows/MATLAB side. This run follows
`docs/WINDOWS_PAPER_LINE_HANDOFF.md` and
`docs/PAPER_LINE_MATLAB_EXPERIMENT_PLAN_CN.md`.

This is not ROS, Gazebo, PX4, EGO internals, or hardware evidence.

## Repository State

- Branch: `paper-line`
- Remote sync: local branch matched `origin/paper-line`
- Worktree before MATLAB run: clean
- Generated CSV and figures remain under ignored paths:
  - `research/outputs/`
  - `research/figures_generated/`

## MATLAB Environment

- MATLAB: R2024b, `24.2.0.2712019`
- MATLAB working folder:
  `\\wsl.localhost\ubuntu-22.04\home\J_ws\follwer\research\matlab`

## Commands Run

```matlab
results = runtests("tests");
demo = runPaperLineDemo(ShowFigures=false);
batch1 = runPaperLineBatch(Seeds=1, SaveOutputs=false);
stress1 = runPaperLineStressBatch(Seeds=1, SaveOutputs=false);
batch3 = runPaperLineBatch(Seeds=1:3, SaveOutputs=true);
stress3 = runPaperLineStressBatch(Seeds=1:3, SaveOutputs=true);
batch5 = runPaperLineBatch(Seeds=1:5, SaveOutputs=true);
stress5 = runPaperLineStressBatch(Seeds=1:5, SaveOutputs=true);
report = summarizePaperLineResults;
```

## Verification

- `runtests("tests")`: 11 passed, 0 failed, 0 incomplete.
- `runPaperLineDemo(ShowFigures=false)`: completed.
- Seed-1 smoke batch: completed.
- Seed-1 smoke stress: completed.
- Seed-1:3 batch and stress: completed and saved outputs.
- Seed-1:5 batch and stress: completed and saved outputs.

Final run counts:

- Stage-one batch: 6 scenarios, 10 conditions, 5 seeds, 300 runs.
- Stress batch: 5 scenarios, 9 conditions, 5 seeds, 225 runs.

## Generated Outputs

Stage-one batch:

- `research/outputs/paper_line_batch/stage1_runs.csv`
- `research/outputs/paper_line_batch/stage1_by_condition.csv`
- `research/outputs/paper_line_batch/stage1_by_scenario_condition.csv`
- `research/figures_generated/paper_line_batch/stage1_condition_summary.png`

Stress batch:

- `research/outputs/paper_line_stress/stress_runs.csv`
- `research/outputs/paper_line_stress/stress_by_condition.csv`
- `research/outputs/paper_line_stress/stress_by_scenario_condition.csv`
- `research/figures_generated/paper_line_stress/stress_condition_summary.png`

These files are generated evidence for local inspection and are not intended to
be committed as default branch artifacts.

## Stage-One Condition Results

| condition | visible ratio | loss duration | min clearance | near-miss | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| ca_kf | 0.8981 | 2.4567 | 1.8292 | 0.0000 | 0.3000 | 0.1667 | 0.7000 |
| fixed_behind | 0.9166 | 2.0100 | 1.7124 | 0.0000 | 0.0000 | 0.0000 | 1.0000 |
| fixed_safety_margin | 0.9570 | 1.0367 | 1.2829 | 67.0333 | 0.3333 | 0.1667 | 0.1667 |
| nearest_feasible | 0.9495 | 1.2167 | 1.7052 | 0.0000 | 0.0000 | 0.0000 | 1.0000 |
| no_occlusion_score | 0.9297 | 1.6933 | 1.6713 | 0.0000 | 0.1667 | 0.1667 | 1.0000 |
| no_planner_feedback | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 2.6667 | 2.6667 | 0.8333 |
| no_prediction | 0.9411 | 1.4200 | 1.8872 | 0.0000 | 0.2333 | 0.1667 | 1.0000 |
| no_recovery_fsm | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 0.3333 | 0.1667 | 1.0000 |
| no_visibility_score | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 0.3333 | 0.1667 | 1.0000 |
| proposed | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 0.3333 | 0.1667 | 1.0000 |

## Stress Condition Results

| condition | visible ratio | loss duration | min clearance | near-miss | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| fixed_behind | 0.8043 | 4.7160 | 1.6758 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| fixed_safety_margin | 0.9650 | 0.8440 | 0.8458 | 107.8000 | 1.4000 | 0.4000 | 0.0000 |
| nearest_feasible | 0.8309 | 4.0760 | 1.4969 | 0.0000 | 1.2000 | 0.4000 | 0.8400 |
| no_occlusion_score | 0.8244 | 4.2320 | 1.5458 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| no_planner_feedback | 0.8551 | 3.4920 | 1.3177 | 0.0000 | 15.4000 | 15.4000 | 0.6000 |
| no_prediction | 0.9129 | 2.1000 | 1.6739 | 0.0000 | 1.1200 | 0.4000 | 1.0000 |
| no_recovery_fsm | 0.8280 | 4.1440 | 1.5332 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| no_visibility_score | 0.8264 | 4.1840 | 1.5504 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| proposed | 0.8264 | 4.1840 | 1.5504 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |

## Scenario-Level Diagnostics

Fixed safety margin produced near-miss events in:

- Stage-one: `obstacle_occlusion`, `occlusion_reacquire`,
  `planner_failure`, `short_loss`, `sudden_turn`.
- Stress: all five stress scenarios.

No planner feedback produced long repeated failure bursts in:

- Stage-one `planner_failure`: `plannerFailureBurstMax_mean = 16`.
- Stress `planner_blocked_goal`: `plannerFailureBurstMax_mean = 16`.
- Stress `planner_feedback_stress`: `plannerFailureBurstMax_mean = 61`.

Prediction did not produce an independent advantage:

- Stage-one `no_prediction` matched proposed task success and improved
  visibility/loss duration in aggregate.
- Stress `no_prediction` had task success 1.0 while proposed had 0.8.
- In stress `planner_feedback_stress`, proposed task success was 0 while
  `no_prediction` was 1.0.

## Supported Claims

The refreshed data supports these as the current paper/patent spine:

- Dynamic safety margin reduces near-miss behavior relative to fixed safety
  margin in this diagnostic suite.
- Hard safety filtering before scoring remains necessary because fixed-margin
  and weaker safety handling can look better in visibility while violating the
  safety shell.
- Planner command-health feedback plus failed-candidate cooldown reduces
  repeated infeasible-command bursts.
- Failure burst length is a useful metric; total planner failure count alone is
  less informative.

## Unsupported Or Weak Claims

Do not claim these as independent contributions from the current MATLAB data:

- prediction improves following performance;
- CA-KF is better than the default CV-KF;
- occlusion score is independently proven;
- visibility score is independently proven;
- FSM recovery is independently proven;
- the full proposed method dominates all baselines.

These mechanisms can remain in the system as engineering components, but the
claim language must stay narrower until ROS1/EGO or later experiments produce
stronger evidence.

## Next Stage Boundary

Do not keep tuning the 2-D MATLAB prototype just to make the full proposed row
win. The next meaningful stage is Ubuntu 20.04 + ROS1 validation using
`docs/PAPER_LINE_UBUNTU20_VALIDATION_MATRIX.md`.

The purpose of that stage is to verify executable integration and collect ROS
topic evidence, not to re-open Ubuntu 22.04 ROS2 local temporary simulation.

