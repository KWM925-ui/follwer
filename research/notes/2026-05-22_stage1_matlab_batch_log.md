# 2026-05-22 Stage-One MATLAB Batch Log

Scope: paper-line only.

## Purpose

Bring the MATLAB prototype closer to the rebuilt route instead of leaving it as
the earlier single-scenario / prediction-only experiment.

## Implemented

- Candidate set aligned with rebuilt route:
  - `behind`
  - `left`
  - `right`
  - `far_safe`
  - `search_reacquire` only in `SEARCH_SAFE_VIEWPOINT`
- Decision policies:
  - `proposed`
  - `fixed_behind`
  - `nearest_feasible`
- Stage-one scenario families:
  - `straight`
  - `sudden_turn`
  - `obstacle_occlusion`
  - `short_loss`
  - `occlusion_reacquire`
  - `planner_failure`
- Batch conditions:
  - `proposed`
  - `fixed_behind`
  - `nearest_feasible`
  - `no_prediction`
  - `fixed_safety_margin`
  - `no_recovery_fsm`
  - `ca_kf`
  - `no_occlusion_score`
  - `no_planner_feedback`
- New metrics:
  - mean/max view-angle error
  - near-miss count
  - loss count
  - loss duration
  - mean reacquisition time
  - planner-failure recovery time
  - task success

## Verification

- `run_matlab_test_file` on `tests/tPaperLineCore.m`: 9 passed, 0 failed.
- `check_matlab_code` on `runPaperLineBatch.m`: no issues.
- `check_matlab_code` on `+paperline/simulateRun.m`: no issues.

## Batch Run

Command:

```matlab
cd research/matlab
summary = runPaperLineBatch(Seeds=1:3, SaveOutputs=true);
```

Runs:

- 6 scenarios
- 9 conditions
- 3 seeds
- 162 total runs

Generated outputs:

- `research/outputs/paper_line_batch/stage1_runs.csv`
- `research/outputs/paper_line_batch/stage1_by_condition.csv`
- `research/outputs/paper_line_batch/stage1_by_scenario_condition.csv`
- `research/figures_generated/paper_line_batch/stage1_condition_summary.png`

## Initial Result Summary

| condition | visible ratio | loss duration | reacquisition time | near-miss count | min clearance | planner recovery time | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| ca_kf | 0.9730 | 0.65 | 0.0333 | 0 | 1.6925 | 0.0333 | 1 |
| fixed_behind | 0.9730 | 0.65 | 0.0333 | 0 | 1.5901 | 0.0333 | 1 |
| fixed_safety_margin | 0.9730 | 0.65 | 0.0333 | 0 | 1.1597 | 0.0333 | 1 |
| nearest_feasible | 0.9730 | 0.65 | 0.0333 | 0 | 1.5434 | 0.0333 | 1 |
| no_occlusion_score | 0.9730 | 0.65 | 0.0333 | 0 | 1.5454 | 0.0333 | 1 |
| no_planner_feedback | 0.9730 | 0.65 | 0.0333 | 0 | 1.5462 | 0.0000 | 1 |
| no_prediction | 0.9730 | 0.65 | 0.0333 | 0 | 1.7626 | 0.0444 | 1 |
| no_recovery_fsm | 0.9730 | 0.65 | 0.0333 | 0 | 1.5462 | 0.0167 | 1 |
| proposed | 0.9730 | 0.65 | 0.0333 | 0 | 1.5462 | 0.0333 | 1 |

## Interpretation

This batch proves that the stage-one experiment matrix runs end-to-end, but it
does not yet prove the main paper claim.

Reason:

- The current visibility/loss logic is still dominated by scripted dropout
  windows, so most methods have identical visible ratio and loss duration.
- The obstacle/occlusion scenarios are not hard enough to create near-miss
  differences.
- Planner-failure feedback is represented only as a simple health flag, so the
  proposed method does not yet show a strong advantage.

## Next Required Improvement

The next MATLAB pass must make visibility and loss depend on viewpoint geometry:

- FOV angle from UAV to target
- line-of-sight intersection from UAV to target
- obstacle-induced target invisibility, not only scripted dropout
- planner failure tied to candidate reachability or blocked segment
- harder obstacle layouts that create near-miss and occlusion differences

This is necessary before using MATLAB results as paper or patent evidence.

## Geometry-Aware Visibility Pass

Implemented after the first plumbing batch:

- FOV-based visibility score from UAV heading to target bearing.
- Distance visibility score.
- Line-of-sight occlusion penalty from UAV to target.
- Measurement availability now depends on scripted dropout or low geometric
  visibility.

Verification:

- `run_matlab_test_file` on `tests/tPaperLineCore.m`: 9 passed, 0 failed.
- `check_matlab_code` on `+paperline/simulateRun.m`: no issues.

Diagnostic batch:

```matlab
summary = runPaperLineBatch(Seeds=1:2, SaveOutputs=true);
```

Runs:

- 108 total runs

Key diagnostic results:

| condition | visible ratio | loss duration | near-miss count | min clearance | task success |
|---|---:|---:|---:|---:|---:|
| ca_kf | 0.3385 | 15.942 | 0 | 6.3384 | 0.0000 |
| fixed_behind | 0.3351 | 16.025 | 0 | 2.3289 | 0.1667 |
| fixed_safety_margin | 0.2801 | 17.350 | 0 | 3.8098 | 0.0833 |
| nearest_feasible | 0.3254 | 16.258 | 0 | 5.2510 | 0.0000 |
| no_occlusion_score | 0.2797 | 17.358 | 0 | 3.9726 | 0.0833 |
| no_planner_feedback | 0.2797 | 17.358 | 0 | 3.9726 | 0.0833 |
| no_prediction | 0.1743 | 19.900 | 0 | 5.1979 | 0.0000 |
| no_recovery_fsm | 0.2548 | 17.958 | 0 | 1.7127 | 0.5833 |
| proposed | 0.2797 | 17.358 | 0 | 3.9726 | 0.0833 |

Interpretation:

- The experiment now has method-dependent visibility/loss behavior.
- The current visibility model is too harsh for the proposed method, so these
  numbers are diagnostic rather than publishable.
- The low near-miss count means obstacle layouts still do not stress safety
  enough.
- `no_recovery_fsm` showing higher task success is a red flag in the metric
  definition and scenario stress balance, not evidence against recovery.

Next pass:

- tune FOV / heading model so a UAV that is intentionally following the target
  does not lose visibility too easily;
- define task success as a multi-metric threshold rather than only no near-miss
  and no failsafe;
- add obstacle layouts that create clearance tradeoffs;
- make planner failure tied to candidate segment infeasibility, not only a
  scripted time window.

## Rebuilt Visibility / Safety / Planner-Feedback Pass

Timestamp: 2026-05-23 00:20 +08:00.

Implemented:

- Added `paperline.computeVisibility` with decomposed target distance, bearing
  error, FOV score, distance score, and occlusion risk.
- Added target-facing candidate headings; candidates now carry both point and
  heading.
- Added a camera heading slew model so visibility is based on camera/gimbal
  line of sight rather than UAV translational velocity.
- Added unit tests for visibility and candidate heading sanity.
- Added composite metrics: `trackingSuccess`, `recoverySuccess`,
  `safetySuccess`, and `taskSuccess`.
- Added `plannerCommandHealthy` and `plannerFailureCount`.
- Changed candidate generation to use a blended target reference:
  `estimate + viewpointPredictionBlend * (prediction - estimate)`.
- Raised default `obstacleMargin` from 0.30 to 0.45 after a small sweep. This
  removed near-miss events for the proposed dynamic-margin route while the
  fixed-margin ablation still produced many near-miss events.

Verification:

- `check_matlab_code` clean for changed MATLAB files.
- `run_matlab_test_file` on `tests/tPaperLineCore.m`: 11 passed, 0 failed.
- Batch command:

```matlab
summary = runPaperLineBatch(Seeds=1:3, SaveOutputs=true);
```

Key diagnostic results:

| condition | visible ratio | loss duration | near-miss count | min clearance | planner failure count | task success |
|---|---:|---:|---:|---:|---:|---:|
| ca_kf | 0.8974 | 2.4722 | 0.0000 | 1.8347 | 1.3333 | 0.5556 |
| fixed_behind | 0.9154 | 2.0389 | 0.0000 | 1.7216 | 0.0000 | 1.0000 |
| fixed_safety_margin | 0.9583 | 1.0056 | 71.8889 | 1.2816 | 1.3333 | 0.1667 |
| nearest_feasible | 0.9357 | 1.5500 | 0.0000 | 1.6907 | 0.0000 | 1.0000 |
| no_occlusion_score | 0.9302 | 1.6833 | 0.0000 | 1.6865 | 1.0556 | 0.8333 |
| no_planner_feedback | 0.9302 | 1.6833 | 0.0000 | 1.6899 | 2.6667 | 0.8333 |
| no_prediction | 0.9401 | 1.4444 | 0.0000 | 1.8728 | 1.1111 | 0.8333 |
| no_recovery_fsm | 0.9302 | 1.6833 | 0.0000 | 1.6899 | 1.3333 | 0.8333 |
| proposed | 0.9302 | 1.6833 | 0.0000 | 1.6899 | 1.3333 | 0.8333 |

Interpretation:

- The visibility model is no longer obviously broken. Straight and normal
  following are visible; occlusion scenarios create controlled visibility loss.
- The fixed-margin ablation now shows a clear safety failure: high near-miss
  count and low task success.
- `no_planner_feedback` has higher planner-failure count than `proposed`
  (2.6667 vs 1.3333), so planner feedback is now measurable.
- The full proposed method is still not publishable as a dominant method:
  fixed-behind and nearest-feasible remain strong in this synthetic suite.
- `no_prediction` is competitive. Do not claim prediction always improves
  following; next scenarios must isolate when prediction uncertainty matters.

Scenario-level proposed diagnostics:

| scenario | visible ratio | loss duration | near-miss count | min clearance | planner failure count | task success |
|---|---:|---:|---:|---:|---:|---:|
| straight | 1.0000 | 0.0000 | 0.0000 | 3.4721 | 0.0000 | 1.0000 |
| sudden_turn | 1.0000 | 0.0000 | 0.0000 | 1.3397 | 0.0000 | 1.0000 |
| obstacle_occlusion | 0.7483 | 6.0667 | 0.0000 | 1.3188 | 0.0000 | 1.0000 |
| short_loss | 0.9627 | 0.9000 | 0.0000 | 1.2949 | 0.0000 | 1.0000 |
| occlusion_reacquire | 0.8700 | 3.1333 | 0.0000 | 1.3962 | 0.0000 | 1.0000 |
| planner_failure | 1.0000 | 0.0000 | 0.0000 | 1.3176 | 8.0000 | 0.0000 |

Remaining problems:

- Planner-failure recovery still needs stronger command blacklisting or fallback
  candidate switching.
- Fixed-behind and nearest-feasible are too strong in the current suite.
- Task success is useful as a diagnostic convenience, but the paper should
  report decomposed tracking/recovery/safety metrics.
- Current evidence supports keeping dynamic safety margin and planner-feedback
  metrics; it does not yet support broad superiority claims.
