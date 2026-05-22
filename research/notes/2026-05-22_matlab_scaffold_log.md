# 2026-05-22 MATLAB Scaffold Log

Scope: paper-line only.

## What Was Added

MATLAB scaffold under `research/matlab/`:

- `runPaperLineDemo.m`
- `+paperline/defaultConfig.m`
- `+paperline/makeScenario.m`
- `+paperline/simulateRun.m`
- `+paperline/predictTarget.m`
- `+paperline/generateCandidates.m`
- `+paperline/filterCandidates.m`
- `+paperline/scoreCandidates.m`
- `+paperline/updateFsm.m`
- `+paperline/lineOfSightRisk.m`
- `+paperline/dynamicSafetyMargin.m`
- `+paperline/minObstacleClearance.m`
- `+paperline/minSegmentObstacleClearance.m`
- `+paperline/computeMetrics.m`
- `+paperline/plotRunSummary.m`
- `tests/tPaperLineCore.m`

## Implemented Route Pieces

- Deterministic 2-D scenario generator.
- Noisy target measurement and dropout windows.
- CV-KF short-horizon predictor.
- Sparse target-frame candidates:
  - `behind`
  - `behind_left`
  - `behind_right`
  - `side_left`
  - `side_right`
  - `far_safe`
  - `search_viewpoint` only in `SEARCH_SAFE_VIEWPOINT`
- Hard safety filtering with speed/covariance-inflated margin.
- 2-D line-of-sight occlusion risk.
- Normalized weighted candidate scoring.
- FSM states:
  - `FOLLOW`
  - `PREDICT_HOLD`
  - `SEARCH_SAFE_VIEWPOINT`
  - `REACQUIRE`
  - `HOLD_SAFE`
  - `FAILSAFE`
- First summary figures.

## How To Run

```matlab
cd research/matlab
result = runPaperLineDemo;
summary = runPaperLineBatch;
runtests("tests")
```

Generated figures:

- `research/figures_generated/paper_line_demo/paper_line_summary.png`
- `research/figures_generated/paper_line_demo/paper_line_scores_metrics.png`

## Verification

MATLAB MCP checks:

- `check_matlab_code` on `runPaperLineDemo.m`: no issues.
- `check_matlab_code` on `+paperline/predictTarget.m`: no issues.
- `check_matlab_code` on `+paperline/plotRunSummary.m`: no issues.
- `check_matlab_code` on `runPaperLineBatch.m`: no issues.
- `run_matlab_test_file` on `tests/tPaperLineCore.m`: 7 passed, 0 failed.
- `runPaperLineDemo` completed and generated both expected figures.
- `runPaperLineBatch` completed with 40 prediction-baseline runs and 40 ablation runs using seeds 1:8.

Observed demo metrics:

```text
meanDistanceError: 0.7430
maxDistanceError: 2.4080
meanPredictionError: 0.6290
maxPredictionError: 0.9159
minClearance: 1.1603
visibilityRatio: 0.8755
meanOcclusionRisk: 0.4168
stateSwitchCount: 9
candidateSwitchCount: 9
safeCandidateRatio: 0.7884
```

Batch outputs:

- `research/outputs/paper_line_batch/prediction_baselines_runs.csv`
- `research/outputs/paper_line_batch/prediction_baselines_summary.csv`
- `research/outputs/paper_line_batch/ablation_runs.csv`
- `research/outputs/paper_line_batch/ablation_summary.csv`
- `research/figures_generated/paper_line_batch/prediction_baselines.png`
- `research/figures_generated/paper_line_batch/component_ablations.png`

Prediction-baseline summary, seeds 1:8:

| condition | mean distance error | mean prediction error | mean min clearance | mean candidate switches |
|---|---:|---:|---:|---:|
| current_measurement | 0.8209 | 0.1470 | 1.5561 | 106.62 |
| cv_extrapolation | 1.1215 | 1.8241 | 1.6324 | 106.88 |
| ca_extrapolation | 4.4925 | 46.6340 | 4.9051 | 99.88 |
| cv_kf | 0.7424 | 0.6318 | 1.1496 | 7.25 |
| ca_kf | 0.8074 | 0.7581 | 1.4604 | 12.75 |

Interpretation:

- `cv_kf` is the current default because it gives the best mean follow-distance error among the tested predictors and sharply reduces candidate switching compared with raw/current or open-loop extrapolation baselines.
- `current_measurement` has low one-step prediction error by definition, but it behaves reactively and switches viewpoints heavily.
- `cv_extrapolation` and `ca_extrapolation` are useful baselines but not suitable as defaults in the dropout/noisy-measurement setting.
- `ca_kf` is retained as an optional baseline; it did not beat `cv_kf` in this first scenario batch.

Ablation summary, seeds 1:8:

| condition | mean distance error | mean prediction error | mean min clearance | mean occlusion risk | mean candidate switches |
|---|---:|---:|---:|---:|---:|
| full | 0.7424 | 0.6318 | 1.1496 | 0.4237 | 7.25 |
| fixed_safety_margin | 0.4256 | 0.6318 | 0.6772 | 0.4218 | 9.25 |
| no_occlusion_score | 0.7048 | 0.6318 | 1.1569 | 0.5074 | 6.25 |
| no_switch_penalty | 0.7451 | 0.6318 | 1.1487 | 0.4224 | 7.25 |
| no_fsm_recovery | 0.7337 | 0.6318 | 1.1496 | 0.4238 | 5.25 |

Interpretation:

- Fixed safety margin can improve distance tracking in this simple scenario, but it sharply reduces clearance; this supports keeping covariance/speed-inflated safety as the safer default.
- Removing occlusion scoring increases occlusion risk, supporting the visibility-aware score term.
- Switch penalty and FSM effects need harder scenarios before strong claims; keep them in the implementation but avoid overclaiming until scenario coverage expands.

MATLAB warning:

- MATLAB reported it could not obtain a change-notification handle for the WSL UNC `tests` folder.
- The warning did not affect test execution.

## Known Gaps

- Candidate-score heatmap is still a simple best-score timeline.
- Planner-in-loop and log replay are intentionally deferred.
- 2.5D is intentionally deferred until the 2-D experiments are stable.
- Batch evidence is still one scenario family with eight seeds; broader obstacle/dropout scenario families are next.
