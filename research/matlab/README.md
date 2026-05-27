# MATLAB Workspace

Put paper-line prototype code here.

Current structure:

- `+paperline/` for the reusable MATLAB package functions
- `tests/` for class-based `matlab.unittest` tests
- `runPaperLineDemo.m` for the first end-to-end deterministic demo
- `runPaperLineBatch.m` for stage-one scenarios, baselines, and ablations
- `runPaperLineStressBatch.m` for stress scenarios
- `summarizePaperLineResults.m` for post-run key comparisons

Rules:

- Keep scripts deterministic.
- Keep generated outputs in `../outputs/` or `../figures_generated/`.
- Prefer plain MATLAB scripts and functions before Simulink.
- Keep code readable on R2024b.

First run:

```matlab
cd research/matlab
result = runPaperLineDemo;
runtests("tests")
```

`runPaperLineDemo` opens visible figures by default. Use
`runPaperLineDemo(ShowFigures=false)` for headless batch runs.

Batch experiments:

```matlab
summary = runPaperLineBatch(Seeds=1:3, SaveOutputs=true);
```

Generated batch outputs:

- `../outputs/paper_line_batch/stage1_runs.csv`
- `../outputs/paper_line_batch/stage1_by_condition.csv`
- `../outputs/paper_line_batch/stage1_by_scenario_condition.csv`
- `../figures_generated/paper_line_batch/stage1_condition_summary.png`

Stress experiments:

```matlab
summary = runPaperLineStressBatch(Seeds=1:3, SaveOutputs=true);
```

Generated stress outputs:

- `../outputs/paper_line_stress/stress_runs.csv`
- `../outputs/paper_line_stress/stress_by_condition.csv`
- `../outputs/paper_line_stress/stress_by_scenario_condition.csv`
- `../figures_generated/paper_line_stress/stress_condition_summary.png`

After batch and stress outputs exist, print the key comparisons:

```matlab
report = summarizePaperLineResults;
```

Current diagnostic status:

- Visibility uses a target-facing camera heading model and has unit tests.
- Candidate viewpoints carry both point and heading.
- Metrics include tracking, recovery, safety, and composite task success.
- Results are diagnostic only. They support keeping dynamic safety margin but
  do not yet prove the full proposed method dominates all baselines.
- Stress results support keeping planner command feedback and failed-candidate
  cooldown because repeated failure bursts are reduced.
- This 2-D MATLAB baseline is now a diagnostic baseline, not a paper-result
  endpoint. Prediction placement, occlusion score, visibility score, and FSM
  recovery are retained as optional mechanisms needing stronger future evidence.
