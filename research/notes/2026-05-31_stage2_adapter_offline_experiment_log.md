# 2026-05-31 Stage2 Adapter Offline Experiment Log

Scope: paper-line only.

## Purpose

Add a MATLAB-free diagnostic experiment runner for Ubuntu 20.04 handoff and
paper-line evidence collection. This is not hardware validation and not a
replacement for ROS1/EGO regression.

## Added Runner

- `research/scripts/run_stage2_adapter_experiments.py`

The runner imports the ROS1 adapter decision core with stubs and evaluates:

- `fixed_behind`
- `nearest_feasible`
- `paper_line_full`
- `no_prediction`
- `fixed_safety_margin`
- `no_hard_filter`
- `no_planner_feedback`

Scenarios:

- `open_straight`
- `sharp_turn`
- `occlusion_corridor`
- `target_loss_reacquire`
- `planner_blocked`

Metrics include:

- follow distance error
- view angle error
- minimum obstacle clearance
- safety-shell violations
- safe-candidate ratio
- target-loss recovery time
- planner failure count and burst length
- candidate/state switches
- goal update latency
- EGO-command continuity proxy
- trajectory smoothness proxy

## Verification

Commands:

```bash
python3 -m py_compile research/scripts/run_stage2_adapter_experiments.py
python3 research/scripts/run_stage2_adapter_experiments.py --seeds 1:5
```

Observed:

```text
stage2 adapter experiments PASS runs=175 ... best=fixed_behind success=1.000
```

Generated outputs are ignored by git under:

- `research/runs/stage2_adapter_experiments/20260531_144415/`

## Current Findings

- `no_hard_filter` produced safety-shell violations, supporting hard safety
  filtering before scoring.
- `no_planner_feedback` produced repeated planner-failure bursts in the blocked
  planner scenario, supporting planner feedback and failed-candidate cooldown.
- `paper_line_full` is not globally dominant over fixed-behind or
  fixed-safety-margin baselines in this diagnostic suite.

## Claim Boundary

Do not claim full-method superiority from this run.

Use this run as diagnostic support for:

- hard filtering as a necessary safety layer;
- planner feedback/cooldown as a recovery mechanism;
- the need for broader ROS1/EGO experiments before paper-level performance
  claims.
