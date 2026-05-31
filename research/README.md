# Research Workspace

Use this directory for paper-line artifacts only.

Suggested layout:

- `notes/` for reading notes, decision logs, and experiment logs
- `prompts/` for reusable Codex prompts
- `matlab/` for scripts, functions, and Live Script drafts
- `figures/` for checked-in figures
- `data/` for small curated inputs
- `outputs/` for generated results, ignored by git
- `figures_generated/` for generated figures, ignored by git
- `runs/` for ad hoc experiment runs, ignored by git

Working order:

1. Converge the paper-line direction.
2. Read literature and extract reusable ideas.
3. Build the smallest MATLAB prototype.
4. Add prediction, scoring, and state-machine logic.
5. Generate plots and a first paper outline.

Python offline experiments:

```bash
python3 research/scripts/run_stage2_adapter_experiments.py --seeds 1:5
```

This runs a software-only baseline/ablation matrix against the ROS1 Stage2
adapter decision core. It writes `runs.csv`, `by_condition.csv`,
`by_scenario_condition.csv`, `manifest.json`, and `summary.md` under
`research/runs/stage2_adapter_experiments/<timestamp>/`.

The Python runner is diagnostic evidence only. It is useful on machines without
MATLAB and before ROS1/EGO runtime validation, but it is not hardware evidence.

ROS bag metric extraction on Ubuntu 20.04 + ROS1:

```bash
bash research/scripts/record_stage2_validation_bag.sh
python3 research/scripts/analyze_stage2_rosbag_metrics.py path/to/stage2_run.bag
```

This writes a compact metric bundle next to the bag, including topic counts,
latency/continuity proxies, state durations, and a markdown summary.

Keep this tree separate from the Ubuntu mainline deployment work.
