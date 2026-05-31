# Paper-Line Ubuntu 20.04 Validation Matrix

Scope: paper-line only. This is a ROS1/EGO software-validation checklist, not a
real-flight checklist.

Status on 2026-06-01:

- Phase 0 through Phase 4 passed on Ubuntu 20.04.6 + ROS Noetic.
- Detailed evidence is recorded in
  `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`.
- Re-run this matrix only after code/environment changes or when collecting a
  new scenario-specific artifact.

## Goal

Validate that the paper-line upper decision layer can be repeatedly exercised
between target fusion and EGO goal ingress, and collect evidence that is useful
for later paper and patent work.

## Phase 0: Build And Static Checks

One-command validation entry on Ubuntu 20.04 + ROS1:

```bash
bash research/scripts/run_paper_line_ubuntu20_validation.sh
```

For a non-ROS quick check, use:

```bash
SKIP_CATKIN=1 SKIP_ROS=1 bash research/scripts/run_paper_line_ubuntu20_validation.sh
```

Commands:

```bash
catkin_make
source devel/setup.bash
python3 -m py_compile src/human_follow_user/scripts/user_stage2_goal_node.py
python3 -m py_compile src/human_follow_bringup/scripts/stage2_paper_line_regression_monitor_node.py
python3 -m py_compile research/scripts/run_stage2_adapter_experiments.py
```

Pass criteria:

- catkin build completes;
- Python syntax checks pass;
- no missing-package error for `human_follow_user`, `human_follow_bringup`,
  `quadrotor_msgs`, or `mavros_msgs`.

## Phase 1: MATLAB-Free Offline Diagnostics

Command:

```bash
python3 research/scripts/run_stage2_adapter_experiments.py --seeds 1:5
```

Artifacts:

- `research/runs/stage2_adapter_experiments/<timestamp>/runs.csv`
- `research/runs/stage2_adapter_experiments/<timestamp>/by_condition.csv`
- `research/runs/stage2_adapter_experiments/<timestamp>/by_scenario_condition.csv`
- `research/runs/stage2_adapter_experiments/<timestamp>/summary.md`
- `research/runs/stage2_adapter_experiments/<timestamp>/manifest.json`

Pass criteria:

- script exits successfully;
- `no_hard_filter` shows safety-shell violations in at least one scenario;
- `no_planner_feedback` shows repeated planner failures in
  `planner_blocked`;
- candidate-set ablations `side_only` and `no_far_safe` are recorded so the
  multi-viewpoint candidate design can be defended or narrowed later;
- `paper_line_full` is recorded without claiming global superiority.

Interpretation:

- This phase supports mechanism debugging and claim narrowing.
- It does not prove ROS topic transport, EGO integration, or hardware behavior.

## Phase 2: Paper-Line ROS1/EGO Regression

Command:

```bash
roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch
```

Target-loss/search regression command:

```bash
roslaunch human_follow_bringup stage2_paper_line_search_real_ego_regression.launch
```

Pass criteria:

- `/follow/stage2/state` is published by `paper_line_stage2_goal`;
- `/follow/stage2/goal` reaches `/move_base_simple/goal`;
- EGO publishes `/follow/stage2/ego_position_cmd`;
- the PX4 bridge emits `/follow/stage2/offboard/setpoint`;
- fake MAVROS receives `/mavros/setpoint_raw/local`;
- monitor exits PASS before timeout.
- search regression sees `follow`, `predict_hold`, and
  `search_safe_viewpoint` states and produces moving search goals/commands.

Record:

- terminal output;
- final monitor PASS line;
- any timeout reason if it fails.

## Phase 3: Repeated Regression

Command:

```bash
bash research/scripts/run_paper_line_ros1_regression.sh 5
```

Pass criteria:

- all 5 runs pass;
- no intermittent startup race;
- no repeated timeout on the same contract, such as `waiting_ego_cmd` or
  `waiting_paper_line_states`.

If a run fails:

- keep the timeout reason;
- rerun once with the same launch;
- do not edit EGO internals to hide the failure.

## Phase 4: Topic-Level Evidence Collection

Minimum topics to record during a passing run:

```bash
/follow/fusion/target_world
/follow/lio/odom
/follow/stage2/state
/follow/stage2/goal
/follow/stage2/debug_goal_path
/move_base_simple/goal
/follow/stage2/ego_position_cmd
/follow/stage2/offboard/setpoint
```

Recording helper:

```bash
bash research/scripts/record_stage2_validation_bag.sh
```

Metrics to derive later:

- goal update rate;
- goal-to-EGO ingress latency;
- EGO command continuity;
- planner command gaps;
- state residence time in `follow`, `predict_hold`, `search_safe_viewpoint`,
  `hold_safe`, and `failsafe`;
- repeated candidate or repeated failure patterns.

Bag analysis command:

```bash
python3 research/scripts/analyze_stage2_rosbag_metrics.py path/to/stage2_run.bag --validate
```

If the runtime uses different topic names, pass a JSON override:

```bash
python3 research/scripts/analyze_stage2_rosbag_metrics.py path/to/stage2_run.bag --topics-json path/to/topics.json
```

Generated outputs:

- `metrics.json`
- `topic_counts.csv`
- `summary.md`

Default bag-validation criteria:

- required topic keys exist: `target`, `odom`, `state`, `goal`, `ego_goal`,
  `ego_cmd`, and `bridge_setpoint`;
- at least 3 Stage2 goals and 3 EGO commands are present;
- goal and command rates are each at least 0.1 Hz;
- max goal and command gaps are each at most 5.0 s;
- mean bridge echo distance is at most 0.75 m;
- state `follow` is observed.

For target-loss/search bags, add:

```bash
python3 research/scripts/analyze_stage2_rosbag_metrics.py path/to/search_run.bag \
  --validate \
  --required-states follow,predict_hold,search_safe_viewpoint
```

## Phase 5: Claim Boundary After Ubuntu20 Validation

Supported after the 2026-06-01 Ubuntu20 run:

- paper-line adapter is executable in the ROS1/EGO software chain;
- hard safety filtering is necessary in diagnostic scenarios;
- planner feedback/cooldown reduces repeated infeasible-goal behavior in
  diagnostic scenarios.
- target-loss/search states `follow`, `predict_hold`, and
  `search_safe_viewpoint` are observed in ROS bag metrics.

Not supported yet:

- real-flight safety;
- full system superiority;
- prediction as an independent novelty claim;
- occlusion scoring, visibility scoring, or FSM recovery as independent
  contributions;
- strict Gazebo + RViz full-chain validation.

## Next Expansion

Do not keep repeating the same matrix as busywork. Add scenario-specific ROS1
fixtures only when they answer a concrete paper or patent claim:

- obstacle-near following;
- planner blocked-goal recovery;
- fixed-behind baseline comparison;
- no-feedback ablation comparison.
