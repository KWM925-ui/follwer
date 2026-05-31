# Paper-Line Ubuntu 20.04 Handoff

Date: 2026-05-31

Target runtime: Ubuntu 20.04 + ROS1 / catkin.

MATLAB is not required on the runtime machine. MATLAB artifacts in this branch
are research prototypes and evidence only.

## Current Stage Boundary

The Windows/MATLAB algorithm-only refresh has been completed through
`Seeds=1:5` for both stage-one and stress batches. The current MATLAB evidence
is recorded in:

- `research/notes/2026-05-31_windows_matlab_refresh_log.md`

If handing the project to a fresh Codex session on Ubuntu 20.04, paste the
startup prompt from:

- `docs/UBUNTU20_CODEX_START_PROMPT.md`

Next meaningful work should move to Ubuntu 20.04 + ROS1 when the target runtime
is available. Do not reopen the rejected Ubuntu 22.04/ROS2 local temporary
simulation path.

## Current Deliverables

Research and route:

- `docs/PAPER_LINE_ROUTE_BOOK_CN.md`
- `docs/PAPER_LINE_TECHNICAL_ROUTE.md`
- `docs/PAPER_LINE_PATENT_PREP.md`
- `docs/PAPER_LINE_MATLAB_EXPERIMENT_PLAN_CN.md`
- `docs/PAPER_LINE_UBUNTU20_VALIDATION_MATRIX.md`
- `research/notes/`
- `research/prompts/`

MATLAB research prototype:

- `research/matlab/`
- `research/matlab/tests/tPaperLineCore.m`
- `research/matlab/runPaperLineBatch.m`
- `research/matlab/runPaperLineStressBatch.m`

ROS1 deployment adapter:

- `src/human_follow_user/scripts/user_stage2_goal_node.py`
- `docs/PAPER_LINE_ROS1_STAGE2_ADAPTER.md`
- `docs/PAPER_LINE_EXPERIMENT_MATRIX.md`
- `research/scripts/smoke_ros1_stage2_adapter_core.py`
- `research/scripts/smoke_ros1_stage2_adapter_scenarios.py`
- `research/scripts/run_paper_line_ros1_regression.sh`
- `research/scripts/run_stage2_adapter_experiments.py`
- `research/scripts/analyze_stage2_rosbag_metrics.py`
- `research/scripts/record_stage2_validation_bag.sh`
- `src/human_follow_bringup/launch/stage2_paper_line_real_ego_regression.launch`
- `src/human_follow_bringup/launch/stage2_paper_line_search_real_ego_regression.launch`
- `src/human_follow_bringup/scripts/stage2_paper_line_regression_monitor_node.py`
- `src/human_follow_bringup/scripts/stage2_paper_line_search_regression_monitor_node.py`
- `src/human_follow_bringup/config/paper_line_stage2_normal.yaml`
- `src/human_follow_bringup/config/paper_line_stage2_target_loss.yaml`

Ignored generated research outputs:

- `research/outputs/`
- `research/figures_generated/`
- `research/runs/`

## What The ROS1 Adapter Does

The adapter is an external Stage2 goal provider. It sits between target fusion
and EGO-Planner goal input.

Inputs:

- `/follow/fusion/target_world`
- `/follow/lio/odom`
- `/grid_map/occupancy_inflate` when available
- `/follow/stage2/ego_position_cmd` as optional planner command feedback

Outputs:

- `/follow/stage2/goal`
- `/follow/stage2/debug_goal_path`
- `/follow/stage2/state`

Implemented mechanisms:

- CV-KF short-horizon target prediction;
- dynamic safety margin;
- hard safety filtering before scoring;
- multi-candidate follow-viewpoint selection;
- deterministic follow/loss/hold/search/failsafe states;
- optional failed-candidate cooldown from planner command feedback.

## Latest Ubuntu20 Validation

Fresh Ubuntu20 validation was completed on 2026-06-01. Detailed log:

- `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`

Runtime:

- Ubuntu 20.04.6 LTS
- Python 3.8.10
- ROS Noetic
- commit `bc684c537198cd3ea77ea877bd9bb590e190476d`

Phase 0 and Phase 1:

```bash
SKIP_ROS=1 bash research/scripts/run_paper_line_ubuntu20_validation.sh
```

Result: PASS.

Completed:

- Python syntax checks;
- offline decision-core smoke;
- offline scenario smoke;
- offline baseline/ablation diagnostics;
- `catkin_make`.

Phase 2 normal ROS1/EGO regression:

```bash
RUN_COUNT=1 LAUNCH_TIMEOUT_SEC=35 MONITOR_DURATION_SEC=18.0 \
bash research/scripts/run_paper_line_ros1_regression.sh 1
```

Result: PASS.

Artifact:

- `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002359`

Phase 2 target-loss/search ROS1/EGO regression:

```bash
MONITOR_DURATION_SEC=9.5 LAUNCH_TIMEOUT_SEC=35 \
bash research/scripts/run_paper_line_ros1_regression.sh 1 \
  fixture_scenario_yaml:=$(pwd)/src/human_follow_bringup/config/paper_line_stage2_target_loss.yaml \
  monitor_full_duration_evidence:=true \
  monitor_required_state_names:=follow,predict_hold,search_safe_viewpoint \
  monitor_required_phase_labels:=loss_target_visible_start,short_dropout_predict_hold,reacquired_after_short_loss,long_dropout_search,reacquired_after_search
```

Result: PASS.

Artifact:

- `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002421`

Phase 3 repeated regression:

```bash
bash research/scripts/run_paper_line_ros1_regression.sh 5
```

Result: 5/5 PASS.

Artifact:

- `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002516`

Phase 4 bag metrics:

- normal bag metrics PASS:
  `research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation_metrics`
- target-loss/search bag metrics PASS:
  `research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation_metrics`

Key target-loss/search bag evidence:

- `goal_count=127`
- `ego_cmd_count=873`
- observed states: `follow`, `predict_hold`, `search_safe_viewpoint`
- state durations:
  `follow=5.714 sec`, `predict_hold=1.887 sec`,
  `search_safe_viewpoint=1.081 sec`

Boundary:

- This proves paper-line ROS1/EGO software-chain executability.
- It does not prove real-flight safety or strict Gazebo/RViz full-chain
  validation.

## Earlier Checks Completed On This Branch

Python syntax:

```bash
python3 -m py_compile src/human_follow_user/scripts/user_stage2_goal_node.py
python3 -m py_compile research/scripts/smoke_ros1_stage2_adapter_core.py
python3 -m py_compile research/scripts/smoke_ros1_stage2_adapter_scenarios.py
python3 -m py_compile research/scripts/run_stage2_adapter_experiments.py
python3 -m py_compile research/scripts/analyze_stage2_rosbag_metrics.py
python3 -m py_compile src/human_follow_bringup/scripts/stage2_paper_line_regression_monitor_node.py
python3 -m py_compile src/human_follow_bringup/scripts/stage2_paper_line_search_regression_monitor_node.py
```

Offline core and scenario smoke:

```bash
python3 research/scripts/smoke_ros1_stage2_adapter_core.py
python3 research/scripts/smoke_ros1_stage2_adapter_scenarios.py
python3 research/scripts/run_stage2_adapter_experiments.py --seeds 1:5
```

Observed result:

```text
offline smoke PASS candidate=behind score=0.806 margin=0.846
offline scenario smoke PASS open_follow:behind:0.806:1.126 behind_blocked:left:0.755:1.126 switching_bias:left:0.810:1.126
stage2 adapter experiments PASS runs=175 ... best=fixed_behind success=1.000
```

Offline scenario smoke:

```bash
python3 research/scripts/smoke_ros1_stage2_adapter_scenarios.py
```

Observed result:

```text
scenario smoke PASS normal=behind short_loss=predict_hold long_loss=search_safe_viewpoint expired_loss=hold_safe hard_filter=behind_rejected cooldown=blocked_then_expired
```

Formal paper-line real-EGO regression:

```bash
roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch
```

Repeated runner:

```bash
research/scripts/run_paper_line_ros1_regression.sh 5
```

Latest repeated evidence on 2026-05-26:

- Artifact:
  `.codex/artifacts/paper_line_ros1_adapter_20260526/formal_real_ego_regression_234357`
- Result: 5/5 PASS.
- Each run: `distinct_goals=3`, `distinct_cmds=3`, `request_count=1`.
- Each run sampled `/planning/bspline` and `/follow/stage2/ego_position_cmd`.
- `/follow/stage2/offboard/setpoint` and `/mavros/setpoint_raw/local` were
  confirmed by the paper-line monitor contract and `offboard_mode_gate` logs.
- Fresh formal runs counted `final_plan_success=0` as `0` and
  `final_plan_success=1` as `22, 22, 22, 22, 23`.
- No `ERROR`, `FATAL`, `Traceback`, or `in obstacle` lines were counted by the
  runner.
- Process cleanup was clean in all runs.

Build caveat on this host:

- Plain `catkin_make` hit a CMake 4.2 / `/usr/src/googletest` compatibility
  error before package configuration.
- `catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5` passed.

MATLAB tests and static checks should be refreshed before handoff when MATLAB is
available. The runtime machine does not need MATLAB.

Latest Windows/WSL-side MATLAB refresh:

- `tPaperLineCore`: 11 passed, 0 failed, 0 incomplete.
- `runPaperLineDemo(ShowFigures=false)`: completed.
- `runPaperLineBatch(Seeds=1, SaveOutputs=false)`: completed.
- `runPaperLineStressBatch(Seeds=1, SaveOutputs=false)`: completed.
- `runPaperLineBatch(Seeds=1:3, SaveOutputs=true)`: completed.
- `runPaperLineStressBatch(Seeds=1:3, SaveOutputs=true)`: completed.
- `runPaperLineBatch(Seeds=1:5, SaveOutputs=true)`: completed, 300 runs.
- `runPaperLineStressBatch(Seeds=1:5, SaveOutputs=true)`: completed, 225 runs.
- `summarizePaperLineResults`: completed.
- Final interpretation: dynamic safety margin and planner feedback/cooldown are
  supported; prediction, occlusion score, visibility score, FSM recovery, and
  global full-method superiority remain unsupported as independent claims.

Before moving to Ubuntu 20.04 + ROS1, read
`docs/PAPER_LINE_UBUNTU20_VALIDATION_MATRIX.md`. That file is now the ordered
acceptance checklist for runtime validation.

Latest Ubuntu-side scenario widening on 2026-05-27:

- `stage2_goal_input_fixture_node.py` can load scenario phases from YAML via
  `fixture_scenario_yaml`.
- Default formal regression now uses
  `src/human_follow_bringup/config/paper_line_stage2_normal.yaml`, preserving
  the previous three-phase chain behavior.
- The paper-line monitor supports a full-duration evidence mode with required
  state and phase checks.
- Target-loss full-duration evidence passed once:
  `.codex/artifacts/paper_line_ros1_adapter_20260527/target_loss_full_duration_010748`
  - observed states: `follow`, `predict_hold`, `search_safe_viewpoint`
  - observed candidates: `behind`, `left`, `right`, `search_reacquire_1`,
    `search_reacquire_5`
  - all five target-loss phase labels were observed
  - `distinct_goals=32`, `distinct_cmds=23`, `request_count=1`
  - many `final_plan_success=0` lines appeared during the harder
    target-loss/search window, so this is state-sequence evidence, not robust
    planner recovery proof.

## If Re-running On Ubuntu 20.04 + ROS1

Use `docs/PAPER_LINE_UBUNTU20_VALIDATION_MATRIX.md` as the ordered acceptance
checklist. The commands below are the quick handoff version.

If GitHub push is still blocked by credentials, export the local paper-line
commits from the Windows/WSL machine and apply them on Ubuntu20:

```bash
git format-patch origin/paper-line..paper-line -o research/runs/paper_line_patch_queue
```

Copy those patch files to Ubuntu20, then run:

```bash
git checkout paper-line
git pull --ff-only origin paper-line
git am path/to/patches/*.patch
```

1. Pull the branch:

```bash
git fetch origin
git checkout paper-line
git pull --ff-only origin paper-line
```

2. Build with the repository's normal catkin flow. If the workspace is the repo
root, this is typically:

```bash
catkin_make
source devel/setup.bash
```

3. Run the paper-line adapter through the existing Stage2 placeholder slot:

```bash
roslaunch human_follow_bringup stage2_placeholder.launch \
  stage2_enabled:=true \
  goal_provider:=external \
  external_goal_pkg:=human_follow_user \
  external_goal_type:=user_stage2_goal_node.py
```

4. Check topics:

```bash
rostopic echo /follow/stage2/state
rostopic echo /follow/stage2/goal
rostopic echo /follow/stage2/debug_goal_path
```

5. Then test through the existing Stage2/EGO launch path with the same external
goal provider override:

```bash
goal_provider:=external \
external_goal_pkg:=human_follow_user \
external_goal_type:=user_stage2_goal_node.py
```

6. For the one-command paper-line adapter + real-EGO regression harness, run:

```bash
roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch
```

For target-loss/search behavior:

```bash
roslaunch human_follow_bringup stage2_paper_line_search_real_ego_regression.launch
```

7. For repeated paper-line regression runs, run:

```bash
bash research/scripts/run_paper_line_ros1_regression.sh 5
```

8. For software-only paper-line baseline/ablation diagnostics without MATLAB:

```bash
python3 research/scripts/run_stage2_adapter_experiments.py --seeds 1:5
```

9. During EGO tests, also inspect:

```bash
rostopic echo /move_base_simple/goal
rostopic echo /follow/stage2/ego_position_cmd
```

## Can This Version Run Directly On Ubuntu 20.04?

Yes, after the normal ROS1 workspace setup and `catkin_make`, this branch now
has a direct paper-line regression entry:

```bash
roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch
```

That entry launches the paper-line adapter itself, not the baseline Stage2 goal
generator. The monitor requires `/follow/stage2/state` from the paper-line node
and checks that the generated Stage2 goal is carried through EGO goal ingress,
EGO command output, the PX4 bridge output, and the fake MAVROS setpoint gate.

Boundary: this is a ROS1/EGO regression harness, not real hardware validation.
It proves that the algorithm can be exercised through the intended software
chain. It still needs to be run on Ubuntu 20.04 + ROS1 because this WSL2
Ubuntu 22.04 shell is not the target ROS1 runtime.

For repeated evidence, run:

```bash
research/scripts/run_paper_line_ros1_regression.sh 5
```

For target-loss full-duration state-sequence evidence, run:

```bash
MONITOR_DURATION_SEC=9.5 LAUNCH_TIMEOUT_SEC=25 \
research/scripts/run_paper_line_ros1_regression.sh 1 \
  fixture_scenario_yaml:=$(pwd)/src/human_follow_bringup/config/paper_line_stage2_target_loss.yaml \
  monitor_full_duration_evidence:=true \
  monitor_required_state_names:=follow,predict_hold,search_safe_viewpoint \
  monitor_required_phase_labels:=loss_target_visible_start,short_dropout_predict_hold,reacquired_after_short_loss,long_dropout_search,reacquired_after_search
```

## What To Do Next

Do not keep rerunning the same validation loop unless code or environment
changes. The useful next work is paper/patent evidence packaging and targeted
scenario widening.

For paper and patent evidence, prepare:

- a compact table from `research/notes/2026-05-31_windows_matlab_refresh_log.md`;
- a compact table from `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`;
- a figure or table showing target-loss/search state residence time;
- a figure or table showing planner failure burst reduction;
- obstacle-near behavior and any over-conservative cases;
- concrete failure cases where candidate cooldown helps or hurts;
- baseline/ablation comparisons:
  original Stage2 baseline, paper-line without prediction, paper-line without
  dynamic margin, paper-line without hard filter, and full paper-line adapter.

## Current Claim Boundary

Do not claim full system superiority yet.

The strongest current claims are:

- dynamic safety margin is worth keeping;
- hard safety filtering before scoring is necessary;
- planner feedback plus failed-candidate cooldown reduces repeated infeasible
  command bursts in diagnostic stress tests.

Latest Python offline experiment status:

- `no_hard_filter` created safety-shell violations in the diagnostic suite.
- `no_planner_feedback` created repeated planner-failure bursts in the blocked
  planner scenario.
- `paper_line_full` is not yet globally better than all baselines, so paper
  claims should focus on safety/recovery mechanisms unless stronger ROS1/EGO
  data proves broader benefit.

The following remain optional or unproven:

- prediction placement as an independent contribution;
- occlusion scoring as an independent contribution;
- visibility scoring as an independent contribution;
- FSM recovery as an independent contribution.
