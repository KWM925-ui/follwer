# Paper-Line Ubuntu 20.04 Handoff

Date: 2026-05-23

Target runtime: Ubuntu 20.04 + ROS1 / catkin.

MATLAB is not required on the runtime machine. MATLAB artifacts in this branch
are research prototypes and evidence only.

## Current Deliverables

Research and route:

- `docs/PAPER_LINE_ROUTE_BOOK_CN.md`
- `docs/PAPER_LINE_TECHNICAL_ROUTE.md`
- `docs/PAPER_LINE_PATENT_PREP.md`
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
- `research/scripts/run_paper_line_local_sim.py`
- `research/scripts/smoke_ros1_stage2_adapter_core.py`
- `research/scripts/smoke_ros1_stage2_adapter_scenarios.py`
- `research/scripts/run_paper_line_ros1_regression.sh`
- `src/human_follow_bringup/launch/stage2_paper_line_real_ego_regression.launch`
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

## Checks Already Completed On This Machine

Python syntax:

```bash
python3 -m py_compile src/human_follow_user/scripts/user_stage2_goal_node.py
python3 -m py_compile research/scripts/smoke_ros1_stage2_adapter_core.py
```

Offline core smoke:

```bash
python3 research/scripts/smoke_ros1_stage2_adapter_core.py
```

Observed result:

```text
offline smoke PASS candidate=behind score=0.806 margin=0.846
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
- MATLAB Code Analyzer: clean for `runPaperLineDemo.m`,
  `runPaperLineBatch.m`, `runPaperLineStressBatch.m`,
  `tests/tPaperLineCore.m`, and all current `+paperline/*.m` functions.
- Small execution smoke:
  `runPaperLineBatch(Seeds=1, SaveOutputs=false)` and
  `runPaperLineStressBatch(Seeds=1, SaveOutputs=false)` both returned
  nonempty summaries.

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
- A small no-MATLAB Ubuntu local 2-D simulation was added:
  `python3 research/scripts/run_paper_line_local_sim.py --seeds 1,2,3,4,5`
  - outputs are ignored under `research/outputs/local_sim/`
  - planner feedback/cooldown reduced repeated failed commands in the local
    planner-feedback case;
  - dynamic safety margin reduced, but did not eliminate, near-miss events in
    the local safety-margin case;
  - treat this as a pre-MATLAB screening simulation, not final paper evidence.

## First Steps On Ubuntu 20.04 + ROS1

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

3. Run the external adapter through the existing Stage2 placeholder slot:

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

6. During EGO tests, also inspect:

```bash
rostopic echo /move_base_simple/goal
rostopic echo /follow/stage2/ego_position_cmd
```

7. For the formal paper-line regression entry, prefer:

```bash
roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch
```

8. For repeated evidence, run:

```bash
research/scripts/run_paper_line_ros1_regression.sh 5
```

9. For target-loss full-duration state-sequence evidence, run:

```bash
MONITOR_DURATION_SEC=9.5 LAUNCH_TIMEOUT_SEC=25 \
research/scripts/run_paper_line_ros1_regression.sh 1 \
  fixture_scenario_yaml:=$(pwd)/src/human_follow_bringup/config/paper_line_stage2_target_loss.yaml \
  monitor_full_duration_evidence:=true \
  monitor_required_state_names:=follow,predict_hold,search_safe_viewpoint \
  monitor_required_phase_labels:=loss_target_visible_start,short_dropout_predict_hold,reacquired_after_short_loss,long_dropout_search,reacquired_after_search
```

## What To Record Next

For paper and patent evidence, record:

- whether `/follow/stage2/goal` updates at the expected rate;
- whether EGO receives `/move_base_simple/goal`;
- whether EGO produces `/follow/stage2/ego_position_cmd`;
- target-loss behavior: `predict_hold`, `search_safe_viewpoint`, `hold_safe`;
- obstacle-near behavior and any over-conservative cases;
- repeated planner failure bursts with and without planner feedback;
- concrete failure cases where candidate cooldown helps or hurts.
- per-run `summary.json` with ROS chain results and metric fields.
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

The following remain optional or unproven:

- prediction placement as an independent contribution;
- occlusion scoring as an independent contribution;
- visibility scoring as an independent contribution;
- FSM recovery as an independent contribution.
