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
- `research/scripts/smoke_ros1_stage2_adapter_core.py`

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

## What To Record Next

For paper and patent evidence, record:

- whether `/follow/stage2/goal` updates at the expected rate;
- whether EGO receives `/move_base_simple/goal`;
- whether EGO produces `/follow/stage2/ego_position_cmd`;
- target-loss behavior: `predict_hold`, `search_safe_viewpoint`, `hold_safe`;
- obstacle-near behavior and any over-conservative cases;
- repeated planner failure bursts with and without planner feedback;
- concrete failure cases where candidate cooldown helps or hurts.

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
