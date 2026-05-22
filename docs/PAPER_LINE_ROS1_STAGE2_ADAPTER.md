# Paper-Line ROS1 Stage2 Adapter

This file records the current deployment bridge for paper-line.

## Scope

The project runtime remains ROS1 / catkin. MATLAB is used only for research,
prototype validation, and experiment design. The deployed ROS node does not
depend on MATLAB.

The adapter is implemented at:

- `src/human_follow_user/scripts/user_stage2_goal_node.py`

It uses the existing external Stage2 goal-provider slot and does not modify
EGO-Planner, PX4 bridge code, fusion code, or launch topology.

## Interfaces

Inputs:

- `/follow/fusion/target_world`
- `/follow/lio/odom`
- `/grid_map/occupancy_inflate` when obstacle safety filtering is available
- `/follow/stage2/ego_position_cmd` as optional downstream planner feedback

Outputs:

- `/follow/stage2/goal`
- `/follow/stage2/debug_goal_path`
- `/follow/stage2/state`

## Implemented Logic

- CV-KF target state estimate and short-horizon prediction.
- Dynamic safety margin using vehicle speed and prediction covariance.
- Hard safety filtering before candidate scoring.
- Multi-candidate viewpoint generation:
  `behind`, `left`, `right`, `far_safe`, and `search_reacquire_*`.
- Weighted quality scoring after hard filtering.
- Deterministic states:
  `idle`, `follow`, `predict_hold`, `search_safe_viewpoint`,
  `hold_safe`, `failsafe`.
- Optional planner command feedback and failed-candidate cooldown.

## Launch Entry

For the existing Stage2 placeholder path:

```bash
roslaunch human_follow_bringup stage2_placeholder.launch \
  stage2_enabled:=true \
  goal_provider:=external \
  external_goal_pkg:=human_follow_user \
  external_goal_type:=user_stage2_goal_node.py
```

For the existing Stage2/EGO path, keep the existing launch and override only:

```bash
goal_provider:=external \
external_goal_pkg:=human_follow_user \
external_goal_type:=user_stage2_goal_node.py
```

## Verification Status

Completed in the current workspace:

```bash
python3 -m py_compile src/human_follow_user/scripts/user_stage2_goal_node.py
python3 research/scripts/smoke_ros1_stage2_adapter_core.py
```

Result:

```text
offline smoke PASS candidate=behind score=0.806 margin=0.846
```

Still required on Ubuntu 20.04 + ROS1:

- `catkin_make` or the repo's normal catkin build command.
- Stage2 placeholder launch smoke test.
- Stage2/EGO planner-in-loop test.
- Runtime topic checks for `/follow/stage2/goal`, `/move_base_simple/goal`,
  `/follow/stage2/state`, and EGO command feedback.
