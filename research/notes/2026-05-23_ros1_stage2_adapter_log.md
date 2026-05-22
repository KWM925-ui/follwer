# 2026-05-23 ROS1 Stage2 Adapter Log

## Context

The user clarified that this repository is a ROS1 / catkin project. The target
runtime for deployment is Ubuntu 20.04 + ROS1. MATLAB is available only for the
research/prototype side and must not be required by the runtime node.

## Decision

Implement the paper-line Stage2 decision layer as a ROS1 external goal provider
inside:

- `src/human_follow_user/scripts/user_stage2_goal_node.py`

This uses the existing Stage2 external slot instead of modifying EGO-Planner,
PX4 bridge code, launch topology, or fusion code.

## Runtime Interface

Inputs:

- `/follow/fusion/target_world` (`human_follow_msgs/Target3D`)
- `/follow/lio/odom` (`nav_msgs/Odometry`)
- `/grid_map/occupancy_inflate` (`sensor_msgs/PointCloud2`, optional safety filter)
- `/follow/stage2/ego_position_cmd` (`quadrotor_msgs/PositionCommand`, optional planner feedback)

Outputs:

- `/follow/stage2/goal` (`geometry_msgs/PoseStamped`)
- `/follow/stage2/debug_goal_path` (`nav_msgs/Path`)
- `/follow/stage2/state` (`human_follow_msgs/FollowState`)

## Implemented Mechanisms

- Constant-velocity Kalman filter for target state and prediction covariance.
- Dynamic safety margin:
  `base + obstacle + speed_gain * vehicle_speed + covariance_gain * covariance_term`.
- Sparse follow candidates:
  `behind`, `left`, `right`, `far_safe`.
- Search candidates:
  `search_reacquire_1 ... search_reacquire_5`.
- Hard safety filtering before scoring.
- Normalized weighted scoring after filtering.
- Deterministic states:
  `idle`, `follow`, `predict_hold`, `search_safe_viewpoint`, `hold_safe`, `failsafe`.
- Optional EGO command feedback with failed-candidate cooldown.

## Verification

Completed:

- `python3 -m py_compile src/human_follow_user/scripts/user_stage2_goal_node.py`

Not completed in this shell:

- ROS master / roslaunch execution.
- Catkin build on Ubuntu 20.04 + ROS1.
- EGO planner-in-loop runtime test.

Reason: current shell is not the target Ubuntu 20.04 + ROS1 runtime. The node is
designed to be transferred and validated there.

## Important Boundary

This adapter is an implementation bridge, not proof of final paper claims. It
should be tested next against the existing Stage2 slot with:

```bash
roslaunch human_follow_bringup stage2_placeholder.launch \
  stage2_enabled:=true \
  goal_provider:=external \
  external_goal_pkg:=human_follow_user \
  external_goal_type:=user_stage2_goal_node.py
```

For real EGO testing, use the existing Stage2/EGO launch path and set:

```bash
goal_provider:=external \
external_goal_pkg:=human_follow_user \
external_goal_type:=user_stage2_goal_node.py
```
