# 2026-06-01 Ubuntu20 Gazebo/RViz Validation Log

Scope: paper-line through local `sim_plane` managed simulation.

## Purpose

Validate whether the current paper-line Stage2 real-EGO chain can run in a
local full simulation path beyond ROS-only regression:

- PX4 Gazebo Classic
- MAVROS
- project Stage2 real-EGO chain
- Gazebo GUI
- RViz

This is still simulation evidence. It is not real hardware flight evidence and
does not cover real camera, real SLAM, hardware calibration, or field safety.

## Source And Managed Workspace

Project source:

- `/home/coco/follower_paper_ws`
- branch: `paper-line`

Simulation platform:

- `/home/coco/sim_plane`
- branch: `main`

Managed ROS workspace:

- `/home/coco/sim_plane_ws/workspaces/ros1_human_follow_stage1`

Sync command:

```bash
python3 scripts/sync_human_follow_stage1_workspace.py \
  --source-ws /home/coco/follower_paper_ws
```

Result:

- synced packages all `clean`
- removed `2` pycache entries

Build command:

```bash
./scripts/build_human_follow_stage1_ws.sh
```

Result: PASS.

## Fresh SIH Control Run

Command:

```bash
python3 -m sim_plane run \
  scenarios/px4_sih_quadx_human_follow_stage2_real_ego.json \
  --artifact-root runs --no-hold-open
```

Artifact:

- `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_stage2_real_ego_20260531_175457_993127`

Result:

- `status=passed`

Latest acceptance command:

```bash
python3 -m sim_plane human-follow-stage2-acceptance --latest \
  --artifact-root runs --json
```

Result:

- `status=passed`
- `algorithm_adapter_offboard_mode_reached=true`
- `algorithm_adapter_stage2_real_ego_path_observed=true`
- `algorithm_adapter_stage2_search_goal_observed=true`
- `algorithm_adapter_stage2_nonzero_mavros_setpoint_count=112`

## Gazebo Headless Run

Scenario:

- `/home/coco/sim_plane/scenarios/px4_gazebo_classic_iris_human_follow_stage2_real_ego.json`

Command:

```bash
python3 -m sim_plane run \
  scenarios/px4_gazebo_classic_iris_human_follow_stage2_real_ego.json \
  --artifact-root runs --no-hold-open
```

Artifact:

- `/home/coco/sim_plane/runs/px4_gazebo_classic_iris_human_follow_stage2_real_ego_20260531_180600_060153`

Result:

- `status=passed`
- `backend=px4_gazebo_classic`
- `world=empty`
- `model=iris`
- `headless=true`
- `gazebo_gui=false`
- `ever_armed=true`
- `algorithm_adapter_completed_successfully=true`
- `algorithm_adapter_offboard_mode_reached=true`
- `algorithm_adapter_stage2_real_ego_path_observed=true`
- `algorithm_adapter_stage2_search_goal_observed=true`
- `algorithm_adapter_stage2_nonzero_mavros_setpoint_count=82`
- `algorithm_adapter_stage2_gate_owned_offboard_inferred=true`

Known residual warning:

- `WARN  [commander] Connection to mission computer lost`
- It appeared after ROS/MAVROS shutdown and did not block `status=passed`.
- Do not describe this run as `info-only`.

## Gazebo GUI + RViz Run

Scenario:

- `/home/coco/sim_plane/scenarios/px4_gazebo_classic_iris_human_follow_stage2_real_ego_visual.json`

Command:

```bash
python3 -m sim_plane run \
  scenarios/px4_gazebo_classic_iris_human_follow_stage2_real_ego_visual.json \
  --artifact-root runs --visualize --no-hold-open
```

Artifact:

- `/home/coco/sim_plane/runs/px4_gazebo_classic_iris_human_follow_stage2_real_ego_visual_20260531_180705_889048`

Result:

- `status=passed`
- `backend=px4_gazebo_classic`
- `world=warehouse`
- `model=iris`
- `headless=false`
- `gazebo_gui=true`
- `ever_armed=true`
- `algorithm_adapter_completed_successfully=true`
- `algorithm_adapter_offboard_mode_reached=true`
- `algorithm_adapter_stage2_real_ego_path_observed=true`
- `algorithm_adapter_stage2_search_goal_observed=true`
- `algorithm_adapter_stage2_nonzero_mavros_setpoint_count=106`
- `algorithm_adapter_stage2_gate_owned_offboard_inferred=true`

RViz evidence:

- adapter launch args included `rviz:=true`
- roslaunch command included `rviz:=true`

Visual capture boundary:

- no Gazebo/RViz screenshot, video, or screen recording was captured in this
  artifact;
- the visual evidence in this run is launch/log evidence plus PASS metrics, not
  saved image evidence;
- a future visual-acceptance run should explicitly save screenshots or video if
  paper/patent review requires complete GUI-frame evidence.

Dashboard:

- `http://127.0.0.1:8765` during run

Known residual warning:

- `forcing process kill` for `human_follow_stage2_integrated_chain`
- `WARN  [commander] Connection to mission computer lost`
- These appeared during shutdown cleanup and did not block `status=passed`.
- Do not describe this run as `info-only`.

## Conclusion

Supported now:

- current paper-line source can be synced into the managed sim workspace;
- managed workspace builds on Ubuntu20;
- `PX4 SIH + MAVROS + Stage2 real-EGO` still passes after the current sync;
- `PX4 Gazebo Classic + MAVROS + Stage2 real-EGO` passes headless;
- `PX4 Gazebo Classic + MAVROS + Stage2 real-EGO + Gazebo GUI + RViz`
  passes on this host.

Still not supported:

- real hardware safety;
- real camera detector-in-the-loop validation;
- real SLAM and hardware calibration validation;
- Gazebo Harmonic or future Gazebo versions;
- broad Gazebo scenario matrix or statistical robustness claims.
