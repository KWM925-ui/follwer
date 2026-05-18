# Windows Paper-Line Handoff

## Purpose

This document is for a Windows-side Codex/MATLAB session. The Ubuntu
workspace remains the main hardware/calibration line. The Windows side should
work on the paper/patent/software-copyright line without changing hardware
bringup assumptions.

## Current Project State

- Workspace: `/home/coco/follwer_ws`
- ROS version used on Ubuntu: ROS Noetic style catkin workspace.
- Stage1: human detection/tracking/fusion/controller simulation and hardware
  bringup assets exist.
- Stage2: EGO planner integration exists under `src/human_follow_bringup` and
  `src/ego_planner_vendor`.
- Stage2/EGO simulation branch has strict no-GUI evidence under the current
  simulation contract:
  - real EGO nodes active
  - `/planning/bspline` active and replanning
  - converted path `/follow/stage2/ego_bspline_path` active
  - simulated odom/grid odom aligned
  - command/odom/path did not enter inflated/static obstacle in the strict
    repeated probes
  - scripted replan showcase passed
- This is simulation evidence, not hardware flight proof.

## Do Not Mix With Mainline

- Do not modify calibration or hardware launch files unless the Ubuntu mainline
  explicitly requests it.
- Do not assume Windows MATLAB outputs are flight-ready.
- Treat MATLAB work as algorithm prototyping, metrics, and paper figures first.

## Branch Split

- `main`: Ubuntu mainline and hardware/calibration work.
- `paper-line`: Windows-side research line for MATLAB, prediction, scoring,
  state machine, figures, and paper/proposal drafting.
- Windows Codex should start from `paper-line`, not from a fresh re-derivation
  of the Ubuntu chat context.

## Research Direction

Target direction:

- Short-horizon human motion prediction.
- Safety-aware follow-point generation.
- Follow quality score and quantitative evaluation.
- Failure protection state machine for target loss, planning failure, and
  reacquisition.

Recommended claim shape:

- The contribution is not "we used Point-LIO" or "we used EGO".
- The contribution is a low-latency human-following decision layer that uses
  prediction, quality scoring, safe follow-point selection, and failure
  recovery on top of a fast odom/perception/planning stack.

## MATLAB First Tasks

1. Build a simple 2D/2.5D simulation of:
   - human trajectory
   - UAV trajectory
   - static obstacles
   - perception dropout windows
   - planner failure windows
2. Implement predictors:
   - constant velocity baseline
   - constant acceleration baseline
   - Kalman filter or alpha-beta filter
3. Implement follow-point candidates:
   - behind target
   - left/right offset
   - farther safety point
   - search/reacquire point
4. Implement a follow quality score:
   - distance error
   - viewing angle
   - predicted occlusion risk
   - obstacle clearance
   - control effort / smoothness
   - target visibility confidence
5. Implement a state machine:
   - `FOLLOW`
   - `PREDICT_HOLD`
   - `SEARCH_SAFE_VIEWPOINT`
   - `REACQUIRE`
   - `HOLD_SAFE`
   - `FAILSAFE`
6. Produce plots:
   - target and UAV trajectories
   - selected follow points
   - score curves
   - state transitions
   - prediction error
   - clearance over time

## Suggested Metrics

- Mean and max target-following distance error.
- Mean and max view angle error.
- Minimum obstacle clearance.
- Number of obstacle safety-shell violations.
- Reacquisition time after target loss.
- Planner failure recovery time.
- Trajectory smoothness.
- End-to-end decision latency.
- Percentage of time target remains inside desired camera field of view.

## Paper Skeleton

Working title:

`A Prediction-Scored Safety Follow-Point Generation and Recovery Framework for Low-Latency UAV Human Following`

Core sections:

1. Introduction.
2. Related work:
   - UAV human following
   - short-term human trajectory prediction
   - safe local planning
   - failure recovery state machines
3. System overview.
4. Short-horizon human prediction.
5. Safety-aware follow-point generation and quality score.
6. Failure recovery state machine.
7. Simulation and experimental setup.
8. Results and ablation.
9. Discussion and limitations.
10. Conclusion.

## Patent / Software Copyright Angle

Possible patent idea:

`A safety-aware UAV human-following method based on short-horizon target prediction, multi-candidate follow-point quality scoring, and failure-state recovery.`

Possible software copyright module name:

`Low-Latency UAV Human Following Prediction and Safety Decision System`

## How To Use This Repository On Windows

- Clone the GitHub repository to Windows for code reading and MATLAB modeling.
- Do not try to build ROS Noetic natively on Windows unless explicitly needed.
- Put MATLAB prototypes under a separate folder such as:
  - `research/matlab_follow_prediction/`
- Keep generated figures/data under ignored output folders:
  - `research/outputs/`
  - `research/figures_generated/`
