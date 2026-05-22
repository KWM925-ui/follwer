# 2026-05-22 Research Gap Map

Scope: paper-line only.

This note turns the browser/ChatGPT discussion into a concrete search map for
the next research passes. It is not a final literature review.

## How ChatGPT Was Used

Same RoxyBrowser ChatGPT review thread:

- https://chatgpt.com/c/6a1071c1-2874-8333-b5be-ebc11b7d4be1

Prompted for search directions rather than invented citations. The requested
areas were:

- human-following robot / UAV subject following
- target loss recovery / reacquisition
- short-horizon human motion prediction in robotics
- active viewpoint / visibility-aware tracking
- safety margin / uncertainty-aware planning
- planner failure feedback / supervisory FSM

## Search Categories To Continue

### Human-Following Robot / UAV Subject Following

Keywords:

- `UAV human following obstacle avoidance`
- `subject following drone tracking occlusion`
- `person following mobile robot target recovery`
- `follow me UAV target tracking`
- `human following robot safety distance`

What to look for:

- fixed follow distance / behind-target controllers
- subject tracking with camera FOV constraints
- commercial subject-following behavior
- mobile robot person-following recovery logic

Prior-art risk:

- Very high for broad "human following" claims.

Useful for our route:

- Provides baseline definitions and motivates fixed-behind / nearest-feasible
  baselines.

### Target Loss Recovery / Reacquisition

Keywords:

- `target recovery person following robot trajectory prediction`
- `reacquisition target tracking occlusion robot`
- `lost target recovery UAV tracking`
- `search reacquire state machine target tracking`

What to look for:

- short target dropout handling
- search patterns after target loss
- trajectory prediction during occlusion
- validation gates before returning to normal tracking

Prior-art risk:

- High for generic recovery and search behavior.

Useful for our route:

- Justifies treating `PREDICT_HOLD`, `SEARCH_SAFE_VIEWPOINT`, and `REACQUIRE`
  as measured recovery states rather than claiming a generic recovery invention.

### Short-Horizon Human Motion Prediction

Keywords:

- `human motion trajectory prediction survey`
- `short horizon pedestrian prediction Kalman filter`
- `constant velocity baseline human trajectory prediction`
- `robotics human trajectory prediction uncertainty`

What to look for:

- CV / CA baselines
- Kalman / alpha-beta filtering
- learned human trajectory predictors as prior art
- prediction uncertainty propagation

Prior-art risk:

- High if prediction is presented as contribution.

Useful for our route:

- Supports CV-KF as a low-latency engineering predictor and CA-KF as a baseline.

### Active Viewpoint / Visibility-Aware Tracking

Keywords:

- `active viewpoint selection target tracking robot`
- `visibility-aware target tracking UAV`
- `occlusion-aware UAV tracking line of sight`
- `visibility-aware replanning UAV occlusion`
- `active perception UAV target tracking`

What to look for:

- view planning under line-of-sight constraints
- visibility volumes
- occlusion-aware tracking
- active perception and target scanning

Prior-art risk:

- High for generic visibility-aware tracking.

Useful for our route:

- Supports using occlusion risk as a decision-layer score term, but warns
  against claiming broad visibility-aware planning novelty.

### Safety Margin / Uncertainty-Aware Planning

Keywords:

- `uncertainty-aware motion planning covariance safety margin`
- `risk-aware motion planning dynamic safety margin`
- `chance constrained motion planning obstacle uncertainty`
- `adaptive safety margin robot planning uncertainty`
- `Kalman covariance safety margin obstacle avoidance`

What to look for:

- chance constraints
- covariance-inflated obstacles
- risk-aware planning
- dynamic margins based on speed, latency, and prediction uncertainty

Prior-art risk:

- Medium to high for generic covariance-based safety.

Useful for our route:

- Supports the dynamic margin as a reasonable design, but the claim must be
  about its integration in the follow-viewpoint decision layer.

### Planner Failure Feedback / Supervisory FSM

Keywords:

- `robot planner failure recovery supervisory state machine`
- `replanning failure feedback robot supervisor`
- `behavior tree finite state machine robot recovery`
- `planner health feedback robot decision layer`

What to look for:

- supervisory state machines
- behavior trees
- planner health / execution feedback
- recovery from infeasible plans

Prior-art risk:

- High for generic FSM recovery.

Useful for our route:

- Supports planner feedback as an interface signal and ablation target rather
  than a standalone novelty claim.

## Extra Sources Found In This Pass

| Area | Source | Link | Use |
|---|---|---|---|
| Uncertainty-aware planning | Probabilistically safe motion planning to avoid dynamic obstacles with uncertain motion patterns | https://www.researchgate.net/publication/257523145_Probabilistically_safe_motion_planning_to_avoid_dynamic_obstacles_with_uncertain_motion_patterns | Shows uncertainty-aware planning is established; avoid broad claims. |
| Occlusion-aware UAV tracking | Real-Time Occluded Target Detection and Collaborative Tracking Method for UAVs | https://www.mdpi.com/2079-9292/14/20/4034 | Shows UAV target tracking under occlusion is active prior art. |
| Visibility-aware UAV planning | FC-Vision: Real-Time Visibility-Aware Replanning for Occlusion-Free Aerial Target Structure Scanning | https://arxiv.org/abs/2602.13720 | Shows visibility-aware replanning is crowded; keep our route narrower. |
| Risk-aware UAV planning | Survey of Risk-Calibrated Certifiably Safe and Resource-Aware Path Planning for UAVs | https://www.mdpi.com/2504-446X/10/5/351 | Supports risk/certifiable safety as a broader literature; avoid safety guarantee wording. |
| Adaptive margin | Risk-Aware Adaptive Safety Margins for MPC with barrier functions | https://www.mdpi.com/2076-0825/15/2/116 | Supports covariance/uncertainty-driven margins as prior art; use as design context. |
| Visibility-constrained UAV planning | Visibility-Constrained Path Planning for Unmanned Aerial Vehicles | https://ecommons.cornell.edu/items/d2156483-e9c9-4bc6-b201-fb3f4a121bde | Confirms visibility constraints are a known planning class. |

## Immediate Use In This Project

- Add fixed-behind and nearest-feasible baselines before expanding the method.
- Keep first-stage experiments in MATLAB 2D.
- Keep EGO and Simulink out of first-stage validation.
- Add scenario families focused on target loss, occlusion, and planner failure.
- Use "safety-aware" and "risk-aware"; avoid "safe" or "guaranteed".
