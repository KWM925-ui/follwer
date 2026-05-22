# 2026-05-22 ChatGPT Rebuild Review

Scope: paper-line only.

## Browser Session

Tool: RoxyBrowser + web ChatGPT.

Protected user page not touched:

- `专利撰写协作方案`

New review page:

- https://chatgpt.com/c/6a1071c1-2874-8333-b5be-ebc11b7d4be1

## Prompt Summary

Asked ChatGPT to review the UAV human-following paper-line as a paper reviewer
and engineering reviewer, under these constraints:

- Do not claim EGO, PX4, detector, SLAM, or planner internals as contribution.
- Candidate contribution: planner-agnostic low-latency safety-aware follow-viewpoint decision layer.
- Handle short-horizon prediction uncertainty, occlusion risk, target loss, and planner failure.
- Candidate method: CV-KF, sparse viewpoints, hard safety filtering, weighted score, deterministic FSM.
- Prior art risk: person following, target recovery, trajectory prediction, obstacle avoidance, commercial tracking products are crowded.

Requested:

- reviewer objections
- keep/delete/downgrade algorithm choices
- required baselines and ablations
- safe contribution statement
- high-risk patent/novelty wording

## Key Review Takeaways

- The route is viable only if framed as a decision-layer integration and evaluation contribution, not as a new human-following algorithm.
- CV-KF, candidate points, weighted scoring, and FSM are not individually novel; none should be presented as standalone innovation.
- The strongest positioning is:
  - planner-agnostic interface
  - deterministic low-latency decision layer
  - hard safety filtering before soft scoring
  - uncertainty/occlusion/recovery handled in one measurable loop
  - systematic ablations under failure scenarios
- "Safety" must be phrased as safety-aware, risk-aware, uncertainty-aware, near-miss reduction, or collision-risk reduction. Avoid collision-free or safety guarantee language.
- Search/reacquire candidates should not compete in normal `FOLLOW`; they belong to recovery states.
- Score terms should be reduced in the paper to a small set of core terms:
  - distance
  - view angle / visibility
  - obstacle clearance
  - predicted occlusion risk
  - smoothness / switching
- Deep predictors, RL, and heavy 3D dynamics should be removed or deferred because they distract from the decision-layer contribution.

## Required Baselines

- fixed-behind following
- no-prediction candidate scoring
- CV prediction + fixed follow point
- nearest feasible candidate
- no-recovery / hold-last-position
- fixed safety margin
- optional CA predictor / CA-KF
- optional oracle short-horizon target as an upper bound

## Required Ablations

- without prediction covariance margin
- without speed-dependent margin
- without occlusion risk
- without visibility term
- without switching penalty / hysteresis
- without `SEARCH_SAFE_VIEWPOINT`
- without `PREDICT_HOLD`
- without planner-failure feedback
- candidate-set ablation: behind-only, side-only, full set

## Required Metrics

- target visible ratio
- target loss count and duration
- reacquisition time
- minimum clearance
- near-miss count
- follow distance error
- view-angle error
- planning-failure recovery rate
- viewpoint switching frequency
- trajectory smoothness / acceleration cost
- decision latency

## How The Review Changes The Route

- Compress normal candidate set to `behind`, `left`, `right`, and `far_safe`; keep `search_reacquire` only in search/recovery.
- Keep CV-KF as default but explicitly call it a baseline-grade engineering choice.
- Add fixed-behind and nearest-feasible baselines.
- Add scenario families before claiming FSM or scoring benefits.
- Avoid final paper wording that sounds like broad UAV follow/tracking novelty.

## Follow-Up Route Convergence Review

Same ChatGPT conversation:

- https://chatgpt.com/c/6a1071c1-2874-8333-b5be-ebc11b7d4be1

Follow-up question asked it to decide what the first paper version must keep,
downgrade, delete, or defer across:

- CV-KF / CA-KF
- fixed-behind and nearest-feasible baselines
- dynamic safety margin
- occlusion risk and visibility term
- switch/hysteresis
- `SEARCH_SAFE_VIEWPOINT`
- `PREDICT_HOLD`
- planner-failure feedback
- 2D vs 2.5D
- Simulink
- EGO planner-in-loop

Convergence answer:

- Keep in first method:
  - CV-KF
  - dynamic safety margin
  - occlusion risk
  - visibility term
  - switching / hysteresis
  - `PREDICT_HOLD`
  - `SEARCH_SAFE_VIEWPOINT`
  - simplified planner-failure feedback
- First baselines must include:
  - fixed-behind
  - nearest feasible
  - fixed safety margin
  - no prediction
  - no recovery FSM
- Downgrade:
  - CA-KF as baseline / appendix
  - 2.5D as later supplement
- Defer/delete from first version:
  - Simulink
  - real EGO planner-in-loop
  - LSTM / Transformer / RL
  - complex dynamics

Minimum first-stage MATLAB loop:

- 2D map
- point-mass or speed-limited UAV executor
- target trajectory generator
- static obstacles
- FOV model
- line-of-sight occlusion detection
- planner success/failure simulator
- target visible/invisible logic
- logging and metrics

Minimum first-stage scenarios:

- straight following
- sudden target turn
- obstacle occlusion
- short target loss
- occlusion then reacquisition
- planner-failure injection

Core metrics if forced to keep only six:

- visible ratio
- loss duration
- reacquisition time
- near-miss count
- minimum clearance
- decision latency
