# Paper-Line Technical Route

Status: rebuilt route v1.0.
Scope: paper-line only. Do not modify ubuntu-mainline.
Rebuilt: 2026-05-22.

## 1. Target

Build a research/paper line for UAV human following that is independent from
the deployment line.

The paper-line contribution is:

> A planner-agnostic, low-latency, safety-aware follow-viewpoint decision layer
> for UAV human following under short-horizon prediction uncertainty, occlusion
> risk, target loss, and planner failure.

This is not a claim about:

- human detection or tracking networks
- camera/lidar fusion
- SLAM or odometry
- EGO-Planner internals
- PX4 control or obstacle avoidance
- generic person following
- generic target recovery
- learned long-horizon human trajectory prediction

Reason:
The rebuilt literature, product, and patent scan shows these broad areas are
already crowded. The defensible route is the specific decision-layer
integration and its measurable behavior under failure cases.

Evidence notes:

- `research/notes/2026-05-22_rebuilt_research_dossier.md`
- `research/notes/2026-05-22_chatgpt_rebuild_review.md`
- `research/notes/2026-05-22_research_gap_map.md`
- `docs/PAPER_LINE_PATENT_PREP.md`
- `research/notes/2026-05-22_stage1_matlab_batch_log.md`

## 2. System Boundary

The decision layer has a narrow interface.

Inputs:

- target position / velocity estimate
- target prediction covariance
- target visibility or measurement confidence
- UAV state
- obstacle primitives or occupancy-derived clearance
- downstream planner/executor health
- previous selected viewpoint and FSM state

Outputs:

- selected follow viewpoint
- hold command
- search/reacquire command
- failsafe request

The layer does not modify detector, tracker, SLAM, EGO, PX4, or planner
internals.

Reason:
This makes the contribution testable and supports the planner-agnostic claim.

Risk:
Planner-agnostic wording will be weak unless experiments include at least one
executor/planner swap or simplified executor comparison.

## 3. Architecture

```text
target measurement + confidence
uav state
obstacle / clearance model
planner health feedback
        -> target estimator and short-horizon predictor
        -> recovery supervisor FSM
        -> sparse follow-viewpoint generator
        -> hard safety filter
        -> visibility/quality scorer
        -> selected viewpoint / hold / search / failsafe
        -> downstream executor or planner
```

## 4. Prediction

Default:

- CV-KF short-horizon predictor

Baselines:

- current target / no prediction
- CV extrapolation
- CA extrapolation
- CA-KF
- optional oracle short-horizon target upper bound

Decision reason:

- CV-KF is low-latency, explainable, and provides covariance for dynamic safety
  margins.
- MATLAB supports standard Kalman tracking workflows.
- Deep predictors and RL would shift the paper toward data-hungry prediction
  research, which is not the intended contribution.

Current evidence boundary:

- Prediction covariance remains useful for dynamic safety margins.
- Predicted-position viewpoint placement is not yet proven beneficial. In the
  current MATLAB 2D prototype, no-prediction baselines are competitive or better
  on visibility/loss metrics. Do not make prediction performance the main paper
  claim.

Rejected/deferred:

- LSTM / Transformer prediction
- RL viewpoint policies
- social-force model as a central contribution

Risk:
CV assumptions fail on sudden turns. The route must include sharp-turn and
velocity-change scenarios.

## 5. Follow-Viewpoint Candidates

Generate sparse candidates in the predicted target motion frame.

Normal following candidates:

- `behind`
- `left`
- `right`
- `far_safe`

Recovery-only candidate:

- `search_reacquire`

Important rule:

`search_reacquire` is generated only in recovery/search states, not in normal
`FOLLOW`.

Decision reason:

- Sparse candidates are interpretable, low-latency, and easy to ablate.
- The previous `side_left` / `side_right` naming is redundant with left/right
  and should not be emphasized in the paper.
- Search behavior should not compete with normal following behavior.

Rejected/deferred:

- dense viewpoint sampling as the main method
- learned viewpoint policy

Risk:
Sparse candidates can miss valid viewpoints in cluttered environments. This
must be tested with candidate-set ablations.

## 6. Hard Safety Filter

Safety filtering happens before scoring.

Candidate rejection checks:

- candidate point inside inflated obstacle
- segment from current UAV position to candidate intersects inflated obstacle
- candidate unreachable under simple speed/acceleration limits
- optional workspace or altitude envelope violation

Dynamic margin:

```text
margin = body_radius
       + obstacle_margin
       + k_v * speed_term
       + k_p * sqrt(lambda_max(P_pred_position))
```

Decision reason:

- Unsafe candidates should not survive by receiving a high soft score.
- Speed and covariance terms are more defensible than a fixed clearance rule.

Rejected/deferred:

- score-only safety
- formal collision-free guarantee

Risk:
This is safety-aware, not safety-guaranteed. Do not claim collision-free
operation without formal proof.

## 7. Quality Score

Score only candidates that pass hard safety filtering.

Paper-facing score terms should be compact:

- follow distance error
- view angle / visibility
- obstacle clearance
- predicted occlusion risk
- smoothness / switching cost

Decision reason:

- Too many score terms look hand-tuned.
- A compact score is easier to explain, sweep, and ablate.

Risk:
Weights can look arbitrary. Use one fixed default weight set across scenarios
and add sensitivity analysis.

## 8. Occlusion

First implementation:

- 2D line-of-sight segment from candidate viewpoint to predicted target
- obstacle intersection means high occlusion risk
- near-intersection creates soft risk

Decision reason:

- Concrete and visual.
- Cheap enough for low-latency decision loops.
- Adequate for first MATLAB evidence.

Rejected/deferred:

- learned visibility prediction
- full 3D raycasting as the first route

Risk:
2D can underrepresent altitude effects. Call this an upper-layer visibility-risk
proxy.

## 9. Recovery FSM

States:

- `FOLLOW`
- `PREDICT_HOLD`
- `SEARCH_SAFE_VIEWPOINT`
- `REACQUIRE`
- `HOLD_SAFE`
- `FAILSAFE`

Decision reason:

- The FSM separates short target dropouts, long target loss, planner failure,
  reacquisition validation, and unsafe candidate availability.
- Deterministic transitions make the behavior auditable and testable.

Rejected/deferred:

- learned recovery policy
- open-ended search without timeout

Risk:
FSMs are common. The paper can only claim value if ablations show reduced loss
duration, reacquisition time, near-miss count, or state oscillation.

## 10. Baselines

Required:

- fixed-behind following
- no-prediction candidate scoring
- CV prediction + fixed follow point
- nearest feasible candidate
- no-recovery / hold-last-position
- fixed safety margin
- CV-KF proposed route
- CA-KF and CA extrapolation as predictor baselines

Optional:

- oracle short-horizon target upper bound
- simplified executor/planner swap for planner-agnostic evidence

## 11. Ablations

Required:

- without prediction covariance margin
- without speed-dependent margin
- without occlusion risk
- without visibility term
- without switching penalty / hysteresis
- without `SEARCH_SAFE_VIEWPOINT`
- without `PREDICT_HOLD`
- without planner-failure feedback
- candidate set: behind-only / side-only / full set
- score weight sensitivity

## 12. Metrics

Primary:

- target visible ratio
- target loss count and duration
- reacquisition time
- minimum clearance
- near-miss count
- follow distance error
- view-angle error
- planner-failure recovery rate
- viewpoint switching frequency
- decision latency

Secondary:

- trajectory smoothness / acceleration cost
- FSM transition count
- fraction of time in each FSM state
- prediction RMSE at selected horizons

## 13. Stage-One Scenario Families

Do not evaluate only normal following.

First-stage minimum:

- straight following
- sudden target turn
- obstacle occlusion
- short target loss
- occlusion followed by reacquisition
- planner-failure injection

Second-stage expansion:

- target speed change
- narrow corridor
- longer target disappearance
- partially unreachable candidate set
- high prediction uncertainty

## 14. MATLAB Route

Keep MATLAB first.

Required modules:

- scenario generator
- noisy measurement / dropout model
- predictor baselines
- candidate generator
- hard safety filter
- occlusion-risk model
- scorer
- FSM
- simple executor / planner stub
- metrics
- figures
- tests

Implementation rule:

Existing `research/matlab/` code is a reusable prototype, not final evidence
until rerun under this rebuilt route.

Current status:

- Stage-one scenario/baseline matrix runs end-to-end in MATLAB.
- Tests pass.
- The first rebuilt batch was a plumbing check, not proof of contribution.
- A geometry-aware visibility pass has been added.
- The overly harsh visibility model has been replaced with a camera-heading/FOV
  visibility model plus standalone visibility unit tests.
- Candidate viewpoints now include both position and target-facing heading.
- Task success is decomposed into tracking, recovery, and safety success.
- Planner failure is represented by a command-health signal and failure-count
  metric.
- Current results are still diagnostic rather than publishable: the dynamic
  safety margin is supported by the fixed-margin ablation, but the full proposed
  method is not yet dominant over fixed-behind or nearest-feasible baselines.
- Next MATLAB pass should strengthen planner-failure recovery and add harder
  scenarios where fixed-behind is safe but occluded, nearest-feasible is safe
  but poor for visibility, and recovery states have measurable benefit.
- A second stress-suite batch has been added. It supports keeping planner
  command feedback and failed-candidate cooldown because repeated consecutive
  planner failures are reduced, but it still does not prove the FSM or occlusion
  score as independent contributions.
- Closure pass conclusion: this MATLAB 2D prototype is now a diagnostic
  baseline, not a paper-result endpoint. It supports dynamic safety margin and
  planner feedback; prediction placement, occlusion score, visibility score,
  and FSM recovery are optional mechanisms requiring stronger future evidence.

First-stage exclusions:

- no Simulink
- no real EGO planner-in-loop
- no PX4 loop
- no complex UAV dynamics
- no deep prediction model

Reason:
The first paper version must prove the decision layer in a controlled MATLAB
2D setting before adding executor/planner complexity.

## 15. Paper Wording

Use:

- safety-aware
- risk-aware
- uncertainty-aware
- collision-risk reduction
- near-miss reduction
- planner-agnostic decision layer
- follow-viewpoint selection

Avoid:

- safe UAV following
- collision-free guarantee
- new human-following algorithm
- new target recovery method
- new trajectory prediction method
- robust in complex environments
- end-to-end autonomous UAV following system

Prior-art areas to keep checking:

- human-following robots and subject-following UAV products
- target loss recovery and reacquisition
- short-horizon pedestrian / human trajectory prediction
- active viewpoint and visibility-aware tracking
- uncertainty-aware and chance-constrained planning
- planner failure recovery supervisors and behavior trees

## 16. Implementation Order

1. Keep the rebuilt route and evidence notes as the route authority.
2. Adjust MATLAB candidate naming and metrics to match this route.
3. Add fixed-behind and nearest-feasible baselines.
4. Add first-stage scenario families:
   straight following, sudden turn, obstacle occlusion, short target loss,
   occlusion-reacquisition, and planner-failure injection.
5. Add ablations for covariance margin, speed margin, occlusion, visibility,
   recovery states, planner feedback, and candidate sets.
6. Rerun MATLAB tests and batch experiments.
7. Generate figures from the rebuilt experiments only.
8. Add a second stress-suite pass focused on planner-command rejection,
   fixed-behind occlusion, nearest-feasible visibility failure, and recovery
   state benefit.
9. Extend the recovery/search candidate set and add targeted ablations where
   no-recovery and no-occlusion variants fail for clear, visual reasons.
10. Only after MATLAB 2D evidence is stable, consider 2.5D, EGO planner-in-loop,
   or Simulink.
11. Draft paper outline from measured evidence, not from desired claims.

## 17. Bottom Line

Proceed, but keep the claim narrow.

The route is strongest as:

> A measurable safety-aware decision layer for follow-viewpoint selection under
> prediction uncertainty, occlusion, target loss, and planner feedback.

It is weakest if written as a broad UAV human-following, target-recovery, or
obstacle-avoidance invention.

## 18. Patent Preparation Boundary

The paper-line is also expected to mature into a patent disclosure.

Patent work is deferred until the user says the timing is mature, but the route
must preserve patent-ready evidence.

Current patent-prep boundary:

> an upper-layer planner-agnostic UAV human-following viewpoint decision method
> that combines target prediction uncertainty, dynamic safety filtering,
> occlusion/visibility scoring, planner feedback, and deterministic recovery
> states.

Current supported core should be narrowed to:

- dynamic safety filtering using speed and prediction covariance;
- planner command-health feedback;
- failed-candidate cooldown or equivalent repeated-failure suppression.

Occlusion/visibility scoring and recovery FSM should remain optional dependent
features unless later experiments prove their independent value.

Do not prepare broad patent claims around:

- UAV human following in general
- target tracking in general
- Kalman prediction in general
- target reacquisition in general
- obstacle avoidance in general

Patent maturity requires:

- prior-art notes
- MATLAB scenario evidence
- baseline and ablation results
- technical effects tied to metrics
- diagrams and flowcharts
- mandatory vs optional feature separation
- fallback embodiments
