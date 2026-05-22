# Paper-Line Patent Preparation

Status: preparation only. Do not draft final claims yet.
Scope: paper-line only.
Template source reviewed: `C:\Users\wysxgd\Downloads\技术交底书样本3-2.doc`.

## 1. Purpose

The paper-line is expected to produce a patent disclosure after the technical
route and experiment evidence are mature.

This file prepares the structure and evidence requirements now, so later patent
drafting does not start from scattered notes.

## 2. Template Structure Extracted

The provided sample content is unrelated to this project, but its disclosure
structure is useful.

Observed template sections:

1. Patent name
2. Background technology
   - product / use / field
   - problems and shortcomings in existing technology
   - invention purpose
3. Technical solution
   - specific technical solution and component relationships
   - working principle, action flow, operation method
4. Technical points to be protected and beneficial effects
5. Drawings
6. Attachments

Template writing notes:

- Patent name should be concise and reflect subject/type.
- Background should objectively state related existing technology and defects.
- Technical solution should describe components, relationships, process, and
  working principle.
- Protected technical points should be ordered by importance.
- Beneficial effects should follow:
  technical means -> mechanism -> solved problem -> technical effect.
- Drawings should clearly show the invention; flowcharts are acceptable for
  method/software inventions.

## 3. Current Patent Boundary Candidate

Do not claim:

- generic UAV human following
- generic target tracking
- generic short-term prediction
- generic target recovery
- generic obstacle avoidance
- EGO/PX4/SLAM/detector internals

Candidate invention boundary:

> A planner-agnostic upper-layer follow-viewpoint decision method for UAV human
> following, using target short-horizon prediction uncertainty, visibility /
> occlusion risk, dynamic safety filtering, candidate follow-viewpoint scoring,
> and deterministic failure-recovery states to output follow, hold, search,
> reacquire, or failsafe commands without modifying the downstream planner.

This boundary is provisional and must be narrowed after more prior-art search.

## 4. Patent Maturity Gate

Do not start formal patent drafting until these are available:

- final paper-line route frozen enough for implementation
- prior-art search notes covering papers, patents, products, and official docs
- MATLAB prototype with first-stage scenarios
- baseline and ablation results
- clear technical effects:
  - reduced target loss duration
  - faster reacquisition
  - fewer near-miss events
  - improved visibility ratio
  - stable decision latency
  - reduced repeated infeasible planner commands
- concrete flowchart or system diagram
- list of mandatory features and optional embodiments

Current evidence status:

- The dynamic safety margin has initial diagnostic support: fixed safety margin
  produces many near-miss events in the rebuilt stage-one batch.
- Candidate viewpoints now include target-facing heading, which is useful for a
  method disclosure because visibility depends on view direction, not only
  viewpoint position.
- Planner feedback is represented as command-health feedback and has a
  measurable failure-count and failure-burst metric. Stress-suite diagnostics
  support the narrower effect of reducing repeated consecutive infeasible
  commands.
- The full proposed method is not yet clearly better than fixed-behind or
  nearest-feasible baselines, so formal claims remain premature.
- Candidate blacklist/cooldown after planner rejection is now a concrete
  fallback embodiment worth preserving, but it should not be claimed as a broad
  planner-failure solution yet.
- Current MATLAB evidence supports a narrower technical effect: reducing
  near-miss risk compared with fixed safety margins and reducing repeated
  consecutive infeasible commands compared with no planner feedback.
- Current MATLAB evidence does not yet support claiming that prediction,
  occlusion scoring, visibility scoring, or FSM recovery independently improves
  all scenarios.

## 5. Disclosure Skeleton For Later Drafting

### Patent Name

Candidate working names:

- 一种面向无人机人跟随的安全感知跟随视点决策方法
- 一种基于预测不确定性和规划反馈的无人机人跟随视点决策方法
- 一种用于无人机人跟随的目标丢失恢复与安全视点选择方法

Avoid names that are too broad, such as "一种无人机人跟随方法".

### Background Technology

To be filled with source-backed facts:

- UAV/robot human following is known.
- Target tracking and reacquisition are known.
- Human trajectory prediction is known.
- Obstacle-aware and visibility-aware planning are known.
- Existing approaches may not jointly expose a planner-agnostic upper-layer
  decision interface that combines prediction uncertainty, occlusion risk,
  dynamic safety filtering, and planner failure feedback for follow-viewpoint
  selection.

### Technical Problems

Candidate problems:

- fixed follow points can lose visibility near obstacles or turns
- reactive current-target following can be unstable during short target dropout
- downstream planners can fail or reject goals without the follow decision layer
  knowing how to recover
- fixed safety margins do not adapt to speed or target prediction uncertainty
- search behavior can be mixed with normal following and cause unstable choices

### Technical Solution

Core modules:

- target state estimator / CV-KF predictor
- prediction covariance propagation
- sparse follow-viewpoint candidate generator
- dynamic safety margin calculator
- hard safety filter
- occlusion / visibility evaluator
- quality scorer
- deterministic recovery FSM
- planner-feedback interface

Core flow:

1. receive target measurement/confidence and UAV state
2. estimate target state and predict short-horizon target state
3. compute prediction uncertainty
4. update FSM based on visibility, uncertainty, candidate availability, and
   planner health
5. generate normal or recovery-specific candidate viewpoints
6. compute dynamic safety margin
7. filter unsafe or unreachable candidates
8. score remaining candidates
9. output selected viewpoint or hold/search/failsafe command
10. log metrics for recovery and safety-risk evaluation

### Technical Points To Protect

Provisional ordering:

1. The complete planner-agnostic decision flow combining prediction
   uncertainty, dynamic safety filtering, occlusion-aware scoring, and FSM
   recovery.
2. Dynamic safety margin using UAV motion term and target prediction covariance
   term before candidate scoring.
3. Recovery-state-specific candidate generation, where search/reacquire
   candidates are not mixed into normal following.
4. Planner-failure feedback as an input to the upper decision layer, causing
   hold/search/failsafe transitions or candidate rejection.
5. Visibility/occlusion risk and switching/hysteresis terms used after hard
   safety filtering.

### Beneficial Effects To Prove

Each effect needs experiment support before formal drafting:

- improves target visible ratio in obstacle/occlusion scenarios
- reduces target loss duration under short dropouts
- reduces reacquisition time after occlusion
- reduces near-miss count compared with fixed margin
- reduces repeated infeasible goal selection under planner failure
- maintains low decision latency suitable for upper-layer control

### Drawings Needed

Minimum drawings:

- overall system block diagram
- decision flowchart
- FSM transition diagram
- candidate viewpoint geometry diagram
- dynamic safety margin / obstacle inflation diagram
- occlusion line-of-sight scoring diagram
- planner feedback interface diagram
- experiment scenario illustration

## 6. Evidence To Keep Collecting

Record in `research/notes/`:

- prior-art query logs and source links
- patent-source comparison
- product/technical case notes
- MATLAB scenario definitions
- baseline and ablation tables
- generated figures
- failure cases and limitations

## 7. Current Do-Not-Draft Rule

Formal claims, abstract, and final specification are deferred.

Reason:
The route still needs MATLAB first-stage evidence and broader prior-art
comparison. Drafting now would risk overclaiming.
