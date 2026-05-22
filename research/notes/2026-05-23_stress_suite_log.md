# 2026-05-23 Stress Suite Log

Scope: paper-line only.

## Purpose

The previous stage-one batch was useful but too easy for fixed-behind and
nearest-feasible baselines. This pass adds a second stress suite to test the
route's weak points:

- fixed-behind occlusion;
- nearest-feasible visibility failure;
- recovery after corner loss;
- planner command rejection;
- fixed safety margin under tighter obstacle pressure.

## External Research Context

Searches performed:

- `active target tracking benchmark visibility aware UAV occlusion 2026 arxiv`
- `visibility-aware active target tracking UAV occlusion FOV planning`
- `planner failure recovery finite state machine robot navigation replanning feedback`
- `person following robot target recovery trajectory prediction occlusion`

Relevant source categories:

- active/visibility-aware target tracking benchmark work;
- person-following target recovery work;
- planner failure and robot recovery supervisors.

Route impact:

- These searches reinforce that visibility-aware tracking, target recovery, and
  planner recovery are crowded. The paper-line should not claim these as
  standalone novelties.
- The defensible experiment target remains the integrated upper-layer decision
  loop and its measurable behavior under stress.

Roxy status:

- Roxy OpenAPI still sees the existing browser profile and CDP endpoint.
- Roxy Playwright transport was closed during this pass, so no new ChatGPT
  second-model message was sent in this round.
- The previous ChatGPT review remains recorded in
  `research/notes/2026-05-23_external_research_and_chatgpt_review.md`.

## Implemented

New stress scenarios in `paperline.makeScenario`:

- `fixed_behind_occlusion`
- `nearest_visibility_trap`
- `recovery_corner_loss`
- `planner_feedback_stress`
- `planner_blocked_goal`

New batch runner:

- `research/matlab/runPaperLineStressBatch.m`

New or extended mechanics:

- `enableCandidateBlacklist`
- `failedCandidateCooldownFrames`
- `plannerFailureHoldFrames`
- failed candidate cooldown after planner command rejection
- planner failure burst metric
- blacklist activation count metric

The blacklist is intentionally conservative:

- It does not change downstream planner internals.
- It only prevents immediately retrying a candidate name that was just rejected
  by the abstract planner contract.

## Verification

- `check_matlab_code` clean for changed MATLAB files.
- `run_matlab_test_file` on `tests/tPaperLineCore.m`: 11 passed, 0 failed.

Stress batch command:

```matlab
summary = runPaperLineStressBatch(Seeds=1:3, SaveOutputs=true);
```

Generated outputs:

- `research/outputs/paper_line_stress/stress_runs.csv`
- `research/outputs/paper_line_stress/stress_by_condition.csv`
- `research/outputs/paper_line_stress/stress_by_scenario_condition.csv`
- `research/figures_generated/paper_line_stress/stress_condition_summary.png`

Runs:

- 5 stress scenarios
- 8 conditions
- 3 seeds
- 120 total runs

## Condition-Level Results

| condition | visible ratio | loss duration | near-miss count | planner failures | max failure burst | blacklist activations | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| fixed_behind | 0.8108 | 4.5600 | 0.0000 | 7.8000 | 0.4000 | 19.2667 | 0.6000 |
| fixed_safety_margin | 0.9663 | 0.8133 | 120.1333 | 7.8000 | 0.4000 | 20.4000 | 0.0000 |
| nearest_feasible | 0.8484 | 3.6533 | 0.0000 | 6.2667 | 0.4000 | 14.5333 | 0.6000 |
| no_occlusion_score | 0.8476 | 3.6733 | 0.0000 | 7.8000 | 0.4000 | 17.6667 | 0.6000 |
| no_planner_feedback | 0.8495 | 3.6267 | 0.0000 | 15.4000 | 15.4000 | 0.0000 | 0.6000 |
| no_prediction | 0.9234 | 1.8467 | 0.0000 | 6.5333 | 0.4000 | 12.5333 | 0.6000 |
| no_recovery_fsm | 0.8476 | 3.6733 | 0.0000 | 7.8000 | 0.4000 | 17.6667 | 0.6000 |
| proposed | 0.8467 | 3.6933 | 0.0000 | 7.8000 | 0.4000 | 17.6667 | 0.6000 |

## Proposed Scenario-Level Results

| scenario | visible ratio | loss duration | near-miss count | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|
| fixed_behind_occlusion | 0.7137 | 6.9000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 |
| nearest_visibility_trap | 0.8188 | 4.3667 | 0.0000 | 0.0000 | 0.0000 | 1.0000 |
| planner_blocked_goal | 1.0000 | 0.0000 | 0.0000 | 8.0000 | 1.0000 | 0.0000 |
| planner_feedback_stress | 0.8382 | 3.9000 | 0.0000 | 31.0000 | 1.0000 | 0.0000 |
| recovery_corner_loss | 0.8631 | 3.3000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 |

## Interpretation

Useful evidence:

- Fixed safety margin is clearly unsafe in this stress suite:
  near-miss count is high and task success is 0.
- Planner feedback now has a measurable effect:
  `no_planner_feedback` has max failure burst 15.4, while proposed is 0.4 at
  the condition level. This supports keeping planner feedback and failed
  candidate cooldown as part of the decision loop.
- The blacklist/cooldown mechanism reduces repeated consecutive planner
  failures without modifying downstream planner internals.

Still weak:

- Proposed does not outperform `no_prediction` on visibility or loss duration.
- Proposed, `no_occlusion_score`, and `no_recovery_fsm` remain nearly identical
  in aggregate. This means the current stress suite still does not isolate the
  occlusion score and FSM recovery value.
- Planner failure total count remains high in planner-specific scenarios, even
  though failure bursts are shortened. This supports the narrower claim
  "reduces repeated consecutive infeasible commands", not "solves planner
  failure".

## Route Decisions

Keep:

- dynamic safety margin;
- planner command health feedback;
- failed-candidate cooldown / blacklist;
- decomposed planner failure metrics, especially burst length;
- stress suite as a separate diagnostic batch from stage one.

Do not claim yet:

- full proposed method dominates all baselines;
- prediction improves all scenarios;
- FSM recovery is independently proven;
- occlusion score is independently proven;
- planner failure is fully recovered.

Next MATLAB target:

- Add candidate-level failure memory by candidate geometry, not only candidate
  name.
- Add a recovery-specific search candidate set with multiple angular options
  rather than a single `search_reacquire` point.
- Add a stress case where `no_recovery_fsm` demonstrably remains lost after
  dropout while FSM-controlled search recovers.
- Add a stress case where disabling occlusion score selects a geometrically
  reachable but visually blocked viewpoint.

## Closure Pass

Timestamp: 2026-05-23.

Implemented after the first stress pass:

- `search_reacquire` was replaced by a recovery search fan:
  `search_reacquire_1` through `search_reacquire_5`.
- Failed-candidate cooldown now filters by candidate name and nearby candidate
  geometry.
- Added `no_visibility_score` condition.
- Reduced viewpoint prediction blend from 0.45 to 0.20. Interpretation: the
  current 2-D prototype supports using prediction mainly for uncertainty/safety
  rather than claiming predicted-position viewpoint placement is beneficial.
- Candidate visibility is now evaluated from each candidate heading and mixed
  with measurement confidence in the score.

Verification:

- `check_matlab_code` clean for changed MATLAB files.
- `run_matlab_test_file` on `tests/tPaperLineCore.m`: 11 passed, 0 failed.
- Stage-one batch rerun:
  `runPaperLineBatch(Seeds=1:3, SaveOutputs=true)`.
- Stress batch rerun:
  `runPaperLineStressBatch(Seeds=1:3, SaveOutputs=true)`.

Updated stress condition results:

| condition | visible ratio | loss duration | near-miss count | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|
| fixed_behind | 0.8036 | 4.7333 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| fixed_safety_margin | 0.9646 | 0.8533 | 108.6000 | 1.4000 | 0.4000 | 0.0000 |
| nearest_feasible | 0.8343 | 3.9933 | 0.0000 | 1.0667 | 0.4000 | 0.8667 |
| no_occlusion_score | 0.8235 | 4.2533 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| no_planner_feedback | 0.8545 | 3.5067 | 0.0000 | 15.4000 | 15.4000 | 0.6000 |
| no_prediction | 0.9270 | 1.7600 | 0.0000 | 1.1333 | 0.4000 | 1.0000 |
| no_recovery_fsm | 0.8293 | 4.1133 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| no_visibility_score | 0.8266 | 4.1800 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| proposed | 0.8266 | 4.1800 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |

Updated stage-one condition results:

| condition | visible ratio | loss duration | near-miss count | planner failures | task success |
|---|---:|---:|---:|---:|---:|
| ca_kf | 0.8940 | 2.5556 | 0.0000 | 0.2778 | 0.7222 |
| fixed_behind | 0.9156 | 2.0333 | 0.0000 | 0.0000 | 1.0000 |
| fixed_safety_margin | 0.9564 | 1.0500 | 67.3889 | 0.3333 | 0.1667 |
| nearest_feasible | 0.9525 | 1.1444 | 0.0000 | 0.0000 | 1.0000 |
| no_occlusion_score | 0.9297 | 1.6944 | 0.0000 | 0.1667 | 1.0000 |
| no_planner_feedback | 0.9297 | 1.6944 | 0.0000 | 2.6667 | 0.8333 |
| no_prediction | 0.9421 | 1.3944 | 0.0000 | 0.2222 | 1.0000 |
| no_recovery_fsm | 0.9297 | 1.6944 | 0.0000 | 0.3333 | 1.0000 |
| no_visibility_score | 0.9297 | 1.6944 | 0.0000 | 0.3333 | 1.0000 |
| proposed | 0.9297 | 1.6944 | 0.0000 | 0.3333 | 1.0000 |

Final stage conclusion:

- Keep as supported mechanisms:
  - dynamic safety margin;
  - planner feedback;
  - failed-candidate cooldown/blacklist;
  - planner failure burst metric;
  - separate stage-one and stress-suite evaluation.
- Downgrade to candidate mechanisms requiring later proof:
  - predicted-position viewpoint placement;
  - occlusion score as an independent contribution;
  - visibility score as an independent contribution;
  - FSM recovery as an independent contribution.
- Do not continue tuning this 2-D prototype just to make the full proposed row
  win. At this point, additional tuning would be less valuable than moving to a
  cleaner experiment design or a planner-in-the-loop validation once the user is
  ready.
