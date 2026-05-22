# 2026-05-23 External Research And ChatGPT Review

Scope: paper-line only.

## Roxy / ChatGPT Usage

RoxyBrowser was connected to the existing profile.

Protected tab not touched:

- `专利撰写协作方案`

New ChatGPT conversation created and tested with `你好`:

- https://chatgpt.com/c/6a107c4e-edf8-8325-82a9-e1d5527e2868

Review prompt sent:

- Asked ChatGPT to act as a strict research reviewer for the paper-line route.
- Included current route, MATLAB diagnostic failures, and requested research
  risks, MATLAB changes, baselines/ablations, patent-prep boundaries, and
  further search keywords.

Useful critique brought back into local work:

- Current diagnostic failures should be treated as evaluation-protocol problems,
  not as proof that the method works or fails.
- Candidate viewpoints should include both position and desired camera/yaw.
- Visibility needs decomposed debug fields and unit tests.
- Task success should be decomposed into tracking, recovery, and safety success.
- Planner failure must be an explicit command-health signal.

## Search Queries

Papers / research:

- `UAV active target tracking visibility-aware planning occlusion`
- `UAV active target tracking visibility-aware planning occlusion arxiv`
- `person following robot target recovery field of view loss trajectory prediction`
- `person following robot target recovery field of view loss trajectory prediction MDPI`
- `visibility-aware trajectory optimization target tracking occlusion distance angle`
- `planner failure recovery state machine robot planning feedback paper`

Patents:

- `site:patents.google.com UAV subject following obstacle avoidance target tracking`
- `site:patents.google.com drone follow mode obstacle avoidance subject tracking`
- `site:patents.google.com UAV target tracking obstacle avoidance follow target patent`
- `site:patents.google.com unmanned aerial vehicle tracking target obstacle avoidance follow target`
- `site:patents.google.com target tracking UAV disable tracking obstacle certain distance`

Products:

- `DJI ActiveTrack 360 obstacle sensing subject tracking official DJI Mini 4 Pro`
- `site:dji.com Mini 4 Pro ActiveTrack 360 Omnidirectional Obstacle Sensing official`
- `site:support.dji.com ActiveTrack 360 obstacle sensing subject tracking DJI official`
- `site:support.skydio.com subject tracking obstacle avoidance follow Skydio official`
- `site:support.skydio.com Skydio tracking subject follow obstacle avoidance`

## Source Notes

| Area | Source | Link | Fact / Use |
|---|---|---|---|
| person-following recovery | Target Recovery for Robust Deep Learning-Based Person Following in Mobile Robots: Online Trajectory Prediction | https://www.mdpi.com/2076-3417/11/9/4165 | Target loss recovery and online trajectory prediction are established in person-following research. |
| human trajectory prediction | Human Motion Trajectory Prediction: A Survey | https://arxiv.org/abs/1905.06113 | Human motion prediction is broad prior art; CV-KF is a baseline/engineering choice, not novelty. |
| human-following robots | A Survey of Human-Following Robotic Systems for Researchers and End-users | https://arxiv.org/abs/1803.08202 | Broad human-following systems are crowded across perception, planning, and control. |
| active target tracking | Active Target Tracking Benchmark | https://arxiv.org/abs/2605.05338 | Active/visibility-aware tracking evaluation reinforces the need for reproducible scenario suites. |
| MATLAB tracking | MathWorks `trackingKF` | https://www.mathworks.com/help/fusion/ref/trackingkf.html | KF models are standard tracking baselines. |
| product prior art | DJI Mini 4 Pro official product page | https://www.dji.com/mini-4-pro | DJI advertises obstacle sensing and ActiveTrack-style subject tracking. |
| product prior art | Skydio subject tracking/follow support | https://support.skydio.com/hc/en-us/articles/45768944808347-How-to-use-Shadow-Subject-Track-and-Follow-Beta | Commercial subject-following products already expose subject tracking and obstacle-aware follow behavior. |
| patent prior art | US9321173B2, tracking and following people with a mobile robotic device | https://patents.google.com/patent/US9321173B2/en | Person tracking/following with robotic devices is prior art. |
| patent prior art | US20190258129A1, UAV target tracking with avoidance/disable logic | https://patents.google.com/patent/US20190258129A1/en | UAV target tracking with obstacle/distance-based behavior is patent-crowded. |

## Route Impact

Confirmed:

- Keep the contribution boundary narrow:
  planner-agnostic follow-viewpoint decision, not a new UAV following system.
- Keep CV-KF as default only because it is simple and covariance-bearing.
- Keep dynamic safety filtering, visibility/occlusion scoring, planner feedback,
  and FSM recovery as an integrated decision loop.

Rejected or deferred:

- Broad claims around UAV human following.
- Claims that trajectory prediction itself is new.
- Claims that target reacquisition itself is new.
- Claims that weighted scoring alone is new.
- Claims that current MATLAB results already prove full superiority.

Patent-prep evidence worth preserving now:

- candidate viewpoints with explicit camera/heading;
- decomposed visibility model and unit tests;
- dynamic safety margin ablation against fixed margin;
- planner feedback metric and command-health signal;
- composite task success decomposition.
