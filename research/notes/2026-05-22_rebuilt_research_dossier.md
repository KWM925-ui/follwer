# 2026-05-22 Rebuilt Paper-Line Research Dossier

Scope: paper-line only.

## Search Queries

Papers and technical route:

- `UAV human following trajectory prediction safety follow point paper`
- `Target Recovery for Robust Deep Learning-Based Person Following in Mobile Robots Online Trajectory Prediction`
- `human motion trajectory prediction survey arXiv UAV robotics`
- `EGO-Planner autonomous drone local planning paper GitHub`
- `Human Motion Trajectory Prediction A Survey arXiv 1905.06113`
- `EGO-Planner An ESDF-free Gradient-based Local Planner for Quadrotors paper`

Patents and products:

- `Google Patents UAV follow target reacquire obstacle avoidance person following`
- `Google Patents tracking following people mobile robotic device target loss recovery trajectory prediction`
- `Google Patents dynamic safety zone UAV following target obstacle avoidance covariance`
- `Google Patents follow me UAV obstacle avoidance target tracking safe distance`
- `DJI ActiveTrack obstacle avoidance subject tracking official manual`
- `Skydio subject tracking obstacle avoidance official manual`

MATLAB / implementation:

- `site:mathworks.com/help/fusion trackingKF constant velocity Kalman filter object tracking`
- `site:mathworks.com/help/fusion constvel constant velocity model trackingKF`
- `MATLAB Kalman filter object tracking constant velocity example official`

## Source Table

| Area | Source | Link | Fact Used | Route Impact |
|---|---|---|---|---|
| Target recovery and person following | Target Recovery for Robust Deep Learning-Based Person Following in Mobile Robots: Online Trajectory Prediction | https://www.mdpi.com/2076-3417/11/9/4165 | Person-following target recovery and online trajectory prediction are existing research topics. | Do not claim generic target recovery or prediction as new. |
| Human trajectory prediction | Human Motion Trajectory Prediction: A Survey | https://arxiv.org/abs/1905.06113 | Human motion prediction is a broad established field with many models and datasets. | Avoid LSTM/Transformer route as default; use simple predictor for decision-layer evidence. |
| Local UAV planning | EGO-Planner / ESDF-free local planning references | https://github.com/ZJU-FAST-Lab/ego-planner | Local trajectory planning is a separate established layer. | Treat EGO/planner as downstream executor or comparator only. |
| MATLAB tracking | MathWorks `trackingKF` documentation | https://www.mathworks.com/help/fusion/ref/trackingkf.html | Linear Kalman filter supports common object-tracking motion models. | CV-KF/CA-KF are reasonable MATLAB baselines, not contribution claims. |
| Product tracking | DJI subject tracking / ActiveTrack official materials | https://support.dji.com/help/content?customId=en-us03400006560&spaceId=34&re=US&lang=en | Commercial UAV systems include subject tracking behaviors. | Avoid claiming broad "UAV follow subject" novelty. |
| Product tracking | Skydio subject tracking official support | https://support.skydio.com/hc/en-us/articles/45768944808347-How-to-use-Shadow-Subject-Track-and-Follow-Beta | Commercial autonomy products expose subject-following and tracking modes. | Product prior art increases risk for broad follow/tracking claims. |
| Patent risk | US9321173B2, tracking/following people with a mobile robotic device | https://patents.google.com/patent/US9321173B2/en | Person tracking/following with robotic devices has patent coverage. | Patent claim must avoid broad person-following language. |
| Patent risk | US20250148634A1, follow-mode UAV operation with obstacle avoidance style concepts | https://patents.google.com/patent/US20250148634A1/en | Follow-mode UAV behavior with obstacle-aware adjustment is patent-crowded. | Keep patent/paper wording narrow around the specific decision-layer integration. |

## Confirmed Facts

- Person following, target recovery, and short-term human trajectory prediction are already crowded research areas.
- Commercial UAV products include subject-tracking/following behavior, so broad product-level claims are risky.
- Patents cover robot/person following and UAV follow-mode obstacle-aware behavior, so patent language must be narrow.
- MATLAB provides standard tracking/Kalman building blocks, making CV-KF and CA-KF suitable baselines rather than novelty points.
- EGO-style local planning is downstream planning, not the paper-line contribution.

## Inferences

- A defensible route should focus on how a top-level decision layer combines prediction uncertainty, visibility/occlusion, hard safety filtering, and recovery states.
- The paper should not be framed as a new detector, predictor, planner, target-recovery method, or obstacle-avoidance method.
- The first implementation should be MATLAB 2D/2.5D because it isolates the decision layer before deployment complexity.

## Route Risks

- The method may look like heuristic engineering unless experiments include strong baselines and ablations.
- A weighted score can look hand-tuned unless weights are fixed across scenario families and sensitivity is reported.
- "Safety" language can be challenged unless phrased as risk-aware / safety-aware rather than guaranteed collision-free.
- Planner-agnostic claims need an explicit interface and at least one executor/planner swap or simplified planner comparison.

## Decisions To Carry Forward

- Keep the contribution boundary at a planner-agnostic, safety-aware follow-viewpoint decision layer.
- Keep CV-KF as default engineering predictor unless new experiments show CA-KF or another lightweight predictor is better.
- Keep deep prediction and RL out of the first route.
- Keep hard safety filtering before scoring.
- Keep search/reacquire candidate generation inside recovery states, not normal following.
- Make baselines and ablations central to the paper, not optional.
