# Paper-Line 论文草稿骨架

状态：初稿骨架，2026-06-01。

这份文件是写论文用的“可展开骨架”。它不是最终论文，也不应把弱证据写成强
结论。所有结论必须服从：

- `research/notes/2026-05-31_windows_matlab_refresh_log.md`
- `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`
- `docs/PAPER_LINE_EVIDENCE_PACKAGE_CN.md`

## 题目候选

中文：

- 面向无人机人跟随的规划反馈感知安全跟随视点决策方法
- 基于动态安全边界和规划反馈的无人机人跟随视点选择方法
- 一种用于无人机人跟随的安全感知上层视点决策框架

英文：

- A Planner-Feedback-Aware Safe Follow-Viewpoint Decision Layer for UAV Human Following
- Dynamic Safety Filtering and Planner-Feedback Suppression for UAV Human-Following Viewpoint Selection

建议不要在题目中突出 prediction，当前数据不支持把 prediction 当主贡献。

## 摘要草稿

无人机人跟随任务中，上层跟随目标点如果只根据当前目标位置或固定后方点生成，
容易在遮挡、目标短时丢失和下游规划失败时产生近失风险或重复不可行指令。
本文提出一种不修改下游规划器的上层跟随视点决策层。该方法根据目标状态估计
和预测不确定性生成候选跟随视点，先用动态安全边界进行硬过滤，再对候选点
进行跟随质量评分；同时接收下游 planner command-health feedback，对失败候选
执行 cooldown，以降低连续重复提交不可行目标的风险。MATLAB 诊断实验表明，
相较固定安全边界，动态安全边界显著降低 near-miss；相较无 planner feedback，
反馈和 cooldown 明显降低 planner failure burst。Ubuntu20 ROS1/EGO 软件链验证
表明，该决策层可作为外部 Stage2 goal provider 接入现有 EGO 链路，并在
target-loss/search 场景中产生 `follow`、`predict_hold` 和
`search_safe_viewpoint` 状态。当前结果支持该方法作为安全感知上层决策层，
但不声称实机安全或对所有 baseline 的全面优越。

## 1. Introduction

### 1.1 问题背景

无人机人跟随不是只“看见人”就结束。实际链路里，上层还要持续决定无人机应该
跟到哪里、从哪个方向看目标、目标短暂丢失时如何保持或搜索、下游规划器拒绝
目标点时如何避免重复发送同一个坏指令。

### 1.2 现有问题

可以这样写：

- 固定后方跟随点在正常场景简单有效，但在障碍、转角或遮挡附近容易失去视角；
- 固定安全边界不能随速度和预测不确定性变化，可能在部分场景给出过近候选点；
- 上层决策如果不知道下游 planner 是否接受目标点，可能重复提交不可行目标；
- 只看失败总次数不够，连续失败长度更能反映实际控制风险。

不要这样写：

- “现有方法都不能解决人跟随”；
- “本文保证无碰撞”；
- “本文提出全新的无人机人跟随系统”。

### 1.3 本文贡献

建议收紧成 3 点：

1. 提出一种 planner-agnostic 的上层跟随视点决策层，在不修改 EGO/PX4/SLAM/
   detector 的前提下输出跟随视点、hold/search/failsafe 请求。
2. 提出动态安全边界和硬安全过滤顺序，将 UAV 速度项和目标预测不确定性引入
   候选点安全过滤，避免不安全候选靠加权评分软通过。
3. 引入 planner command-health feedback 和 failed-candidate cooldown，并使用
   failure burst 评价连续不可行指令，从而降低重复规划失败风险。

可作为工程验证贡献：

- MATLAB stage/stress 诊断实验；
- Ubuntu20 ROS1/EGO adapter 验证；
- target-loss/search 状态链路验证。

## 2. Related Work

建议分四类写：

1. UAV/robot human following；
2. short-horizon target prediction；
3. visibility-aware or active viewpoint planning；
4. safe local planning and planner failure handling。

写作原则：

- 预测、可见性、避障都不是本文要抢的最大创新；
- 本文强调“上层跟随视点决策 + 动态安全过滤 + planner feedback/cooldown”；
- 需要后续补正式引用和专利/论文对比。

## 3. System Overview

### 3.1 系统边界

本文方法位于目标融合输出和下游 EGO 目标输入之间。

输入：

- target position / velocity estimate；
- prediction covariance；
- UAV odom；
- obstacle clearance 或占据栅格派生信息；
- planner command-health feedback；
- 上一帧状态和失败候选记忆。

输出：

- `/follow/stage2/goal`；
- `/follow/stage2/debug_goal_path`；
- `/follow/stage2/state`；
- hold/search/failsafe 请求语义。

### 3.2 ROS1 接入

Ubuntu20 验证中的 adapter：

- `src/human_follow_user/scripts/user_stage2_goal_node.py`

关键 topic 链：

```text
/follow/fusion/target_world
/follow/lio/odom
        -> paper-line adapter
/follow/stage2/goal
        -> /move_base_simple/goal
        -> EGO
/follow/stage2/ego_position_cmd
        -> PX4 bridge / fake MAVROS gate
```

## 4. Method

### 4.1 Target State Estimation

默认使用 CV-KF 做短时估计和预测。当前论文不能强调“预测提升跟随效果”，更稳妥
的说法是：

> 预测模块主要提供短时目标状态和不确定性，后者用于动态安全边界。

### 4.2 Candidate Follow-Viewpoint Generation

普通候选：

- `behind`
- `left`
- `right`
- `far_safe`

搜索候选：

- `search_reacquire_*`

注意：

- 搜索候选只在 recovery/search 状态使用；
- 候选包含 position 和 target-facing heading；
- 多候选不是为了证明所有场景最好，而是提供可解释选择空间。

### 4.3 Dynamic Safety Margin

核心公式可写：

```text
margin = body_radius
       + obstacle_margin
       + k_v * speed_term
       + k_p * sqrt(lambda_max(P_pred_position))
```

解释：

- `body_radius + obstacle_margin` 是基础安全壳；
- `speed_term` 反映运动越快需要更保守；
- `P_pred_position` 反映目标预测越不确定，候选点应保持更大余量。

论文重点：

- 安全过滤发生在评分之前；
- 不安全候选直接拒绝；
- 不声明形式化 collision-free guarantee。

### 4.4 Candidate Scoring

只对通过硬过滤的候选评分。评分项可以包括：

- 跟随距离误差；
- view angle / visibility；
- clearance；
- occlusion risk；
- motion cost；
- switching penalty。

写作上不要把加权评分本身当主创新。

### 4.5 Planner Feedback And Cooldown

下游 planner 或执行器反馈目标点是否可执行。若候选点失败：

1. 记录候选名称或几何邻近区域；
2. 在 cooldown 时间内降低或禁止再次选择该候选；
3. 避免连续重复提交不可行目标；
4. 使用 `plannerFailureBurstMax` 衡量连续失败。

这是当前最强贡献之一。

### 4.6 State Machine

状态：

- `follow`
- `predict_hold`
- `search_safe_viewpoint`
- `hold_safe`
- `failsafe`

当前证据支持：

- target-loss/search 状态能在 ROS1/EGO 软件链中出现；
- 不能把 FSM recovery 写成独立主贡献。

## 5. Experiments

### 5.1 MATLAB Diagnostic Suite

场景：

- `straight`
- `sudden_turn`
- `obstacle_occlusion`
- `short_loss`
- `occlusion_reacquire`
- `planner_failure`

stress 场景：

- `fixed_behind_occlusion`
- `nearest_visibility_trap`
- `recovery_corner_loss`
- `planner_feedback_stress`
- `planner_blocked_goal`

条件：

- `proposed`
- `fixed_behind`
- `nearest_feasible`
- `no_prediction`
- `fixed_safety_margin`
- `no_recovery_fsm`
- `ca_kf`
- `no_occlusion_score`
- `no_visibility_score`
- `no_planner_feedback`

### 5.2 Ubuntu20 ROS1/EGO Validation

验证目标：

- 证明 paper-line adapter 能在 ROS1/EGO 软件链中运行；
- 证明 topic chain 打通；
- 证明 target-loss/search 状态在 ROS bag 中可观察。

不用于证明：

- 实机安全；
- strict Gazebo/RViz full-chain；
- 全指标优越。

## 6. Results

### 6.1 Dynamic Safety Margin

可写结论：

- stage-one 中 `fixed_safety_margin` 的 `nearMiss=67.0333`，`taskSuccess=0.1667`；
- stress 中 `fixed_safety_margin` 的 `nearMiss=107.8000`，`taskSuccess=0.0000`；
- proposed 在两组中 near-miss 都是 0。

### 6.2 Planner Feedback/Cooldown

可写结论：

- stage-one `no_planner_feedback` 的 `plannerFailureBurstMax=2.6667`；
- stress `no_planner_feedback` 的 `plannerFailureBurstMax=15.4000`；
- 场景级 `planner_feedback_stress` 可达到 `plannerFailureBurstMax_mean=61`；
- proposed 的 stress `plannerFailureBurstMax=0.4000`。

### 6.3 Prediction Boundary

必须如实写：

- `no_prediction` 在 stage-one 和 stress 中很强；
- stress 中 `no_prediction taskSuccess=1.0000`，proposed 是 `0.8000`；
- 所以 prediction 不作为独立主贡献。

### 6.4 ROS1/EGO Validation

可写：

- repeated normal regression 5/5 PASS；
- normal bag: `goal_count=171`, `ego_cmd_count=1133`；
- target-loss/search bag: `goal_count=127`, `ego_cmd_count=873`；
- target-loss/search 状态驻留：
  `follow=5.714 sec`, `predict_hold=1.887 sec`,
  `search_safe_viewpoint=1.081 sec`。

## 7. Discussion

应该主动承认：

- 当前结果是诊断和软件链验证，不是实机；
- full proposed row 不全指标最好；
- 预测、遮挡评分、可见性评分、FSM recovery 暂时是工程组成；
- 动态安全过滤和 planner feedback/cooldown 是当前最稳贡献。

## 8. Conclusion

可以这样收束：

本文提出并验证了一种无人机人跟随上层视点决策层。MATLAB 诊断实验表明，
动态安全边界和硬安全过滤可以降低近失风险，planner feedback/cooldown 可以
降低连续不可行指令。Ubuntu20 ROS1/EGO 验证表明，该决策层能以外部 goal
provider 形式接入现有软件链，并在 target-loss/search 场景中产生可观测状态
转移。后续工作将围绕更严格的 Gazebo/RViz 全链路验证、实机实验和更系统的
baseline 对比展开。

## 9. 当前缺口

写正式论文前还缺：

- 正式参考文献；
- 图表绘制；
- 更漂亮的实验表格格式；
- 论文语言统一；
- 如果要投更高要求会议/期刊，需要补更多 baseline 或场景。
