# Paper-Line 技术路线书

状态：Ubuntu20 ROS1 验证后收口版，2026-06-01。
范围：paper-line only。不得修改 `ubuntu-mainline`、EGO/PX4/SLAM/检测器。
当前允许的部署侧改动仅限 `src/human_follow_user/` 中的外部算法接入口，
用于把 paper-line 决策层接入已有 ROS1 Stage2 链路。

## 1. 项目定位

paper-line 不做“完整无人机人跟随系统”，也不做检测、融合、SLAM、EGO 或 PX4 的改进。

当前最稳的定位是：

> 面向无人机人体跟随的上层跟随视点决策层，在不修改下游规划器的前提下，利用预测不确定性、动态安全边界、下游 planner 反馈和失败候选抑制，降低近失风险和重复不可行指令。

必须避免的表述：

- 新的人体跟随算法；
- 新的目标重获取算法；
- 新的人体轨迹预测算法；
- 保证安全、保证无碰撞；
- 端到端自主无人机跟随系统。

## 2. 已确认的主贡献边界

当前证据支持的核心机制：

1. 动态安全边界：
   `body_radius + obstacle_margin + speed term + prediction covariance term`。
2. 硬安全过滤先于评分：
   不安全候选点不能靠加权得分“软通过”。
3. planner command-health feedback：
   下游 planner 或执行器拒绝目标点后，上层决策层接收反馈。
4. 失败候选 cooldown / blacklist：
   被拒绝的候选名或附近几何位置短时间内不再重复提交。
5. failure burst 指标：
   不只看失败总数，还要看连续失败长度。

当前不能作为主贡献的机制：

- 预测点视点生成；
- occlusion score 独立贡献；
- visibility score 独立贡献；
- FSM recovery 独立贡献。

原因：

- MATLAB 2D 诊断中 `no_prediction` 经常强于 proposed。
- `no_occlusion_score`、`no_visibility_score`、`no_recovery_fsm` 与 proposed 差异不足。
- 这些模块可以保留为可选组件，但后续必须有更强实验再提升为贡献点。

## 3. 系统接口

输入：

- 目标位置、速度估计；
- 目标预测协方差；
- 目标可见性或测量置信度；
- UAV 当前状态；
- 障碍物或占据栅格派生的 clearance；
- 下游 planner / executor 健康状态；
- 上一帧候选点、状态机状态和失败候选记忆。

输出：

- 跟随视点；
- hold command；
- search / reacquire command；
- failsafe request。

## 4. 方法框架

```text
target measurement
UAV state
obstacle model
planner feedback
        -> CV-KF state estimate and covariance
        -> candidate viewpoint generation
        -> dynamic safety margin
        -> hard safety filter
        -> optional visibility / occlusion scoring
        -> selected viewpoint
        -> planner command-health feedback
        -> failed-candidate cooldown
```

## 5. 算法选择

### 5.1 预测

默认使用 CV-KF。

选择原因：

- 低延迟；
- 可解释；
- 能提供协方差；
- MATLAB 容易复现；
- 不把论文拖进数据驱动预测模型路线。

证据边界：

- 目前只支持“协方差用于动态安全边界”。
- 不支持“预测点生成跟随视点一定更好”。

### 5.2 候选视点

普通状态候选：

- `behind`
- `left`
- `right`
- `far_safe`

恢复状态候选：

- `search_reacquire_1 ... search_reacquire_5`

候选点包含：

- 2D position；
- target-facing heading。

### 5.3 安全过滤

硬过滤条件：

- 候选点 clearance 小于动态 margin；
- UAV 到候选点的线段 clearance 小于动态 margin；
- 候选点不可达；
- 后续可扩展工作空间、速度、加速度、高度约束。

### 5.4 评分

当前评分项保留为工程实现：

- 跟随距离；
- 视角；
- clearance；
- occlusion risk；
- motion cost；
- candidate visibility；
- switching penalty；
- prediction uncertainty。

论文写作时不能把“加权评分函数”作为主创新点。它只能作为动态安全过滤和 planner feedback 闭环之后的选择器。

### 5.5 planner feedback

planner feedback 当前是最值得保留的路线之一。

机制：

- 下游 planner 接收候选视点；
- 若目标点或路径不可行，返回 command-health failure；
- 上层记录失败候选；
- 对同名候选或几何邻近候选施加 cooldown；
- 避免连续重复提交不可行目标。

支持证据：

- stress suite 中 `no_planner_feedback` 的 max failure burst 为 15.4；
- proposed 使用 feedback/cooldown 后 max failure burst 为 0.4。

## 6. 实验设计

### 6.1 Stage-One Batch

场景：

- straight；
- sudden_turn；
- obstacle_occlusion；
- short_loss；
- occlusion_reacquire；
- planner_failure。

条件：

- proposed；
- fixed_behind；
- nearest_feasible；
- no_prediction；
- fixed_safety_margin；
- no_recovery_fsm；
- ca_kf；
- no_occlusion_score；
- no_visibility_score；
- no_planner_feedback。

当前结果摘要（2026-05-31 Windows/MATLAB，Seeds=1:5）：

| condition | visible ratio | loss duration | min clearance | near-miss | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| proposed | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 0.3333 | 0.1667 | 1.0000 |
| fixed_safety_margin | 0.9570 | 1.0367 | 1.2829 | 67.0333 | 0.3333 | 0.1667 | 0.1667 |
| no_planner_feedback | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 2.6667 | 2.6667 | 0.8333 |
| no_prediction | 0.9411 | 1.4200 | 1.8872 | 0.0000 | 0.2333 | 0.1667 | 1.0000 |

解释：

- fixed safety margin 明显失败；
- planner feedback 有价值；
- no_prediction 很强，预测不能作为主效果宣称。

### 6.2 Stress Batch

场景：

- fixed_behind_occlusion；
- nearest_visibility_trap；
- recovery_corner_loss；
- planner_feedback_stress；
- planner_blocked_goal。

当前结果摘要（2026-05-31 Windows/MATLAB，Seeds=1:5）：

| condition | visible ratio | loss duration | min clearance | near-miss | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| proposed | 0.8264 | 4.1840 | 1.5504 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| fixed_safety_margin | 0.9650 | 0.8440 | 0.8458 | 107.8000 | 1.4000 | 0.4000 | 0.0000 |
| no_planner_feedback | 0.8551 | 3.4920 | 1.3177 | 0.0000 | 15.4000 | 15.4000 | 0.6000 |
| no_prediction | 0.9129 | 2.1000 | 1.6739 | 0.0000 | 1.1200 | 0.4000 | 1.0000 |

解释：

- 动态安全边界有明确价值；
- planner feedback/cooldown 有明确价值；
- proposed 不是全指标最好；
- 不能把 full stack 写成“全面优于 baseline”。

## 7. 指标

主指标：

- visible ratio；
- loss duration；
- near-miss count；
- min clearance；
- planner failure count；
- planner failure burst max；
- task success。

辅助指标：

- candidate switch count；
- reacquisition time；
- state switch count；
- prediction RMSE；
- decision latency。

论文建议：

- 不只报 task success；
- 必须分开报 safety、visibility、planner failure；
- failure burst 比 failure count 更能体现 feedback/cooldown 的价值。

## 8. 论文路线

建议论文主线：

1. 问题：
   固定跟随点、固定安全边界、无 planner feedback 的人跟随上层决策容易造成近失风险和重复不可行指令。
2. 方法：
   上层 planner-agnostic follow-viewpoint decision layer。
3. 核心：
   动态安全过滤 + command-health feedback + failed-candidate cooldown。
4. 实验：
   MATLAB 2D stage/stress suite。
5. 结论：
   该诊断基线证明动态安全边界和 feedback/cooldown 值得进入下一阶段。

暂不建议写：

- proposed 全面超过 baseline；
- prediction 使跟随显著更好；
- FSM recovery 是成熟贡献；
- occlusion score 是独立贡献。

## 9. 专利准备路线

当前可保留的可保护点：

- 动态安全边界；
- 候选点硬过滤顺序；
- planner command-health feedback；
- 失败候选 cooldown / blacklist；
- failure burst 作为控制/评价指标；
- 候选视点 position + heading 表达。

当前不宜写进主权利要求：

- CV-KF 本身；
- generic target recovery；
- generic visibility-aware planning；
- generic UAV human following；
- 加权评分函数本身；
- EGO/PX4/SLAM/检测器相关内容。

## 10. ROS1 接入状态

当前已把 paper-line 决策层落成 ROS1 外部 Stage2 goal provider：

- `src/human_follow_user/scripts/user_stage2_goal_node.py`

该节点不依赖 MATLAB。MATLAB 只作为研究、验证和出图工具。

ROS1 节点实现内容：

- CV-KF 目标状态估计和短时预测；
- 动态安全边界；
- 多候选跟随视点；
- 硬安全过滤；
- 加权评分；
- `idle / follow / predict_hold / search_safe_viewpoint / hold_safe / failsafe`
  状态；
- 可选 planner command feedback 和 failed-candidate cooldown。

当前已完成：

- Python 语法检查；
- 离线核心逻辑 smoke test。
- Ubuntu 20.04 + ROS Noetic `catkin_make`；
- normal ROS1/EGO regression 单次 PASS；
- target-loss/search ROS1/EGO regression 单次 PASS；
- normal ROS1/EGO repeated regression 5/5 PASS；
- normal 和 target-loss/search rosbag metrics PASS。

关键证据记录：

- `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`
- repeated artifact:
  `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002516`
- normal bag metrics:
  `research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation_metrics`
- target-loss/search bag metrics:
  `research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation_metrics`

当前 ROS1/EGO 证据支持：

- paper-line adapter 能在 ROS1/EGO 软件链中运行；
- `/follow/stage2/state` 和 `/follow/stage2/goal` 能发布；
- EGO 能收到 `/move_base_simple/goal`；
- EGO 能输出 `/follow/stage2/ego_position_cmd`；
- bridge / fake MAVROS 链路能走到 `/follow/stage2/offboard/setpoint`
  和 `/mavros/setpoint_raw/local`；
- target-loss/search 中实际出现 `follow`、`predict_hold`、
  `search_safe_viewpoint`。

仍不能声称：

- 实机飞行安全；
- 严格 Gazebo + RViz 全链路完成；
- proposed 全面优于所有 baseline；
- prediction、occlusion score、visibility score、FSM recovery 是独立主贡献。

## 11. 下一阶段

本阶段不再继续调 2D MATLAB 数值，也不需要反复重跑同一套 Ubuntu20
ROS1 验证，除非代码或环境发生变化。

下一阶段优先顺序：

1. 把 MATLAB 诊断结果和 Ubuntu20 ROS1/EGO 结果整理成论文/专利证据包；
2. 写论文方法和实验章节草案，把主线收紧到动态安全边界、硬安全过滤、
   planner feedback、failed-candidate cooldown 和 failure burst；
3. 只在能回答具体主张时，再补 ROS1 场景：
   obstacle-near、planner-blocked、fixed-behind baseline、no-feedback
   ablation；
4. Gazebo + RViz 严格全链路仿真放到后续系统级验证，不作为当前
   paper-line 结果包的前置阻塞。

只有论文/专利证据包缺少明确支撑时，才重新讨论 2.5D、更多
EGO planner-in-loop 场景或 Simulink。
