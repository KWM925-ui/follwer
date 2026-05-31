# Paper-Line 专利技术交底书草稿

状态：草稿，2026-06-01。

这份文件用于后续交给专利代理人或继续完善技术交底书。它不是正式权利要求书。
当前写法要保守，避免把没有证据的内容写成核心创新。

## 1. 专利名称

候选名称：

1. 一种面向无人机人跟随的安全感知跟随视点决策方法
2. 一种基于动态安全边界和规划反馈的无人机人跟随视点选择方法
3. 一种用于无人机人跟随的连续规划失败抑制与安全视点选择方法

推荐：

> 一种基于动态安全边界和规划反馈的无人机人跟随视点选择方法

原因：

- 没有把“人跟随”本身说成创新；
- 突出了当前最有证据的两个点：动态安全边界、规划反馈；
- 不把 prediction 写成标题主贡献。

## 2. 技术领域

本发明涉及无人机自主跟随、移动机器人目标跟随和路径规划控制领域，尤其涉及
一种用于无人机人跟随任务的上层跟随视点决策方法。该方法位于目标感知/定位
模块和下游局部规划器之间，用于生成安全感知的跟随视点，并根据下游规划反馈
抑制连续不可行指令。

## 3. 背景技术

无人机人跟随系统通常包括目标检测、目标跟踪、定位建图、局部规划和飞控执行
等模块。已有系统可以根据目标当前位置生成跟随目标点，并交由下游规划器进行
路径规划和避障。

但是，在遮挡、目标短时丢失、目标突然转向或下游规划器拒绝目标点时，简单的
固定跟随点或固定安全边界存在以下问题：

1. 固定后方跟随点容易在遮挡或转角场景中失去观察视角；
2. 固定安全边界不能根据无人机运动速度和目标预测不确定性调整，可能导致
   候选视点过近；
3. 如果上层决策层不知道下游规划器是否成功执行目标点，可能连续重复发送
   不可行目标；
4. 只统计规划失败总数不足以反映连续失败风险，连续失败长度更能反映实际
   控制风险。

因此，需要一种不依赖特定下游规划器内部实现的上层跟随视点决策方法，在候选
视点选择前进行动态安全过滤，并利用规划反馈抑制连续不可行目标。

## 4. 发明目的

本发明的目的在于提供一种面向无人机人跟随任务的上层跟随视点决策方法，用于：

1. 根据目标预测不确定性和无人机运动状态动态调整候选视点安全边界；
2. 在评分前过滤不满足安全边界的候选视点；
3. 根据下游规划器或执行器反馈记录失败候选；
4. 对失败候选进行 cooldown 或 blacklist 处理，减少连续重复不可行指令；
5. 在目标短时丢失或长时间丢失时输出 hold/search/failsafe 等状态请求。

## 5. 技术方案

### 5.1 系统组成

该方法包括以下模块：

1. 目标状态估计模块；
2. 目标短时预测和不确定性估计模块；
3. 候选跟随视点生成模块；
4. 动态安全边界计算模块；
5. 候选视点硬安全过滤模块；
6. 候选视点评分模块；
7. 下游规划反馈接收模块；
8. 失败候选 cooldown / blacklist 模块；
9. 目标丢失和搜索状态管理模块；
10. 跟随视点输出模块。

### 5.2 输入输出

输入包括：

- 目标位置、速度或目标状态估计；
- 目标预测协方差或不确定性；
- 无人机当前位姿、速度或里程计；
- 障碍物信息、占据栅格或由其计算得到的 clearance；
- 下游规划器或执行器的 command-health feedback；
- 上一时刻的状态、候选视点和失败候选记录。

输出包括：

- 跟随视点位置和朝向；
- hold 命令；
- search / reacquire 命令；
- failsafe 请求；
- 状态信息和调试路径。

### 5.3 方法流程

1. 接收目标测量、无人机状态和障碍物信息；
2. 根据目标测量更新目标状态估计；
3. 预测短时目标状态并计算目标位置预测不确定性；
4. 根据目标是否可见、目标丢失时间、候选可用性和 planner feedback 更新状态；
5. 在普通跟随状态生成 `behind`、`left`、`right`、`far_safe` 等候选视点；
6. 在搜索或恢复状态生成 `search_reacquire_*` 候选视点；
7. 根据无人机速度和预测不确定性计算动态安全边界；
8. 对候选视点进行硬安全过滤；
9. 对通过过滤的候选视点进行评分；
10. 输出得分最优的跟随视点或 hold/search/failsafe 请求；
11. 接收下游 planner command-health feedback；
12. 若候选视点被拒绝，则记录该候选名称或几何位置，并在 cooldown 时间内
    抑制再次选择；
13. 记录 failure burst 等指标，用于评价连续不可行指令。

## 6. 关键技术点

### 6.1 动态安全边界

动态安全边界可表示为：

```text
margin = body_radius
       + obstacle_margin
       + k_v * speed_term
       + k_p * sqrt(lambda_max(P_pred_position))
```

其中：

- `body_radius` 为无人机自身尺寸相关项；
- `obstacle_margin` 为基础障碍物安全余量；
- `speed_term` 与无人机运动速度相关；
- `P_pred_position` 为目标预测位置协方差；
- `lambda_max` 表示位置协方差最大特征值。

该边界不是固定值，而是随运动速度和目标预测不确定性变化。

### 6.2 硬安全过滤先于评分

候选视点在进入评分前必须先通过安全过滤。过滤条件可以包括：

- 候选视点 clearance 小于动态安全边界；
- 无人机当前位置到候选视点的线段 clearance 小于动态安全边界；
- 候选视点不可达；
- 候选视点超出工作空间或高度限制。

不满足安全条件的候选视点直接拒绝，不允许通过加权评分获得选择机会。

### 6.3 planner command-health feedback

下游规划器或执行器向上层决策层反馈目标点是否被接受、是否产生可执行命令或
是否连续失败。上层决策层使用该反馈更新候选视点状态。

### 6.4 失败候选 cooldown / blacklist

当某候选视点被下游拒绝或导致连续规划失败时，系统记录该候选的名称或几何
邻近区域。在设定时间内，该候选被暂时抑制或降低优先级，以避免连续重复提交
不可行目标。

### 6.5 failure burst 指标

除记录规划失败总数外，系统还记录连续失败长度，即 failure burst。该指标用于
评价连续不可行指令风险，也可作为调整 cooldown 或状态切换的依据。

## 7. 有益效果

基于当前实验，能够支持的有益效果如下：

1. 相比固定安全边界，动态安全边界可以降低 near-miss 行为；
2. 硬安全过滤先于评分，可以避免不安全候选通过软评分被选中；
3. 相比无 planner feedback，加入 planner feedback 和 failed-candidate
   cooldown 可以降低连续不可行目标的长度；
4. failure burst 指标可以比失败总数更直接反映连续规划失败风险；
5. 该方法能够以外部 goal provider 的形式接入 ROS1/EGO 软件链，而不需要修改
   EGO、PX4、SLAM 或检测器内部。

## 8. 实验支撑

### 8.1 MATLAB 诊断实验

证据文件：

- `research/notes/2026-05-31_windows_matlab_refresh_log.md`

关键结果：

- stage-one batch: 300 runs；
- stress batch: 225 runs；
- `fixed_safety_margin` 在 stage-one 中 `nearMiss=67.0333`；
- `fixed_safety_margin` 在 stress 中 `nearMiss=107.8000`；
- `no_planner_feedback` 在 stress 中 `plannerFailureBurstMax=15.4000`；
- 场景级 `planner_feedback_stress` 中无反馈条件可达到
  `plannerFailureBurstMax_mean=61`。

### 8.2 Ubuntu20 ROS1/EGO 验证

证据文件：

- `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`

关键结果：

- `catkin_make` 通过；
- normal ROS1/EGO regression 单次 PASS；
- target-loss/search ROS1/EGO regression 单次 PASS；
- normal repeated regression 5/5 PASS；
- target-loss/search bag 中观察到 `follow`、`predict_hold`、
  `search_safe_viewpoint`；
- target-loss/search bag:
  `goal_count=127`, `ego_cmd_count=873`, `search_count=16`。

## 9. 附图建议

建议准备以下附图：

1. 系统模块图；
2. 方法流程图；
3. 动态安全边界示意图；
4. 候选跟随视点示意图；
5. 硬安全过滤和评分顺序图；
6. planner feedback 与 failed-candidate cooldown 流程图；
7. failure burst 指标示意图；
8. ROS1/EGO 软件链接入图；
9. target-loss/search 状态转移图。

## 10. 权利要求草案方向

### 独立权利要求方向

一种用于无人机人跟随的跟随视点选择方法，包括：

1. 获取目标状态、无人机状态和障碍物信息；
2. 对目标状态进行短时预测并获得预测不确定性；
3. 生成多个候选跟随视点；
4. 根据无人机速度和预测不确定性计算动态安全边界；
5. 在候选视点评分前，基于动态安全边界过滤不安全候选视点；
6. 对通过过滤的候选视点评分并输出跟随视点；
7. 获取下游规划器或执行器反馈；
8. 对被拒绝或连续失败的候选视点进行 cooldown 或 blacklist；
9. 根据目标可见性和规划反馈输出 follow、hold、search 或 failsafe 状态。

### 从属权利要求方向

可以围绕以下点展开：

- 动态安全边界的速度项；
- 动态安全边界的预测协方差项；
- 候选视点包括 `behind`、`left`、`right`、`far_safe`；
- 搜索状态专用 `search_reacquire_*` 候选；
- planner feedback 的失败候选记录；
- 失败候选几何邻近区域 cooldown；
- failure burst 作为评价或控制指标；
- 输出 topic 或接口形式可替换，不限于 ROS1/EGO。

## 11. 不能写成核心保护点的内容

当前不要把以下内容写成核心创新：

- CV-KF 本身；
- 无人机人跟随本身；
- 目标检测或跟踪；
- 目标重获取的一般方法；
- 避障的一般方法；
- EGO planner 或 PX4 控制；
- prediction 单独提升跟随效果；
- proposed 全面优于所有 baseline。

## 12. 后续补强

如果要增强专利或论文支撑，优先补：

1. obstacle-near ROS1 场景；
2. planner-blocked ROS1 场景；
3. no-feedback ROS1 ablation；
4. fixed-behind ROS1 baseline；
5. failure burst 图；
6. 动态安全边界和 fixed margin 对比图。
