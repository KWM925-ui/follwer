# Paper-Line 论文表格和图计划

状态：初稿，2026-06-01。

这份文件把现有证据转换成论文/专利最容易使用的表格和图。当前先定义内容、
数据来源和结论边界；真正画图可以后续在 Windows/MATLAB 或 Python 中完成。

## 1. 表格清单

### 表 1：MATLAB Stage-One Ablation 总体结果

数据来源：

- `research/notes/2026-05-31_windows_matlab_refresh_log.md`

建议列：

- condition
- visible ratio
- loss duration
- min clearance
- near-miss
- planner failures
- max failure burst
- task success

建议重点列出的条件：

- `proposed`
- `fixed_safety_margin`
- `no_planner_feedback`
- `no_prediction`

主要结论：

- `fixed_safety_margin` near-miss 高，说明动态安全边界有必要；
- `no_planner_feedback` failure burst 高，说明 feedback/cooldown 有必要；
- `no_prediction` 不差，说明 prediction 不应作为主贡献。

不能写：

- proposed 全面优于所有 baseline。

### 表 2：MATLAB Stress Ablation 总体结果

数据来源：

- `research/notes/2026-05-31_windows_matlab_refresh_log.md`

建议重点列出的条件：

- `proposed`
- `fixed_safety_margin`
- `no_planner_feedback`
- `no_prediction`

主要结论：

- `fixed_safety_margin` 在 stress 中 `nearMiss=107.8000`；
- `no_planner_feedback` 在 stress 中 `plannerFailureBurstMax=15.4000`；
- proposed 保持 near-miss 为 0，但 task success 不是最高。

不能写：

- proposed 在所有压力场景都最好。

### 表 3：ROS1/EGO Regression 通过结果

数据来源：

- `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`

建议列：

- run type
- pass count
- distinct goals
- distinct commands
- final plan success 0
- final plan success 1
- sampled required topics
- timeout

建议行：

- normal single regression
- target-loss/search single regression
- repeated normal regression 5-run summary

主要结论：

- paper-line adapter 能在 ROS1/EGO 软件链中运行；
- target-loss/search 场景能触发更多 goal/cmd 和部分 planner failure；
- repeated normal regression 5/5 PASS。

不能写：

- 实机可飞；
- 真实障碍环境已经安全。

### 表 4：ROS Bag Metrics

数据来源：

- `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`
- normal metrics:
  `research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation_metrics`
- target-loss/search metrics:
  `research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation_metrics`

建议列：

- bag type
- duration
- goal count
- ego command count
- goal rate
- ego command rate
- max goal gap
- max command gap
- bridge echo distance
- states seen

主要结论：

- normal bag 的 `/follow/stage2/goal` 和 `/follow/stage2/ego_position_cmd`
  连续稳定；
- target-loss/search bag 中出现 `follow`、`predict_hold`、
  `search_safe_viewpoint`。

## 2. 图清单

### 图 1：方法总流程图

内容：

```text
target measurement / UAV state / obstacle clearance / planner feedback
        -> target estimation and prediction uncertainty
        -> candidate generation
        -> dynamic safety margin
        -> hard safety filtering
        -> candidate scoring
        -> selected follow viewpoint / hold / search / failsafe
        -> downstream planner
        -> command-health feedback
```

用途：

- 放在方法章节；
- 说明本文改的是上层决策层，不是 EGO/PX4/SLAM/detector。

### 图 2：动态安全边界示意图

内容：

- UAV 当前点；
- 障碍物；
- fixed margin 圆或安全壳；
- dynamic margin 安全壳；
- 被过滤的候选点和通过的候选点。

用途：

- 解释为什么安全过滤在评分前；
- 支撑专利中的“动态安全边界 + 硬过滤”。

### 图 3：Near-Miss 对比柱状图

数据：

| suite | proposed | fixed_safety_margin |
|---|---:|---:|
| stage-one | 0.0000 | 67.0333 |
| stress | 0.0000 | 107.8000 |

用途：

- 直接支撑动态安全边界；
- 论文和专利都可用。

注意：

- 不要只画 visible ratio，因为 fixed margin 的 visible ratio 可能更高，但那是以
  near-miss 为代价。

### 图 4：Planner Failure Burst 对比柱状图

数据：

| suite/scenario | proposed | no_planner_feedback |
|---|---:|---:|
| stage-one aggregate | 0.1667 | 2.6667 |
| stress aggregate | 0.4000 | 15.4000 |
| planner_failure scenario | lower | 16 |
| planner_feedback_stress scenario | lower | 61 |

用途：

- 支撑 planner feedback/cooldown；
- 支撑 failure burst 作为指标。

注意：

- 如果没有 exact proposed scenario-level 数字，就只画 aggregate，scenario-level
  数字写在文字中。

### 图 5：ROS1/EGO Topic Chain 图

内容：

```text
/follow/fusion/target_world
/follow/lio/odom
        -> paper-line adapter
/follow/stage2/goal
        -> /move_base_simple/goal
        -> EGO
/follow/stage2/ego_position_cmd
        -> bridge
/follow/stage2/offboard/setpoint
        -> /mavros/setpoint_raw/local
```

用途：

- 证明软件链接入位置；
- 放在系统验证章节。

### 图 6：Target-Loss/Search 状态驻留时间图

数据：

| state | duration sec |
|---|---:|
| follow | 5.714 |
| predict_hold | 1.887 |
| search_safe_viewpoint | 1.081 |

用途：

- 说明 target-loss/search 状态在 ROS bag 中实际出现；
- 支撑“可执行状态链”，不是支撑“恢复一定成功”。

## 3. 推荐论文表述

可以写：

> In the diagnostic MATLAB suite, fixed safety margin caused substantial
> near-miss events, while the proposed dynamic safety filtering kept near-miss
> count at zero in the reported aggregate tables.

可以写：

> Planner command-health feedback and failed-candidate cooldown reduced
> consecutive infeasible-command bursts compared with the no-feedback ablation.

可以写：

> The Ubuntu20 ROS1/EGO validation confirms that the decision layer can be
> executed as an external Stage2 goal provider and that target-loss/search
> states are observable in bag-level metrics.

不要写：

> The proposed method outperforms all baselines.

不要写：

> The system guarantees collision-free UAV following.

不要写：

> Prediction is the key reason for performance improvement.

## 4. 后续画图顺序

优先级：

1. Near-miss 对比图；
2. Planner failure burst 对比图；
3. ROS1/EGO topic chain 图；
4. target-loss/search 状态驻留时间图；
5. 方法总流程图；
6. 动态安全边界示意图。

前两张最适合先做，因为它们直接支撑论文和专利的核心技术效果。
