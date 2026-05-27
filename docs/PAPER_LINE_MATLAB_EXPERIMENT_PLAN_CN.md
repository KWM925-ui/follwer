# Paper-Line MATLAB 实验执行计划

状态：Windows/MATLAB 前最终准备版，2026-05-27。

这份文档只管“算法决策部分”的仿真，不管 Gazebo、机体、真实传感器、
PX4 实机、EGO 内部优化。

## 1. 这轮实验要回答什么

这轮实验不是为了证明“整个无人机系统已经能飞”。

它只回答三件事：

1. 动态安全边界有没有必要；
2. planner 反馈和失败候选冷却有没有必要；
3. 目标丢失、遮挡、规划失败时，上层决策是否比简单基线更稳。

如果实验结果不支持某个模块，就不要把它写成论文或专利主贡献。

## 2. 当前最稳的论文/专利主线

当前最稳的主线是：

> 面向无人机人跟随的上层跟随视点决策方法，通过动态安全边界、
> 硬安全过滤、planner 反馈和失败候选冷却，降低近失风险和重复不可行
> 指令。

现在不要主打：

- “预测一定提升跟随效果”；
- “遮挡评分是独立创新”；
- “可见性评分是独立创新”；
- “恢复状态机是独立创新”；
- “proposed 全面优于所有 baseline”。

这些点可以保留为工程组成，但只有实验明显支持时才能升级为主贡献。

## 3. 已有 MATLAB 代码

入口：

- `research/matlab/runPaperLineBatch.m`
- `research/matlab/runPaperLineStressBatch.m`
- `research/matlab/runPaperLineDemo.m`
- `research/matlab/summarizePaperLineResults.m`
- `research/matlab/tests/tPaperLineCore.m`

核心函数：

- `+paperline/makeScenario.m`
- `+paperline/simulateRun.m`
- `+paperline/computeMetrics.m`
- `+paperline/generateCandidates.m`
- `+paperline/filterCandidates.m`
- `+paperline/scoreCandidates.m`
- `+paperline/updateFsm.m`
- `+paperline/predictTarget.m`

## 4. 普通场景

`runPaperLineBatch.m` 当前包含：

| 场景 | 目的 |
|---|---|
| `straight` | 正常跟随 |
| `sudden_turn` | 人突然转向 |
| `obstacle_occlusion` | 障碍遮挡 |
| `short_loss` | 短时目标丢失 |
| `occlusion_reacquire` | 遮挡后重获取 |
| `planner_failure` | 下游 planner 失败 |

## 5. 压力场景

`runPaperLineStressBatch.m` 当前包含：

| 场景 | 目的 |
|---|---|
| `fixed_behind_occlusion` | 固定后方跟随容易被遮挡 |
| `nearest_visibility_trap` | 最近可行点可能可见性差 |
| `recovery_corner_loss` | 转角丢失和恢复 |
| `planner_feedback_stress` | 长时间 planner 失败压力 |
| `planner_blocked_goal` | 目标点被 planner 拒绝 |

## 6. 对比方法

普通 batch 当前包含：

| 方法 | 含义 |
|---|---|
| `proposed` | 当前完整方法 |
| `fixed_behind` | 固定在人后方跟随 |
| `nearest_feasible` | 选最近可行候选点 |
| `no_prediction` | 不用预测点 |
| `fixed_safety_margin` | 固定安全边界 |
| `no_recovery_fsm` | 去掉恢复状态机 |
| `ca_kf` | 改用 CA-KF 预测 |
| `no_occlusion_score` | 去掉遮挡评分 |
| `no_visibility_score` | 去掉可见性评分 |
| `no_planner_feedback` | 去掉 planner 反馈和冷却 |

压力 batch 当前不包含 `ca_kf`，其余主要方法都有。

## 7. 指标

必须重点看：

| 指标 | 意义 |
|---|---|
| `visibilityRatio` | 目标可见比例 |
| `lossDuration` | 目标丢失总时间 |
| `meanReacquisitionTime` | 平均重获取时间 |
| `minClearance` | 最小障碍距离 |
| `nearMissCount` | 近失次数 |
| `plannerFailureCount` | planner 失败次数 |
| `plannerFailureBurstMax` | 最大连续 planner 失败长度 |
| `taskSuccess` | 综合成功率 |

辅助看：

- `meanDistanceError`
- `meanViewAngleError`
- `candidateSwitchCount`
- `stateSwitchCount`
- `blacklistActivationCount`
- `meanPredictionError`

## 8. Windows/MATLAB 执行顺序

先进入仓库：

```matlab
cd research/matlab
```

第一步，跑单元测试：

```matlab
runtests("tests")
```

第二步，跑最小 smoke：

```matlab
demo = runPaperLineDemo(ShowFigures=false);
batch1 = runPaperLineBatch(Seeds=1, SaveOutputs=false);
stress1 = runPaperLineStressBatch(Seeds=1, SaveOutputs=false);
```

第三步，跑正式小批量：

```matlab
batch3 = runPaperLineBatch(Seeds=1:3, SaveOutputs=true);
stress3 = runPaperLineStressBatch(Seeds=1:3, SaveOutputs=true);
```

第四步，如果小批量结果正常，再跑正式 5-seed：

```matlab
batch5 = runPaperLineBatch(Seeds=1:5, SaveOutputs=true);
stress5 = runPaperLineStressBatch(Seeds=1:5, SaveOutputs=true);
```

不要一上来就跑更大的 seed。先看 3-seed 是否方向稳定。

第五步，打印关键对比：

```matlab
report = summarizePaperLineResults;
```

这个脚本会读已经导出的 CSV，重点打印：

- `proposed` 对 `fixed_safety_margin`；
- `proposed` 对 `no_planner_feedback`；
- `proposed` 对 `no_prediction`。

## 9. 输出文件

普通 batch 输出：

- `research/outputs/paper_line_batch/stage1_runs.csv`
- `research/outputs/paper_line_batch/stage1_by_condition.csv`
- `research/outputs/paper_line_batch/stage1_by_scenario_condition.csv`
- `research/figures_generated/paper_line_batch/stage1_condition_summary.png`

压力 batch 输出：

- `research/outputs/paper_line_stress/stress_runs.csv`
- `research/outputs/paper_line_stress/stress_by_condition.csv`
- `research/outputs/paper_line_stress/stress_by_scenario_condition.csv`
- `research/figures_generated/paper_line_stress/stress_condition_summary.png`

这些生成文件默认不提交。先看结果，再决定是否挑选少量论文图另存。

## 10. 怎么判断结果

### 动态安全边界

重点比较：

- `proposed`
- `fixed_safety_margin`

如果 `fixed_safety_margin` 的 `nearMissCount` 明显更高，或
`minClearance` 明显更低，说明动态安全边界值得作为主贡献。

### planner 反馈和冷却

重点比较：

- `proposed`
- `no_planner_feedback`

如果 `no_planner_feedback` 的 `plannerFailureBurstMax` 明显更大，说明
planner 反馈和失败候选冷却值得作为主贡献。

### 预测

重点比较：

- `proposed`
- `no_prediction`
- `ca_kf`

如果 `no_prediction` 不差，甚至更好，就不要把预测写成主贡献。
预测最多写成提供不确定性给安全边界。

### 遮挡评分、可见性评分、恢复状态机

重点比较：

- `proposed`
- `no_occlusion_score`
- `no_visibility_score`
- `no_recovery_fsm`

如果差异不明显，就只写成工程组成，不写成独立创新。

## 11. 论文表格建议

最少需要三张表：

1. 普通场景总体表：按 condition 汇总；
2. 压力场景总体表：按 condition 汇总；
3. 关键场景表：只列 safety margin 和 planner feedback 相关场景。

最少需要四类图：

1. target/UAV 轨迹图；
2. 候选跟随点选择图；
3. 状态变化曲线；
4. planner failure burst 对比图。

当前 MATLAB 代码已经能生成条件汇总图。轨迹图和状态图如果不够，需要在
MATLAB 端补，不在 Ubuntu 端补。

## 12. 专利材料该怎么用这些实验

可以支撑的专利点：

- 动态安全边界；
- 硬安全过滤先于评分；
- planner command-health feedback；
- 失败候选 cooldown；
- 使用 failure burst 评价连续失败。

暂时不要写成主权利要求的点：

- CV-KF 本身；
- generic 人跟随；
- generic 目标重获取；
- generic 遮挡感知；
- generic 避障。

## 13. 去 Windows 前 Ubuntu 还剩什么

Ubuntu 本地这条算法仿真准备线只剩：

1. 确认这份实验计划；
2. 确认 MATLAB 代码文件齐全；
3. 确认没有未提交改动；
4. 把本计划推到 `paper-line`。

做完这些，就该去 Windows/MATLAB 跑实验。

Gazebo + RViz 严格全链路仿真是后续系统级验证，不属于这轮算法仿真的前置阻塞。

Windows 端入口文档是 `docs/WINDOWS_PAPER_LINE_HANDOFF.md`。到 Windows 后先让
Codex 读那份文件，再按本文件第 8 节的 MATLAB 顺序执行。
