# Windows Paper-Line Handoff

状态：Windows/MATLAB 执行入口，2026-05-27。

这份文件给 Windows 端 Codex 直接使用。用户不需要重新解释 Ubuntu
这边的长对话；Windows 端先读本文件，再按下面顺序执行。

## 一句话结论

Ubuntu 这边的算法仿真准备已经做到可以交给 Windows/MATLAB 的程度。

下一步不是继续在 Ubuntu 上搭临时仿真，也不是现在做 Gazebo + RViz
严格全链路仿真。下一步是在 Windows/MATLAB 运行现有实验，拿到数据，
再判断哪些模块能写进论文和专利主线。

## 仓库和分支

- GitHub: `https://github.com/KWM925-ui/follwer.git`
- Windows 端必须使用分支：`paper-line`
- Ubuntu 当前工作区：`/home/coco/follower_paper_ws`
- 仓库历史文档里可能出现旧路径 `/home/coco/follwer_ws`，不要把它当成
  Windows 端必须复现的路径。

Windows 端建议先做：

```bash
git clone https://github.com/KWM925-ui/follwer.git
cd follwer
git checkout paper-line
git pull --ff-only origin paper-line
```

如果仓库已经存在，就进入仓库后执行：

```bash
git fetch origin
git checkout paper-line
git pull --ff-only origin paper-line
```

## 当前已经完成什么

已完成的，是“算法决策部分”的 MATLAB 实验准备，不是完整无人机系统已经
完成。

当前已有 MATLAB 入口：

- `research/matlab/runPaperLineDemo.m`
- `research/matlab/runPaperLineBatch.m`
- `research/matlab/runPaperLineStressBatch.m`
- `research/matlab/summarizePaperLineResults.m`
- `research/matlab/tests/tPaperLineCore.m`

当前已有核心函数：

- `research/matlab/+paperline/makeScenario.m`
- `research/matlab/+paperline/simulateRun.m`
- `research/matlab/+paperline/generateCandidates.m`
- `research/matlab/+paperline/filterCandidates.m`
- `research/matlab/+paperline/scoreCandidates.m`
- `research/matlab/+paperline/updateFsm.m`
- `research/matlab/+paperline/predictTarget.m`
- `research/matlab/+paperline/computeMetrics.m`

之前助手临时做过的 Ubuntu 本地算法仿真已经被用户否定，并且相关提交已经
回退。Windows 端不要重开那条路。

## 当前实验到底要验证什么

这轮 MATLAB 实验只验证上层算法决策，不验证机体、真实传感器、PX4、
Gazebo、RViz 或 EGO 内部优化。

主要回答三件事：

1. 动态安全边界有没有必要；
2. planner 反馈和失败候选 cooldown 有没有必要；
3. 目标丢失、遮挡、planner 失败时，上层决策是否比简单基线更稳。

当前最稳的论文/专利主线是：

- 动态安全边界；
- 硬安全过滤先于评分；
- planner command-health feedback；
- 失败候选 cooldown / blacklist；
- failure burst 指标。

现在不要把下面这些直接写成主贡献：

- “预测一定提升跟随效果”；
- “遮挡评分是独立创新”；
- “可见性评分是独立创新”；
- “恢复状态机是独立创新”；
- “proposed 全面优于所有 baseline”。

这些点可以保留为系统组成，但必须等实验结果支持后才能升级成论文/专利
主张。

## Windows 端先读哪些文件

按这个顺序读：

1. `docs/WINDOWS_PAPER_LINE_HANDOFF.md`
2. `docs/PAPER_LINE_MATLAB_EXPERIMENT_PLAN_CN.md`
3. `research/matlab/README.md`
4. `docs/PAPER_LINE_ROUTE_BOOK_CN.md`
5. `docs/PAPER_LINE_PATENT_PREP.md`

`FOLLOWER_ROUTE_MASTER.txt` 是整机路线背景，不是这轮 MATLAB 实验的直接
执行说明。需要了解总路线时再读。

## MATLAB 执行顺序

在 MATLAB 里进入：

```matlab
cd research/matlab
```

如果 MATLAB 当前不在仓库根目录，就用 Windows 的绝对路径进入
`research/matlab`。

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

第四步，如果 3-seed 没有明显异常，再跑 5-seed：

```matlab
batch5 = runPaperLineBatch(Seeds=1:5, SaveOutputs=true);
stress5 = runPaperLineStressBatch(Seeds=1:5, SaveOutputs=true);
```

第五步，打印关键对比：

```matlab
report = summarizePaperLineResults;
```

不要一上来就加大 seed。先确认 3-seed 方向稳定，再跑 5-seed。

## 应该拿回哪些结果

Windows 端跑完后，至少要把这些结果带回来：

- `runtests("tests")` 的通过/失败信息；
- `runPaperLineDemo`、`runPaperLineBatch`、`runPaperLineStressBatch` 是否报错；
- `summarizePaperLineResults` 打印的关键对比；
- 下面 CSV 的主要结论；
- 生成的图是否正常；
- 任何失败、反直觉、或者 proposed 不占优的场景。

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

这些生成文件默认被 `.gitignore` 忽略。先不要为了提交结果随便改
`.gitignore`。如果需要长期保存结果，先把关键结论写成小的 Markdown
记录，再决定是否挑选少量论文图另存。

## 怎么判断实验结果

动态安全边界重点比较：

- `proposed`
- `fixed_safety_margin`

如果 `fixed_safety_margin` 的 `nearMissCount` 明显更高，或
`minClearance` 明显更低，动态安全边界就可以作为主贡献。

planner 反馈和 cooldown 重点比较：

- `proposed`
- `no_planner_feedback`

如果 `no_planner_feedback` 的 `plannerFailureBurstMax` 明显更大，
planner feedback 和 failed-candidate cooldown 就可以作为主贡献。

预测重点比较：

- `proposed`
- `no_prediction`
- `ca_kf`

如果 `no_prediction` 不差，甚至更好，就不要把预测写成主贡献。预测最多
写成给动态安全边界提供不确定性。

遮挡评分、可见性评分、恢复状态机重点比较：

- `proposed`
- `no_occlusion_score`
- `no_visibility_score`
- `no_recovery_fsm`

如果差异不明显，就只写成工程组成，不写成独立创新。

## 和 Gazebo/RViz 的关系

Gazebo + RViz 严格全链路仿真是后面的系统级验证。

当前顺序是：

1. 先把 MATLAB 算法决策实验跑完；
2. 根据结果收紧论文和专利主线；
3. 必要时补 MATLAB 图表和统计；
4. 再回 Ubuntu 做 Gazebo + RViz + EGO + 跟踪的严格全链路验证。

不要把第 4 步提前到当前 MATLAB 结果之前。

## Windows Codex 启动词

到 Windows 后，可以直接把下面这段发给 Codex：

```text
你现在接手 follwer 项目的 paper-line 分支。请先拉取远端最新代码，不要让我重复解释 Ubuntu 这边的长对话。

仓库：
- https://github.com/KWM925-ui/follwer.git
- 分支：paper-line

请先读：
- docs/WINDOWS_PAPER_LINE_HANDOFF.md
- docs/PAPER_LINE_MATLAB_EXPERIMENT_PLAN_CN.md
- research/matlab/README.md
- docs/PAPER_LINE_ROUTE_BOOK_CN.md

边界：
- 这轮只做 Windows/MATLAB 的算法决策实验。
- 不改 Ubuntu 主线、硬件标定、PX4、Gazebo、RViz、EGO 内部代码。
- 不重开之前被否定的 Ubuntu 本地临时仿真。
- 生成的 CSV 和图片默认不要提交，先汇总结果给我看。

请按文档顺序执行：
1. 检查当前分支和工作区状态；
2. 在 MATLAB 进入 research/matlab；
3. 依次运行 runtests("tests")、runPaperLineDemo、runPaperLineBatch、runPaperLineStressBatch；
4. 先跑 Seeds=1，再跑 Seeds=1:3，稳定后再跑 Seeds=1:5；
5. 运行 report = summarizePaperLineResults；
6. 把测试结果、CSV 摘要、生成图路径、异常场景、以及哪些论文/专利主张被数据支持讲清楚。

说人话汇报：先告诉我通过没通过，再告诉我哪些结果支持动态安全边界、planner feedback/cooldown，哪些结果不支持预测或其他模块作为主贡献。
```
