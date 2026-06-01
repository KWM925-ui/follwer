# Paper-Line 论文结果表

状态：基于已完成证据整理，2026-06-01。

用途：把当前 MATLAB、Ubuntu20 ROS1/EGO、Gazebo/RViz 三条证据线整理成
论文实验章节可直接改写的结果表。本文只整理已有数字，不新增实验。

总边界：

- 可以支撑动态安全边界、硬安全过滤、planner feedback/cooldown、
  failure burst 指标和软件链可运行性；
- 不能支撑实机安全、真实相机、真实 SLAM、硬件标定、Gazebo Harmonic、
  proposed 全面优于所有 baseline。

## 表 1：MATLAB Stage-One Ablation 总体结果

数据来源：`research/notes/2026-05-31_windows_matlab_refresh_log.md`

| condition | visible ratio | loss duration | min clearance | near-miss | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| proposed | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 0.3333 | 0.1667 | 1.0000 |
| fixed_safety_margin | 0.9570 | 1.0367 | 1.2829 | 67.0333 | 0.3333 | 0.1667 | 0.1667 |
| no_planner_feedback | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 2.6667 | 2.6667 | 0.8333 |
| no_prediction | 0.9411 | 1.4200 | 1.8872 | 0.0000 | 0.2333 | 0.1667 | 1.0000 |

可写结论：

- `fixed_safety_margin` 的 near-miss 很高，动态安全边界有明确必要；
- `no_planner_feedback` 的 failure burst 更高，planner feedback/cooldown 有价值；
- `no_prediction` 不差，prediction 不能写成主贡献。

## 表 2：MATLAB Stress Ablation 总体结果

数据来源：`research/notes/2026-05-31_windows_matlab_refresh_log.md`

| condition | visible ratio | loss duration | min clearance | near-miss | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| proposed | 0.8264 | 4.1840 | 1.5504 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| fixed_safety_margin | 0.9650 | 0.8440 | 0.8458 | 107.8000 | 1.4000 | 0.4000 | 0.0000 |
| no_planner_feedback | 0.8551 | 3.4920 | 1.3177 | 0.0000 | 15.4000 | 15.4000 | 0.6000 |
| no_prediction | 0.9129 | 2.1000 | 1.6739 | 0.0000 | 1.1200 | 0.4000 | 1.0000 |

可写结论：

- stress 中 `fixed_safety_margin` 的 `nearMiss=107.8000`，说明固定安全边界风险更高；
- stress 中 `no_planner_feedback` 的 `plannerFailureBurstMax=15.4000`，说明连续不可行指令更严重；
- proposed 不是全指标最好，论文应强调安全和连续失败抑制，不强调全面优越。

## 表 3：Ubuntu20 ROS1/EGO Regression 通过结果

数据来源：`research/notes/2026-06-01_ubuntu20_ros1_validation_log.md`

| run type | result | artifact | distinct goals | distinct cmds | final plan success 0 | final plan success 1 | key observation |
|---|---|---|---:|---:|---:|---:|---|
| normal single | PASS | `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002359` | 4 | 3 | 0 | 20 | required topics sampled |
| target-loss/search single | PASS | `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002421` | 30 | 22 | 27 | 99 | `follow,predict_hold,search_safe_viewpoint` exercised |
| repeated normal | 5/5 PASS | `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002516` | 3-4 | 3 | 0 in all runs | 20-23 per run | no timeout counted |

可写结论：

- paper-line adapter 能进入 ROS1/EGO 软件链；
- target-loss/search 能触发更多 goal/cmd 和状态切换；
- repeated normal regression 证明正常链路不是一次性偶然通过。

## 表 4：Ubuntu20 Rosbag Metrics

数据来源：

- `research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation_metrics`
- `research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation_metrics`

| bag type | duration sec | goal count | ego cmd count | goal rate Hz | ego cmd rate Hz | goal max gap sec | ego cmd max gap sec | bridge echo mean m | states seen |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| normal | 11.4229 | 171 | 1133 | 15.0000 | 99.9999 | 0.0703 | 0.0101 | 0.0072 | `follow` |
| target-loss/search | 8.8151 | 127 | 873 | 14.4832 | 100.0000 | 0.1129 | 0.0101 | 0.0061 | `follow,predict_hold,search_safe_viewpoint` |

Target-loss/search 状态驻留时间：

| state | duration sec |
|---|---:|
| follow | 5.714 |
| predict_hold | 1.887 |
| search_safe_viewpoint | 1.081 |

可写结论：

- normal bag 中 goal 和 EGO command 连续稳定；
- target-loss/search bag 中状态链可观察；
- 该表证明软件链行为可记录，不证明复杂场景恢复一定成功。

## 表 5：Gazebo/RViz 受管仿真通过结果

数据来源：`research/notes/2026-06-01_ubuntu20_gazebo_rviz_validation_log.md`

| run type | backend | world | Gazebo GUI | RViz evidence | offboard reached | real-EGO path observed | search goal observed | nonzero MAVROS setpoints | result |
|---|---|---|---|---|---|---|---|---:|---|
| SIH control | `px4_sih` | n/a | no | no | true | true | true | 112 | PASS |
| Gazebo headless | `px4_gazebo_classic` | `empty` | false | no | true | true | true | 82 | PASS |
| Gazebo GUI + RViz | `px4_gazebo_classic` | `warehouse` | true | `rviz:=true` | true | true | true | 106 | PASS |

已知 warning：

- Gazebo headless 和 visual run 都有 shutdown 阶段
  `WARN [commander] Connection to mission computer lost`；
- visual run 还有一次 shutdown cleanup warning：
  `forcing process kill` for `human_follow_stage2_integrated_chain`；
- 这些 warning 没有阻止 `status=passed`，但论文不能写成完全无 warning。

画面证据边界：

- 当前 artifact 没有 `.png/.jpg/.mp4/.webm` 等 Gazebo/RViz 截图或录屏；
- `rviz:=true` 只能说明 RViz 被启动到链路里，不能当成“完整画面已保存”；
- 若需要答辩或论文附图，应补一次带截图/录屏保存的 visual-acceptance run。

可写结论：

- 当前 paper-line 能在本机 `PX4 Gazebo Classic + MAVROS + Stage2 real-EGO`
  受管链路中运行；
- visual run 支持 `Gazebo GUI + RViz` 已经实际接入；
- 该表不支持实机安全、真实传感器或 Gazebo Harmonic 外推。

## 图数据最小清单

### Near-Miss 对比图

| suite | proposed | fixed_safety_margin |
|---|---:|---:|
| stage-one | 0.0000 | 67.0333 |
| stress | 0.0000 | 107.8000 |

### Planner Failure Burst 对比图

| suite | proposed | no_planner_feedback |
|---|---:|---:|
| stage-one | 0.1667 | 2.6667 |
| stress | 0.4000 | 15.4000 |

### Target-Loss/Search 状态驻留图

| state | duration sec |
|---|---:|
| follow | 5.714 |
| predict_hold | 1.887 |
| search_safe_viewpoint | 1.081 |

## 推荐实验章节顺序

1. 先报 MATLAB stage-one/stress ablation，支撑动态安全边界和 planner feedback；
2. 再报 ROS1/EGO regression 和 rosbag metrics，证明 adapter 可进入软件链；
3. 最后报 Gazebo/RViz 受管仿真，证明链路能进入本机 PX4/Gazebo/RViz；
4. 讨论中主动承认 proposed 不是全指标最好，以及真实传感器/实机安全未验证。
