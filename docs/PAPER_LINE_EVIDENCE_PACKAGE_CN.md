# Paper-Line 论文/专利证据包

状态：Ubuntu20 ROS1/Gazebo/RViz 验证后整理版，2026-06-01。

这份文件把已经完成的 MATLAB 诊断实验、Ubuntu20 ROS1/EGO 验证、
以及本地 `sim_plane` 的 Gazebo/RViz 受管仿真结果整理成论文、专利、
软件著作权可用的证据索引。

## 1. 当前一句话结论

当前证据支持的主线是：

> 面向无人机人跟随的上层跟随视点决策层，通过动态安全边界、硬安全过滤、
> planner command-health feedback 和失败候选 cooldown，降低近失风险和
> 连续不可行指令，并且该决策层已经能在 Ubuntu20 ROS1/EGO 软件链、
> `PX4 Gazebo Classic + MAVROS + Gazebo GUI + RViz` 受管仿真链路中运行。

不能说：

- 已经实机安全；
- 已经覆盖真实相机、真实 SLAM、硬件标定和实机飞行；
- Gazebo Classic 结果可直接外推到所有 Gazebo/Harmonic 环境；
- proposed 全面优于所有 baseline；
- prediction、occlusion score、visibility score、FSM recovery 是独立主贡献。

## 2. 已有证据来源

| 证据 | 文件 | 用途 |
|---|---|---|
| MATLAB 诊断实验 | `research/notes/2026-05-31_windows_matlab_refresh_log.md` | 支撑动态安全边界、硬安全过滤、planner feedback/cooldown |
| Ubuntu20 ROS1/EGO 验证 | `research/notes/2026-06-01_ubuntu20_ros1_validation_log.md` | 支撑 adapter 在 ROS1/EGO 软件链中可运行 |
| Ubuntu20 Gazebo/RViz 受管仿真 | `/home/coco/sim_plane/runs/px4_gazebo_classic_iris_human_follow_stage2_real_ego_20260531_180600_060153` 和 `/home/coco/sim_plane/runs/px4_gazebo_classic_iris_human_follow_stage2_real_ego_visual_20260531_180705_889048` | 支撑 `PX4 Gazebo Classic + MAVROS + Stage2 real-EGO + Gazebo GUI + RViz` 链路可运行 |
| 技术路线 | `docs/PAPER_LINE_ROUTE_BOOK_CN.md` | 定义论文和专利边界 |
| ROS1 接入说明 | `docs/PAPER_LINE_ROS1_STAGE2_ADAPTER.md` | 说明 adapter 接口、话题和验证入口 |
| 验证矩阵 | `docs/PAPER_LINE_UBUNTU20_VALIDATION_MATRIX.md` | 说明 Ubuntu20 验证流程和通过标准 |
| 专利准备 | `docs/PAPER_LINE_PATENT_PREP.md` | 说明专利结构、保护点和不应过度主张的内容 |

## 3. MATLAB 诊断证据

MATLAB 环境：

- MATLAB R2024b
- `tPaperLineCore`: 11 passed, 0 failed, 0 incomplete
- stage-one batch: 6 scenarios, 10 conditions, 5 seeds, 300 runs
- stress batch: 5 scenarios, 9 conditions, 5 seeds, 225 runs

### 3.1 Stage-One 总体表

| condition | visible ratio | loss duration | min clearance | near-miss | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| proposed | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 0.3333 | 0.1667 | 1.0000 |
| fixed_safety_margin | 0.9570 | 1.0367 | 1.2829 | 67.0333 | 0.3333 | 0.1667 | 0.1667 |
| no_planner_feedback | 0.9297 | 1.6933 | 1.6731 | 0.0000 | 2.6667 | 2.6667 | 0.8333 |
| no_prediction | 0.9411 | 1.4200 | 1.8872 | 0.0000 | 0.2333 | 0.1667 | 1.0000 |

结论：

- `fixed_safety_margin` 的 `nearMiss` 明显更高，说明固定安全边界不可靠。
- `no_planner_feedback` 的 planner failure 和 failure burst 更高，说明
  feedback/cooldown 有价值。
- `no_prediction` 并不差，不能把预测写成主贡献。

### 3.2 Stress 总体表

| condition | visible ratio | loss duration | min clearance | near-miss | planner failures | max failure burst | task success |
|---|---:|---:|---:|---:|---:|---:|---:|
| proposed | 0.8264 | 4.1840 | 1.5504 | 0.0000 | 1.4000 | 0.4000 | 0.8000 |
| fixed_safety_margin | 0.9650 | 0.8440 | 0.8458 | 107.8000 | 1.4000 | 0.4000 | 0.0000 |
| no_planner_feedback | 0.8551 | 3.4920 | 1.3177 | 0.0000 | 15.4000 | 15.4000 | 0.6000 |
| no_prediction | 0.9129 | 2.1000 | 1.6739 | 0.0000 | 1.1200 | 0.4000 | 1.0000 |

结论：

- `fixed_safety_margin` 在所有 stress 场景都产生 near-miss。
- `no_planner_feedback` 产生长连续失败，最高 `plannerFailureBurstMax=15.4`。
- `proposed` 不是全指标最好，所以论文要强调安全和连续失败抑制，不强调
  全面优越。

### 3.3 场景级诊断

`fixed_safety_margin` 产生 near-miss 的场景：

- stage-one:
  `obstacle_occlusion`, `occlusion_reacquire`, `planner_failure`,
  `short_loss`, `sudden_turn`
- stress:
  all five stress scenarios

`no_planner_feedback` 产生长连续失败的场景：

- stage-one `planner_failure`: `plannerFailureBurstMax_mean=16`
- stress `planner_blocked_goal`: `plannerFailureBurstMax_mean=16`
- stress `planner_feedback_stress`: `plannerFailureBurstMax_mean=61`

## 4. Ubuntu20 ROS1/EGO 软件链证据

运行环境：

- Ubuntu 20.04.6 LTS
- Python 3.8.10
- ROS Noetic
- commit `bc684c537198cd3ea77ea877bd9bb590e190476d`

### 4.1 构建和离线检查

命令：

```bash
SKIP_ROS=1 bash research/scripts/run_paper_line_ubuntu20_validation.sh
```

结果：PASS。

通过内容：

- Python syntax checks
- offline decision-core smoke
- offline scenario smoke
- offline baseline/ablation diagnostics
- `catkin_make`

### 4.2 单次 normal ROS1/EGO regression

结果：PASS。

artifact:

- `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002359`

关键数据：

- `distinct_goals=4`
- `distinct_cmds=3`
- `request_count=1`
- `final_plan_success`: `0=0`, `1=20`
- 关键 topic 均采样成功：
  `/follow/stage2/state`, `/follow/stage2/goal`,
  `/follow/stage2/debug_goal_path`, `/move_base_simple/goal`,
  `/waypoint_generator/waypoints`, `/planning/bspline`,
  `/follow/stage2/ego_position_cmd`, `/follow/stage2/offboard/setpoint`,
  `/mavros/setpoint_raw/local`

### 4.3 target-loss/search ROS1/EGO regression

结果：PASS。

artifact:

- `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002421`

关键数据：

- `distinct_goals=30`
- `distinct_cmds=22`
- `request_count=1`
- `final_plan_success`: `0=27`, `1=99`

解释：

- 这个场景触发了更难的搜索/恢复窗口；
- 它支持“状态序列能在 ROS1/EGO 软件链中执行”；
- 它不等于证明“复杂场景稳定恢复”。

### 4.4 repeated normal regression

命令：

```bash
bash research/scripts/run_paper_line_ros1_regression.sh 5
```

结果：5/5 PASS。

artifact:

- `.codex/artifacts/paper_line_ros1_adapter_20260601/formal_real_ego_regression_002516`

| run | distinct goals | distinct cmds | final plan success 0 | final plan success 1 | timeout |
|---:|---:|---:|---:|---:|---|
| 1 | 3 | 3 | 0 | 21 | false |
| 2 | 3 | 3 | 0 | 23 | false |
| 3 | 3 | 3 | 0 | 20 | false |
| 4 | 4 | 3 | 0 | 21 | false |
| 5 | 3 | 3 | 0 | 22 | false |

### 4.5 rosbag metrics

normal bag metrics：PASS。

- bag:
  `research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation.bag`
- metrics:
  `research/runs/stage2_rosbags/20260601_002655_normal_validation/normal_validation_metrics`

关键指标：

- `bag_duration_sec=11.4229`
- `goal_count=171`
- `ego_cmd_count=1133`
- `goal_rate_hz=15.0000`
- `ego_cmd_rate_hz=99.9999`
- `goal_max_gap_sec=0.0703`
- `ego_cmd_max_gap_sec=0.0101`
- `bridge_echo_distance_mean_m=0.0072`
- states seen: `follow`

target-loss/search bag metrics：PASS。

- bag:
  `research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation.bag`
- metrics:
  `research/runs/stage2_rosbags/20260601_002811_target_loss_validation/target_loss_validation_metrics`

关键指标：

- `bag_duration_sec=8.8151`
- `goal_count=127`
- `ego_cmd_count=873`
- `goal_rate_hz=14.4832`
- `ego_cmd_rate_hz=100.0000`
- `goal_max_gap_sec=0.1129`
- `ego_cmd_max_gap_sec=0.0101`
- `bridge_echo_distance_mean_m=0.0061`
- `search_count=16`
- state durations:
  `follow=5.714 sec`, `predict_hold=1.887 sec`,
  `search_safe_viewpoint=1.081 sec`

## 5. Ubuntu20 Gazebo/RViz 受管仿真证据

仿真平台：

- `/home/coco/sim_plane`
- backend: `px4_gazebo_classic`
- adapter: `human_follow_ros_stage2`
- managed workspace:
  `/home/coco/sim_plane_ws/workspaces/ros1_human_follow_stage1`

同步和构建：

- `python3 scripts/sync_human_follow_stage1_workspace.py --source-ws /home/coco/follower_paper_ws`
- `./scripts/build_human_follow_stage1_ws.sh`
- 结果：PASS。

fresh SIH 对照：

- artifact:
  `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_stage2_real_ego_20260531_175457_993127`
- latest acceptance：PASS。

Gazebo headless：

- scenario:
  `/home/coco/sim_plane/scenarios/px4_gazebo_classic_iris_human_follow_stage2_real_ego.json`
- artifact:
  `/home/coco/sim_plane/runs/px4_gazebo_classic_iris_human_follow_stage2_real_ego_20260531_180600_060153`
- result: `status=passed`
- key metrics:
  `ever_armed=true`,
  `algorithm_adapter_offboard_mode_reached=true`,
  `algorithm_adapter_stage2_real_ego_path_observed=true`,
  `algorithm_adapter_stage2_search_goal_observed=true`,
  `algorithm_adapter_stage2_nonzero_mavros_setpoint_count=82`

Gazebo GUI + RViz：

- scenario:
  `/home/coco/sim_plane/scenarios/px4_gazebo_classic_iris_human_follow_stage2_real_ego_visual.json`
- artifact:
  `/home/coco/sim_plane/runs/px4_gazebo_classic_iris_human_follow_stage2_real_ego_visual_20260531_180705_889048`
- result: `status=passed`
- key metrics:
  `gazebo_gui=true`,
  `world=warehouse`,
  `ever_armed=true`,
  `algorithm_adapter_offboard_mode_reached=true`,
  `algorithm_adapter_stage2_real_ego_path_observed=true`,
  `algorithm_adapter_stage2_search_goal_observed=true`,
  `algorithm_adapter_stage2_nonzero_mavros_setpoint_count=106`
- RViz 证据：
  adapter launch args 和 roslaunch command 均包含 `rviz:=true`。

边界：

- 这证明当前论文线能在本机
  `PX4 Gazebo Classic + MAVROS + Stage2 real-EGO + Gazebo GUI + RViz`
  受管链路中跑通。
- 两条 Gazebo run 都有 shutdown 阶段 PX4 残留 warning：
  `WARN  [commander] Connection to mission computer lost`。
  该 warning 发生在 ROS/MAVROS shutdown 附近，未阻止 `status=passed`，
  但不能把这两次 run 写成 `info-only`。
- visual run 还出现一次 shutdown 清理 warning：
  `forcing process kill` for `human_follow_stage2_integrated_chain`。
- 这还不是 detector、真实相机、真实 SLAM、硬件标定和实机飞行证据。

## 6. 论文可用表格建议

建议至少做四张表：

1. MATLAB stage-one ablation 总体表；
2. MATLAB stress ablation 总体表；
3. Ubuntu20 ROS1/EGO regression 通过表；
4. Ubuntu20 rosbag metrics 表；
5. Ubuntu20 Gazebo/RViz 受管仿真通过表。

建议至少做四类图：

1. 动态安全边界与 fixed safety margin 的 near-miss 对比；
2. planner feedback/cooldown 与 no feedback 的 failure burst 对比；
3. ROS1/EGO topic chain 示意图；
4. Gazebo/RViz 受管仿真链路图；
5. target-loss/search 状态驻留时间图。

## 7. 专利可用证据点

可以写进技术效果：

- 动态安全边界降低 near-miss；
- 硬安全过滤避免不安全候选靠评分“软通过”；
- planner feedback/cooldown 降低连续不可行指令；
- failure burst 比总失败次数更能反映连续失败风险；
- ROS1/EGO/Gazebo/RViz 验证说明方法可以作为上层 adapter 接入现有
  规划链和仿真链。

不要写成主权利要求：

- CV-KF 本身；
- 一般意义上的无人机人跟随；
- 一般意义上的目标重获取；
- 一般意义上的避障；
- EGO、PX4、SLAM、检测器内部实现；
- proposed 全面优于所有方案。

## 8. 当前下一步

不用继续重复跑 Windows/MATLAB、Ubuntu20 ROS1 验证或刚刚通过的
Gazebo/RViz 单次链路验证。

下一步优先做：

1. 论文方法章节；
2. 论文实验章节；
3. 专利技术交底书；
4. 图表生成脚本或手工表格；
5. 只有当具体论文/专利主张缺证据时，再补 Gazebo/RViz 场景矩阵、
   obstacle-near、
   planner-blocked、fixed-behind baseline、no-feedback ablation 等 ROS1 场景。
