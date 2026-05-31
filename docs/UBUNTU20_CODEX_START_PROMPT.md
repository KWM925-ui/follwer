# Ubuntu20 Codex Start Prompt

把下面整段发给 Ubuntu 20.04 + ROS1 那边的 Codex。不要手动操作；让那边的
Codex 自己拉代码、读文档、跑验证、记录结果。

```text
你现在接手 KWM925-ui/follwer 的 paper-line 分支。不要让我重新解释 Windows/MATLAB 和 Ubuntu 这边的长对话，请直接按仓库文档接手。

仓库：
- https://github.com/KWM925-ui/follwer.git
- 分支：paper-line

环境边界：
- 当前机器是 Ubuntu 20.04 + ROS1/catkin 目标运行环境。
- 这轮目标是 paper-line ROS1/EGO 软件链验证，不是 Windows/MATLAB 算法实验。
- MATLAB 不是运行环境依赖，不要试图安装 MATLAB。
- 不要改 ubuntu-mainline，不要改 EGO 内部、PX4、硬件标定、检测器、SLAM。
- 不要把项目迁移到 ROS2。
- 不要重开之前被否定的 Ubuntu 22.04/ROS2 本地临时仿真路线。

请先做：
1. 拉取远端最新代码：
   git fetch origin
   git checkout paper-line
   git pull --ff-only origin paper-line
2. 检查分支和工作区状态。
3. 先读这些文件，按这个顺序：
   - docs/PAPER_LINE_UBUNTU20_HANDOFF.md
   - docs/PAPER_LINE_UBUNTU20_VALIDATION_MATRIX.md
   - docs/PAPER_LINE_ROS1_STAGE2_ADAPTER.md
   - docs/PAPER_LINE_ROUTE_BOOK_CN.md
   - research/notes/2026-05-31_windows_matlab_refresh_log.md

当前已知结论：
- Windows/MATLAB 算法决策实验已经完成到 Seeds=1:5。
- 数据支持：动态安全边界、硬安全过滤、planner feedback/cooldown、failure burst 指标。
- 数据不支持把 prediction、occlusion score、visibility score、FSM recovery、proposed 全面优于 baseline 写成主贡献。
- 现在该验证的是 ROS1/EGO 软件链能否稳定运行和采集证据。

请按 docs/PAPER_LINE_UBUNTU20_VALIDATION_MATRIX.md 执行，不要中途只跑一点就停：
1. Phase 0：catkin build 和 Python 静态检查。
2. Phase 1：MATLAB-free offline diagnostics：
   python3 research/scripts/run_stage2_adapter_experiments.py --seeds 1:5
3. Phase 2：paper-line ROS1/EGO regression：
   roslaunch human_follow_bringup stage2_paper_line_real_ego_regression.launch
   roslaunch human_follow_bringup stage2_paper_line_search_real_ego_regression.launch
4. Phase 3：重复回归：
   bash research/scripts/run_paper_line_ros1_regression.sh 5
5. Phase 4：能跑通后记录 rosbag，并跑：
   bash research/scripts/record_stage2_validation_bag.sh
   python3 research/scripts/analyze_stage2_rosbag_metrics.py path/to/stage2_run.bag --validate

如果遇到失败：
- 先自己诊断和修复 paper-line 自己的 wrapper/launch/monitor/adapter 问题。
- 不要为了让测试过而修改 EGO 内部、PX4、硬件标定或 ubuntu-mainline。
- 保留失败日志、timeout reason、topic 缺失情况和复现命令。

请最终给我一份人话汇报：
- 先说 Ubuntu20 ROS1/EGO 验证通过没通过；
- 哪些 phase 通过，哪些失败；
- catkin build 是否成功；
- paper-line adapter 是否发布 /follow/stage2/state 和 /follow/stage2/goal；
- EGO 是否收到 /move_base_simple/goal；
- EGO 是否输出 /follow/stage2/ego_position_cmd；
- PX4 bridge/fake MAVROS 链路是否打通；
- target-loss/search 状态是否出现 follow、predict_hold、search_safe_viewpoint；
- repeated regression 是否 5/5 PASS；
- rosbag metrics 是否达标；
- 结果对论文/专利主张有什么影响；
- 需要提交的修复就提交并推送到 paper-line。
```

