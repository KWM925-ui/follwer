# Human Follow Collaboration Ledger

更新时间：2026-05-10 Asia/Shanghai

## 作用

这份文件是 `/home/coco/follwer_ws` 与 `/home/coco/sim_plane` 之间的异步协作面。

目标不是重复写项目总览，而是防止两边会话在以下问题上反复漂移：

- 把 `human_follow_user` 误当成当前主线算法入口
- 把旧的 Stage2 placeholder 证据误当成真实 EGO 证据
- 把 `sim_plane` 独立基线误当成当前项目进度
- 把已经证明过的接口问题重新当成阻塞点

## 当前边界

- 本文件只服务于 human-follow 仿真支线。
- 不讨论实机标定、真机雷达、真机相机、真机飞控联调。
- 项目源码主真源始终是：
  - `/home/coco/follwer_ws`
- 平台受管仿真包装始终是：
  - `/home/coco/sim_plane`

## 当前最高目标

- 旧的 Stage2 managed real-EGO acceptance 已经封装完成，不再是当前最高目标。
- 当前新的最高目标已锁定为：
  - 先完成 Stage1 完整系统仿真
  - 先站住 truth-to-control + PX4 managed acceptance
  - 再升级到 detector/tracker-in-loop
  - 最后才把 EGO 接回完整系统线
- 详细前沿文件：
  - `/home/coco/follwer_ws/STAGE1_FULL_SYSTEM_SIM_FRONTIER.txt`

## 当前项目侧新进展

- 项目侧旧 Stage1 truth-to-control 基线已 fresh PASS：
  - `stage1_truth_fusion_controller_regression.launch`
- 项目侧新的 detector/tracker-in-loop 本地回归也已落地并 PASS：
  - `stage1_truth_detector_tracker_controller_regression.launch`
- 这条新 detector/tracker-in-loop 本地链已经是：
  - synthetic image
  - detector
  - tracker
  - fusion
  - controller
  - px4 bridge
- 这条 detector/tracker-in-loop 本地链已经做过更严格的二次验证：
  - detector 必须发出有效检测
  - detector source 必须带 `synthetic_box`
  - tracker source 也必须带 `synthetic_box`
- 但它现在还只是 project-side 本地证据，不是 sim-plane managed acceptance。
- 项目侧又补了一轮 2026-05-10 的左右横移本地核验：
  - `lateral_left_track` 本地 case validation PASS
  - `lateral_right_track` 本地 case validation PASS
  - 左右两边的 `cmd_lateral` 和 `dy` 都按预期对称成立
  - 所以当前没有证据支持“follwer_ws 里的 Stage1 横移逻辑已经坏掉”

## 当前 sim-plane 新进展

- `sim_plane` 已经 fresh rerun 了 Stage1 truth-to-control managed proof、七行为矩阵、latest acceptance。
- 这次 latest acceptance 报告路径是：
  - `/home/coco/sim_plane/runs/human_follow_stage1_acceptance/human_follow_stage1_acceptance_latest_20260509_160731_368057/report.json`
- 这次状态不是 full-chain 失败，而是 acceptance 失败：
  - `status=failed`
  - 唯一 issue：
    - `metric max_altitude_m increased by 0.287 beyond allowed 0.15`
  - 最早失败 scenario：
    - `px4_sih_quadx_human_follow_case_lateral_left_track`
- 这个 failing scenario 自己的 artifact result 仍然是：
  - `status=passed`
  - `algorithm_adapter_completed_successfully=true`
  - `algorithm_adapter_follow_valid_command_seen=true`
- 当前更像 acceptance/reference 问题，而不是项目算法崩掉：
  - 当前 left `max_altitude_m=0.349`
  - 当前 right `max_altitude_m=0.318`
  - reference right `max_altitude_m=0.256`
  - 只有 reference left `max_altitude_m=0.062` 显著偏离其余三者
- 后续 `sim_plane` 已在自己侧修过 reference，并拿到了一次新的 managed PASS：
  - report:
    - `/home/coco/sim_plane/runs/human_follow_stage1_acceptance/human_follow_stage1_acceptance_latest_20260509_161825_507929/report.json`
  - status:
    - `passed`
- 但这条线还没有稳定收口，因为它后面又出现了新的 rerun 失败：
  - later report:
    - `/home/coco/sim_plane/runs/human_follow_stage1_acceptance/human_follow_stage1_acceptance_latest_20260509_164535_325604/report.json`
  - earliest failing scenario:
    - `px4_sih_quadx_human_follow_case_lateral_left_track`
  - failing artifact:
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_case_lateral_left_track_20260509_164441`
  - failure note:
    - `The algorithm adapter failed before completion: Stage1 ROS follow probe failed at wait_ready`
- 这个新失败的性质已经变了：
  - 不再是 reference artifact 问题
  - 也不是项目侧控制逻辑坏掉
  - 而是 sim-plane 受管启动稳定性问题
  - 具体表现：
    - `follow_valid_command_seen=true`
    - `setpoint_count=22`
    - `nonzero_setpoint_count=22`
    - 但 warmup 阈值要求 `25`
    - 所以 probe 没进入 OFFBOARD 请求阶段
    - `offboard_requested=false`
    - `arm_command_sent=false`
    - `current_mode=AUTO.LOITER`
- detector/tracker managed full-chain 也出现过同类现象：
  - first fail:
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_detector_tracker_full_chain_20260509_162838`
    - first failing point: `bridge`
    - 实际 failure class 仍然是 `wait_ready`
  - later pass:
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_detector_tracker_full_chain_20260509_163202`

## 当前唯一有效判断

- 现在不要再把 Stage1 的问题描述成“reference 还没修好”。
- reference 问题已经被 sim-plane 自己修过，并出现过 PASS 证据。
- 现在的剩余前沿是：
  - sim-plane 受管 `MAVROS + probe + OFFBOARD warmup` 启动稳定性
  - 不是 `/home/coco/follwer_ws` 算法逻辑
- 所以当前需要的是：
  - 只在 `sim_plane` 内收敛 `wait_ready` 偶发失败
  - 用重复 rerun 证明稳定
  - 而不是再拿一次 lucky PASS 就算结束

## 对 2026-05-09 晚上的最新“转绿”审计结论

- `sim_plane` 确实拿到了新的 latest acceptance PASS：
  - `/home/coco/sim_plane/runs/human_follow_stage1_acceptance/human_follow_stage1_acceptance_latest_20260509_183513_866342/report.json`
- 但这次 PASS 不是一句“启动抖动已经真正修稳”就能概括的。
- fresh 审计确认它至少包含两类 acceptance 语义变化：
  - 七个 Stage1 scenario 全部改成了：
    - `allow_early_stop_on_adapter_success=true`
  - Stage1 latest-vs-reference regression budget 删掉了：
    - `telemetry_count`
    - `mode_changes`
    - `algorithm_adapter_follow_non_hold_count`
- 当前 `human_follow_stage1_acceptance_matrix.json` 的状态是：
  - latest-vs-reference 只保留：
    - `max_altitude_m`
    - `max_speed_mps`
  - 但下面这些仍然保留为单次运行阈值：
    - `telemetry_count min`
    - `mode_changes min`
    - `algorithm_adapter_follow_non_hold_count min`
- 这意味着：
  - 核心行为门槛没有被全部拿掉
  - 但“和旧 reference 的逐项最新对比”被明显收窄了
- 还有一个关键语义变化来自 backend 本身：
  - `sim_plane/sim_plane/backends/px4_sih.py`
  - 当 `allow_early_stop_on_adapter_success=true` 时，
  - 一旦 adapter thread 成功退出，telemetry 收集就提前结束
- 所以这批新 artifact 中天然会看到：
  - `telemetry_count` 更低
  - `mode_changes` 更少
  - 这两项不再适合和旧的“跑满时长” reference 直接做 latest-vs-reference 比较
- 证据链是连续的：
  - `...182041` 还是失败：
    - 还在报 `max_speed_mps`、`max_altitude_m`、`follow_non_hold_count`
  - `...183047` 失败：
    - 报 `max_speed_mps`、`max_altitude_m`
  - `...183353` 失败：
    - 只剩 `telemetry_count`、`mode_changes` regression
  - `...183513` 通过：
    - 说明最后这一步“转绿”确实与去掉这两类 regression gate 有直接关系
- 因此当前最准确的说法不是：
  - “sim-plane 已经把 Stage1 启动稳定性彻底修好了”
- 而应该是：
  - “sim-plane 把 Stage1 acceptance 收敛到了 early-stop packaging 语义，并在这个新语义下得到了一次 PASS”
- 这件事本身不一定错误，因为：
  - early-stop 后，旧 reference 的 telemetry 长度类指标本来就不可比
- 但它也绝对不等于：
  - 已经完成了重复稳定性证明
- 所以下一前沿必须收紧成：
  - 先冻结现在这版 Stage1 acceptance 语义
  - 不再继续改 matrix/scenario
  - 在这版冻结语义下做重复 rerun 稳定性证明

## 冻结 contract 下的重复稳定性结论

- 这一步现在已经完成，不需要继续怀疑式空转。
- `sim_plane` 最新回合声明的 5+5+7+acceptance，我已做了交叉核验，和 artifact 一致：
  - latest acceptance：
    - `/home/coco/sim_plane/runs/human_follow_stage1_acceptance/human_follow_stage1_acceptance_latest_20260509_185801_710595/report.json`
    - `status=passed`
  - `lateral_left_track` 连续 PASS 证据：
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_case_lateral_left_track_20260509_185354`
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_case_lateral_left_track_20260509_185407`
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_case_lateral_left_track_20260509_185421`
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_case_lateral_left_track_20260509_185434`
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_case_lateral_left_track_20260509_185447`
  - `detector_tracker_full_chain` 连续 PASS 证据：
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_detector_tracker_full_chain_20260509_185511`
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_detector_tracker_full_chain_20260509_185543`
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_detector_tracker_full_chain_20260509_185616`
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_detector_tracker_full_chain_20260509_185649`
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_detector_tracker_full_chain_20260509_185722`
- 我核验到的关键点是：
  - 这些重复运行都不是“侥幸 PASS 后缺关键指标”的假绿
  - `algorithm_adapter_completed_successfully=true`
  - `algorithm_adapter_offboard_requested=true`
  - `algorithm_adapter_arm_command_sent=true`
  - 最新 acceptance 也是 `issues=[]`
- 因此当前最准确的结论变成：
  - 在冻结 `183513` 这版 early-stop contract 后，
  - Stage1 managed startup 稳定性已经被重复证明
  - 当前没有证据支持“wait_ready / warmup / launch-exit / MAVROS disconnect 仍是活动阻塞”
- 这不表示“语义变化不存在”，而是表示：
  - 这版语义已经审过
  - 在这版语义下，Stage1 managed 现在可以视为稳定收口
- 所以下一前沿不再是继续反复跑 Stage1 startup stability，
- 而应该切到：
  - detector/tracker-in-loop 的受管 acceptance formalization / packaging
  - 同时保留现有 truth-driven Stage1 matrix 作为 baseline

## detector/tracker-in-loop 受管 acceptance 已正式收口

- 这一步也已经不是“口头说做了”，而是有完整落地物：
  - acceptance matrix：
    - `/home/coco/sim_plane/configs/human_follow_stage1_detector_tracker_acceptance_matrix.json`
  - acceptance 实现：
    - `/home/coco/sim_plane/sim_plane/human_follow_stage1_detector_tracker_acceptance.py`
  - CLI 入口：
    - `python3 -m sim_plane human-follow-stage1-detector-tracker-acceptance --latest --artifact-root runs --json`
- fresh artifact/report 也已经闭环：
  - full-chain artifact：
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_detector_tracker_full_chain_20260509_191613`
  - acceptance report：
    - `/home/coco/sim_plane/runs/human_follow_stage1_detector_tracker_acceptance/human_follow_stage1_detector_tracker_acceptance_latest_20260509_191701_296089/report.json`
  - `status=passed`
- 我核验到的关键点：
  - 不是复用 truth-driven Stage1 acceptance 名字硬塞进去
  - 是独立的 matrix、独立的 report root、独立的 CLI command
  - 这条 surface 明确要求 launch 名称保持为：
    - `stage1_truth_detector_tracker_controller_regression.launch`
  - report 中核心 managed gate 都是成立的：
    - `algorithm_adapter_completed_successfully=true`
    - `algorithm_adapter_offboard_requested=true`
    - `algorithm_adapter_offboard_mode_reached=true`
    - `algorithm_adapter_arm_command_sent=true`
    - `algorithm_adapter_follow_valid_command_seen=true`
- 因此到当前为止，仿真支线里与 Stage1 相关的两层都已经收口：
 - 因此到当前为止，仿真支线里与 Stage1 相关的两层都已经收口：
  - truth-driven Stage1 managed acceptance：有独立 matrix/report，且稳定
  - detector/tracker-in-loop Stage1 managed acceptance：也有独立 matrix/report，且已通过
- 所以当前正确表述已经变成：
  - Stage1 simulation branch 可以视为完成
  - 下一前沿不再是 Stage1 内部继续修补
  - 而是要么 widen 到 Stage2/EGO integrated simulation acceptance
  - 要么回到硬件主线

## Stage2 / EGO integrated acceptance 也已经收口

- 这一步现在也不是“准备开始”，而是已经落地并通过了：
  - integrated scenario：
    - `/home/coco/sim_plane/scenarios/px4_sih_quadx_human_follow_stage2_real_ego_integrated.json`
  - integrated acceptance matrix：
    - `/home/coco/sim_plane/configs/human_follow_stage2_integrated_acceptance_matrix.json`
  - integrated acceptance 实现：
    - `/home/coco/sim_plane/sim_plane/human_follow_stage2_integrated_acceptance.py`
  - integrated adapter：
    - `/home/coco/sim_plane/sim_plane/adapters/human_follow_ros_stage2.py`
  - integrated CLI：
    - `python3 -m sim_plane human-follow-stage2-integrated-acceptance --latest --artifact-root runs --json`
- fresh artifact/report：
  - integrated artifact：
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_stage2_real_ego_integrated_20260509_200452`
  - latest acceptance report：
    - `/home/coco/sim_plane/runs/human_follow_stage2_integrated_acceptance/human_follow_stage2_integrated_acceptance_latest_20260509_200559_527102/report.json`
  - `status=passed`
- 我做过的关键核验：
  - 这条线不是 placeholder 伪装
  - report 中明确有：
    - `algorithm_adapter_stage2_variant=real_ego`
    - `algorithm_adapter_stage2_search_goal_observed=true`
    - `algorithm_adapter_stage2_lost_hold_observed=true`
    - `algorithm_adapter_stage2_real_ego_path_observed=true`
    - `algorithm_adapter_stage2_waypoint_count=30`
    - `algorithm_adapter_stage2_distinct_goal_count=13`
    - `algorithm_adapter_stage2_distinct_ego_cmd_count=16`
  - notes 里也明确写了：
    - rolling follow goals
    - search goals after target loss
    - lost/hold behavior after search timeout
    - real EGO waypoint/path generation
    - not a sim-plane independent ego baseline
- 因此当前最准确的整体结论已经变成：
  - 仿真支线不只是 Stage1 完成
  - 连 Stage2 / real-EGO integrated acceptance 也已经正式通过
  - 所以“这个完整系统的仿真做全做好”这一条，到当前证据下已经功能性收口

## 当前唯一剩余噪声

- 这条噪声现在也已经清掉了：
  - probe 已重命名为：
    - `/home/coco/sim_plane/scripts/ros_stage2_integrated_probe.py`
  - `sim_plane/adapters/human_follow_ros_stage2.py` 的引用也已切过去
  - rename 后最小复验仍为 PASS
- 相关 fresh 证据：
  - scenario artifact：
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_stage2_real_ego_integrated_20260509_201915`
  - latest acceptance pointer：
    - `/home/coco/sim_plane/runs/human_follow_stage2_integrated_acceptance/latest_latest.json`
  - latest acceptance report dir：
    - `/home/coco/sim_plane/runs/human_follow_stage2_integrated_acceptance/human_follow_stage2_integrated_acceptance_latest_20260509_201915_939776`
  - `status=passed`
- 因此现在没有剩余功能性阻塞，也没有剩余必须处理的 hygiene 阻塞。
- 这条仿真支线可以正式封口。

## 当前结论

- 到当前证据为止，这条 human-follow 仿真支线已经全部收口：
  - Stage1 truth-driven managed acceptance：完成
  - Stage1 detector/tracker-in-loop managed acceptance：完成
  - Stage2 / real-EGO integrated managed acceptance：完成
  - Stage2 probe hygiene rename：完成
- 因此下一步不再是继续扩仿真，而是：
  - 回到硬件主线
  - 或者只在未来出现 fresh failing artifact 时再重开仿真支线

## 锁定事实

- `human_follow_user` 目前只是用户后续自定义算法插槽，不是当前项目主线算法入口。
- 当前项目侧 Stage2 真实入口是：
  - `/home/coco/follwer_ws/src/human_follow_bringup/launch/stage2_real_ego.launch`
- 当前项目侧 Stage2 已经不是“只有 follow 没有 search”：
  - follow 滚动目标已实现
  - target 丢失后的 search 目标已实现
  - search 超时后的 lost/hold 行为已实现
- 当前项目侧 Stage2 本地真实 EGO 链已经有通过证据：
  - `/home/coco/follwer_ws/src/human_follow_bringup/launch/stage2_search_goal_regression.launch`
  - `/home/coco/follwer_ws/src/human_follow_bringup/launch/stage2_real_ego_full_chain_regression.launch`
- `sim_plane` 侧也已经存在受管 Stage2 real-ego 证据，且不是 placeholder：
  - managed launch:
    - `/home/coco/sim_plane/sim_plane/ros/human_follow_stage2_real_ego_managed.launch`
  - scenario:
    - `/home/coco/sim_plane/scenarios/px4_sih_quadx_human_follow_stage2_real_ego.json`
  - reference artifact:
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_stage2_real_ego_20260508_062640/result.json`
  - latest refreshed artifact:
    - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_stage2_real_ego_20260508_102052/result.json`
  - latest acceptance report:
    - `/home/coco/sim_plane/runs/human_follow_stage2_acceptance/human_follow_stage2_acceptance_latest_20260508_102121_077539/report.json`
- 上述 managed Stage2 artifact 已经明确记录：
  - `algorithm_adapter_stage2_search_goal_observed=true`
  - `algorithm_adapter_stage2_real_ego_path_observed=true`
  - `algorithm_adapter_stage2_waypoint_count=14`
  - `algorithm_adapter_stage2_distinct_goal_count=6`
  - `algorithm_adapter_stage2_distinct_ego_cmd_count=10`
- 上述 latest acceptance report 已明确记录：
  - `status=passed`
  - `selection_mode=latest`
  - `reference_artifact_dir=/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_stage2_real_ego_20260508_062640`
  - `artifact_dir=runs/px4_sih_quadx_human_follow_stage2_real_ego_20260508_102052`
  - `issues=[]`
- `sim_plane` 中间会话断过一次，但它最后一轮已经给出了完成态结论；该结论的来源会话是：
  - `/home/coco/.codex/sessions/2026/04/27/rollout-2026-04-27T18-00-51-019dce62-7308-7b21-8782-a5ad52531cee.jsonl`

## 已排除分支

- 不要再把 “Stage2 search 还没接进去” 当成当前事实。
- 不要再把 “managed sim 还停留在 placeholder” 当成当前事实。
- 不要再追问 “你的真实算法到底在 human_follow_user 哪个文件里”，这不是当前主线。
- 不要把 `sim_plane` 里独立的 `ego_planner*` 基线当作当前项目融合完成证据。

## topic 主链

项目侧 Stage2 真实主链当前应被理解为：

- 输入：
  - `/follow/fusion/target_world`
  - `/follow/lio/odom`
  - `/follow/lidar/points`
- Stage2 目标层：
  - `/follow/stage2/goal`
  - `/follow/stage2/debug_goal_path`
- EGO ingress / planner：
  - `/move_base_simple/goal`
  - `/waypoint_generator/waypoints`
  - `/odom_world`
  - `/grid_map/odom`
  - `/grid_map/cloud`
- 下游控制：
  - `/follow/stage2/ego_position_cmd`
  - `/follow/stage2/offboard/setpoint`
  - `/mavros/setpoint_raw/local`

## 当前唯一正确的协作前沿

只剩一个有效前沿：

- Stage2 managed acceptance refresh 这一前沿已经完成。
- 当前剩余工作不再是“证明 Stage2 real-ego managed 能不能跑通”。
- 当前应保持：
  - project-side 本地 Stage2 证据
  - sim-plane managed Stage2 acceptance 证据
  - 两者分离存档
- 如果继续扩大仿真范围，下一步必须是新前沿，而不是重开当前已完成前沿。

## sim_plane 侧下一任务

`sim_plane` 新会话如果继续推进，第一优先不再是重复证明当前这条 Stage2 managed acceptance。

只有在下面两种情况下才需要它继续动这条线：

- 当前 artifact 丢失、损坏、不可复现
- 或者 `/home/coco/follwer_ws` 的 Stage2 主链合同再次变化

## sim_plane 侧禁止事项

- 不要改 `/home/coco/follwer_ws` 的项目算法文件来规避平台问题。
- 不要回退到 Stage2 placeholder 线。
- 不要把 Stage1 managed proof 和 Stage2 real-ego proof 混写成一条证据。
- 不要重新打开 real-fusion、detector-in-the-loop、硬件链路这些更高层任务。

## 给 sim_plane 会话的直接指令模板

把下面这段原样发给 `sim_plane` 新会话即可：

你现在接手的是 human-follow 仿真支线的受管验证，不是项目主线开发。

先读：
- `/home/coco/follwer_ws/human_follow_collab_ledger.md`
- `/home/coco/follwer_ws/STAGE2_EGO_FRONTIER.txt`

锁定事实：
- 项目主线源码真源在 `/home/coco/follwer_ws`
- 当前 Stage2 主线入口不是 `human_follow_user`
- 当前真实入口是 `/home/coco/follwer_ws/src/human_follow_bringup/launch/stage2_real_ego.launch`
- Stage2 follow + search + real EGO 的 project-side 本地证据已存在
- 你这边历史上也已经做出过 managed Stage2 real-ego artifact：
  - `/home/coco/sim_plane/runs/px4_sih_quadx_human_follow_stage2_real_ego_20260508_062640/result.json`

你的任务不要再问接口归属，不要重开 placeholder 线。只做这一件事：
- 在 `/home/coco/sim_plane` 内核验并刷新 `px4_sih_quadx_human_follow_stage2_real_ego` managed 受管证据
- 如需改动，只改 `sim_plane` 侧包装、scenario、acceptance、runner，不改 `/home/coco/follwer_ws` 项目算法主线

完成后只回这几项：
- 实际运行或核验的命令
- 最新 artifact 路径
- PASS/FAIL
- 这几个字段的实际值：
  - `algorithm_adapter_stage2_search_goal_observed`
  - `algorithm_adapter_stage2_real_ego_path_observed`
  - `algorithm_adapter_stage2_waypoint_count`
  - `algorithm_adapter_stage2_distinct_goal_count`
  - `algorithm_adapter_stage2_distinct_ego_cmd_count`
