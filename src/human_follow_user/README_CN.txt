human_follow_user
=================

这个包是给你自己算法留的唯一标准位置。

目的
----

- 不再把你自己的算法直接混写进 `human_follow_control`
- 不再到处新写 launch 版本
- 以后你自己的控制类/规划类算法，都只往这个包里放

当前默认行为
------------

当前这两个入口脚本先不自己实现算法，而是默认转接到已经验过的模板：

- `scripts/user_control_node.py`
- `scripts/user_planning_node.py`
- `scripts/user_stage2_goal_node.py`

这样做的意义是：

1. 这个包现在立刻就能接进真线
2. 你可以先验证“自己的包接入没问题”
3. 后面再直接把这两个脚本替换成你自己的真实算法
4. Stage2 也先给你留好了独立 goal 生成入口，不用再改主线 launch

建议用法
--------

控制类算法：

- 订阅 `/follow/fusion/target_body`
- 发布 `/follow/control/cmd_body`
- 发布 `/follow/state`

规划/感知类算法：

- 订阅 `/follow/fusion/target_world`
- 订阅 `/follow/sim/vehicle_odom` 或部署时的 `/follow/lio/odom`
- 订阅 `/follow/lidar/points` 或真实点云 topic
- 发布 `/follow/control/cmd_body`
- 发布 `/follow/state`
- 可选发布 `/follow/planning/debug_path`

Stage2 目标生成类算法：

- 订阅 `/follow/fusion/target_world`
- 订阅 `/follow/lio/odom`
- 发布 `/follow/stage2/goal`
- 可选发布 `/follow/stage2/debug_goal_path`

当前主线还会自动把 Stage2 标准 topic 适配成 EGO 常用 topic：

- `/follow/stage2/goal` -> `/move_base_simple/goal`
- `/follow/lio/odom` -> `/odom_world`
- `/follow/lio/odom` -> `/grid_map/odom`
- `/follow/lidar/points` -> `/grid_map/cloud`

直接跑当前用户包
----------------

仿真真线 + 用户控制入口：

`roslaunch human_follow_bringup stage1_truth_visual_demo.launch controller_provider:=external external_controller_pkg:=human_follow_user external_controller_type:=user_control_node.py`

仿真真线 + 用户规划入口：

`roslaunch human_follow_bringup stage1_truth_visual_demo.launch controller_provider:=external external_controller_pkg:=human_follow_user external_controller_type:=user_planning_node.py`

真线接入验收：

控制类：

`roslaunch human_follow_bringup stage1_external_ingress_regression.launch external_controller_pkg:=human_follow_user external_controller_type:=user_control_node.py`

规划类：

`roslaunch human_follow_bringup stage1_external_ingress_regression.launch external_controller_pkg:=human_follow_user external_controller_type:=user_planning_node.py require_planning_path:=true`

Stage2 goal 入口：

`roslaunch human_follow_bringup stage2_placeholder.launch stage2_enabled:=true goal_provider:=external external_goal_pkg:=human_follow_user external_goal_type:=user_stage2_goal_node.py`

后续替换原则
------------

- 只改 `human_follow_user` 里的脚本
- 不改主线 topic 名
- 不改主线 launch 结构
- 先在真仿真链验收，再切真部署链
