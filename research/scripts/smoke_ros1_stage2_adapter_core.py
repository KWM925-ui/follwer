#!/usr/bin/env python3
"""Offline smoke test for the ROS1 paper-line Stage2 adapter core.

This does not start ROS or validate topic transport. It stubs ROS modules so
the pure Python prediction/candidate/scoring logic can be checked on machines
that do not have the target Ubuntu 20.04 + ROS1 runtime sourced.
"""

import importlib.util
from pathlib import Path
import sys
import types


class Dummy:
    pass


def install_stubs():
    rospy = types.ModuleType("rospy")
    rospy.AnyMsg = Dummy
    sys.modules["rospy"] = rospy

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = Dummy
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = Dummy
    nav_msgs_msg.Path = Dummy
    sys.modules["nav_msgs"] = nav_msgs
    sys.modules["nav_msgs.msg"] = nav_msgs_msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_pc2 = types.ModuleType("sensor_msgs.point_cloud2")
    sensor_msgs_msg.PointCloud2 = Dummy
    sensor_msgs_pc2.read_points = lambda *args, **kwargs: []
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
    sys.modules["sensor_msgs.point_cloud2"] = sensor_msgs_pc2

    human_follow_msgs = types.ModuleType("human_follow_msgs")
    human_follow_msgs_msg = types.ModuleType("human_follow_msgs.msg")
    human_follow_msgs_msg.FollowState = type(
        "FollowState",
        (),
        {"IDLE": 0, "TARGET_ACQUIRED": 1, "FOLLOW": 2, "SEARCH": 3, "LOST": 4},
    )
    human_follow_msgs_msg.Target3D = Dummy
    sys.modules["human_follow_msgs"] = human_follow_msgs
    sys.modules["human_follow_msgs.msg"] = human_follow_msgs_msg


def load_adapter():
    repo_root = Path(__file__).resolve().parents[2]
    adapter = repo_root / "src" / "human_follow_user" / "scripts" / "user_stage2_goal_node.py"
    spec = importlib.util.spec_from_file_location("paper_line_stage2_goal", str(adapter))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_uninitialized_node(module):
    node = object.__new__(module.PaperLineStage2GoalNode)
    node.last_goal_yaw = 0.0
    node.face_target_yaw = True
    node.last_valid_target = None
    node.vertical_bias_m = 0.0
    node.follow_distance_m = 3.5
    node.lateral_bias_m = 0.0
    node.min_motion_speed_mps = 0.15
    node.search_observation_distance_m = -1.0
    node.search_goal_radius_m = 1.0
    node.search_phase = 0
    node.enable_obstacle_safety_filter = False
    node.base_safety_margin_m = 0.28
    node.obstacle_margin_m = 0.18
    node.speed_margin_gain = 0.35
    node.covariance_margin_gain = 1.0
    node.failed_candidates = []
    node.failed_candidate_cooldown_sec = 2.5
    node.failed_candidate_radius_m = 0.7
    node.last_candidate_name = ""
    node.weight_distance = 0.24
    node.weight_clearance = 0.26
    node.weight_motion = 0.20
    node.weight_visibility = 0.12
    node.weight_switching = 0.10
    node.weight_uncertainty = 0.08
    return node


def main():
    install_stubs()
    module = load_adapter()

    kf = module.ConstantVelocityKalmanFilter(
        process_var=1.2,
        measurement_var=0.09,
        initial_covariance=1.0,
    )
    kf.update(0.0, 0.0, 0.0)
    kf.update(0.2, 0.2, 0.0)
    state, covariance = kf.predict(0.6)
    assert state is not None and covariance is not None
    assert len(state) == 4 and state[0] > 0.2

    node = make_uninitialized_node(module)
    candidates = node._generate_follow_candidates([2.0, 0.0, 1.0, 0.0], (0.0, 0.0, 1.0))
    assert [candidate.name for candidate in candidates] == ["behind", "left", "right", "far_safe"]

    safe, margin = node._filter_and_score_candidates(
        candidates,
        (0.0, 0.0, 1.0),
        0.2,
        module.mat_eye(4, 0.05),
        None,
        1.0,
    )
    assert safe
    assert safe[0].score > 0.0
    assert margin > 0.0
    print("offline smoke PASS candidate=%s score=%.3f margin=%.3f" % (safe[0].name, safe[0].score, margin))


if __name__ == "__main__":
    main()
