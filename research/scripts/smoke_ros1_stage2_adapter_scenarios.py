#!/usr/bin/env python3
"""Offline scenario smoke tests for the paper-line ROS1 adapter core.

This checks decision-layer behavior without starting ROS transport. It is not
an EGO planner or hardware validation; the ROS regression launch covers the
planner-in-loop topic chain separately.
"""

import importlib.util
from pathlib import Path
import sys
import time
import types


class Dummy:
    pass


class FakeDuration:
    def __init__(self, seconds):
        self._seconds = float(seconds)

    def to_sec(self):
        return self._seconds


class FakeStamp:
    def __init__(self, seconds):
        self._seconds = float(seconds)

    def to_sec(self):
        return self._seconds

    def __sub__(self, other):
        return FakeDuration(self._seconds - other._seconds)


class Vector3:
    def __init__(self, x_value=0.0, y_value=0.0, z_value=0.0):
        self.x = float(x_value)
        self.y = float(y_value)
        self.z = float(z_value)


class Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


def install_stubs():
    rospy = types.ModuleType("rospy")
    rospy.AnyMsg = Dummy
    rospy.ROSException = RuntimeError
    rospy.logwarn_throttle = lambda *args, **kwargs: None
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


def nested(*names):
    root = Dummy()
    current = root
    for name in names:
        value = Dummy()
        setattr(current, name, value)
        current = value
    return root


def make_target(x_value=2.0, y_value=0.0, z_value=1.7):
    target = Dummy()
    target.valid = True
    target.position = Vector3(x_value, y_value, z_value)
    return target


def make_odom():
    odom = nested("pose", "pose")
    odom.twist = Dummy()
    odom.twist.twist = Dummy()
    odom.pose.pose.position = Vector3(0.0, 0.0, 1.0)
    odom.pose.pose.orientation = Quaternion()
    odom.twist.twist.linear = Vector3(0.2, 0.0, 0.0)
    return odom


def make_node(module):
    node = object.__new__(module.PaperLineStage2GoalNode)
    node.last_goal_yaw = 0.0
    node.face_target_yaw = True
    node.last_valid_target = make_target()
    node.last_valid_target_stamp = FakeStamp(99.0)
    node.last_target_seen_monotonic_sec = time.monotonic()
    node.last_odom = make_odom()
    node.last_odom_stamp = FakeStamp(100.0)
    node.vertical_bias_m = 0.0
    node.follow_distance_m = 3.5
    node.lateral_bias_m = 0.0
    node.min_motion_speed_mps = 0.15
    node.search_observation_distance_m = -1.0
    node.search_goal_radius_m = 1.0
    node.search_phase = 0
    node.enable_obstacle_safety_filter = True
    node.occupancy_resolution_m = 0.1
    node.base_safety_margin_m = 0.28
    node.obstacle_margin_m = 0.18
    node.speed_margin_gain = 0.35
    node.covariance_margin_gain = 1.0
    node.segment_sample_step_m = 0.15
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
    node.target_timeout_sec = 0.35
    node.odom_timeout_sec = 0.6
    node.predict_hold_timeout_sec = 1.2
    node.lost_timeout_sec = 5.0
    node.enable_planner_feedback = True
    node.require_planner_feedback = True
    node.planner_feedback_startup_grace_sec = 0.0
    node.planner_feedback_timeout_sec = 0.5
    node.last_candidate_stamp_sec = None
    node.last_planner_feedback_monotonic_sec = None
    node.start_monotonic_sec = time.monotonic() - 10.0
    return node


def assert_equal(actual, expected, label):
    if actual != expected:
        raise AssertionError("%s expected %r got %r" % (label, expected, actual))


def main():
    install_stubs()
    module = load_adapter()
    node = make_node(module)
    now = FakeStamp(100.0)

    node.last_valid_target_stamp = FakeStamp(99.9)
    assert_equal(node._state_name(now), "follow", "fresh target state")

    node.last_valid_target_stamp = FakeStamp(99.0)
    node.last_target_seen_monotonic_sec = time.monotonic() - 0.8
    assert_equal(node._state_name(now), "predict_hold", "short target loss state")

    node.last_target_seen_monotonic_sec = time.monotonic() - 2.0
    assert_equal(node._state_name(now), "search_safe_viewpoint", "long target loss state")

    node.last_target_seen_monotonic_sec = time.monotonic() - 6.0
    assert_equal(node._state_name(now), "hold_safe", "expired target loss state")

    predicted = [2.0, 0.0, 1.0, 0.0]
    vehicle_xyz = (0.0, 0.0, 1.0)
    candidates = node._generate_follow_candidates(predicted, vehicle_xyz)
    safe, _margin = node._filter_and_score_candidates(
        candidates,
        vehicle_xyz,
        0.2,
        module.mat_eye(4, 0.0),
        None,
        1.0,
    )
    assert safe and safe[0].name == "behind"

    behind = next(candidate for candidate in candidates if candidate.name == "behind")
    occupied = {node._cell_key(behind.x, behind.y, behind.z)}
    filtered, _margin = node._filter_and_score_candidates(
        candidates,
        vehicle_xyz,
        0.2,
        module.mat_eye(4, 0.0),
        occupied,
        2.0,
    )
    filtered_names = [candidate.name for candidate in filtered]
    if "behind" in filtered_names:
        raise AssertionError("hard safety filter did not reject occupied behind candidate")

    node.last_candidate_stamp_sec = time.monotonic() - 1.0
    if not node._planner_feedback_is_late():
        raise AssertionError("planner feedback timeout did not become late")

    node._mark_candidate_failed(behind, 3.0)
    if not node._candidate_in_cooldown(behind, 3.1):
        raise AssertionError("failed candidate was not blocked by cooldown")
    if node._candidate_in_cooldown(behind, 6.0):
        raise AssertionError("failed candidate cooldown did not expire")

    search_candidates = node._generate_search_candidates(predicted, vehicle_xyz)
    if not search_candidates or not search_candidates[0].name.startswith("search_reacquire_"):
        raise AssertionError("search candidates were not generated")

    print(
        "scenario smoke PASS normal=behind short_loss=predict_hold long_loss=search_safe_viewpoint "
        "expired_loss=hold_safe hard_filter=behind_rejected cooldown=blocked_then_expired"
    )


if __name__ == "__main__":
    main()
