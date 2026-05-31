#!/usr/bin/env python3
"""Offline scenario smoke tests for the paper-line ROS1 adapter core.

This script does not start ROS. It imports the ROS1 node with stubs and checks
the deterministic decision core across candidate selection, target-loss state
transitions, hard safety filtering, and failed-candidate cooldown.
"""

import math
import time

from smoke_ros1_stage2_adapter_core import install_stubs, load_adapter, make_uninitialized_node


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


def make_scenario_node(module):
    node = make_uninitialized_node(module)
    node.last_valid_target = make_target()
    node.last_valid_target_stamp = FakeStamp(99.0)
    node.last_target_seen_monotonic_sec = time.monotonic()
    node.last_odom = make_odom()
    node.last_odom_stamp = FakeStamp(100.0)
    node.enable_obstacle_safety_filter = True
    node.occupancy_resolution_m = 0.1
    node.segment_sample_step_m = 0.15
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


def _run_candidate_case(module, name, predicted, vehicle_xyz, occupied_points=None, last_candidate_name=""):
    node = make_scenario_node(module)
    node.last_candidate_name = last_candidate_name
    node.enable_obstacle_safety_filter = occupied_points is not None
    occupied = None
    if occupied_points is not None:
        occupied = set(node._cell_key(point[0], point[1], point[2]) for point in occupied_points)

    candidates = node._generate_follow_candidates(predicted, vehicle_xyz)
    safe, margin = node._filter_and_score_candidates(
        candidates,
        vehicle_xyz,
        vehicle_speed=math.hypot(predicted[2], predicted[3]),
        covariance=module.mat_eye(4, 0.05),
        occupied=occupied,
        now_sec=1.0,
    )
    if not safe:
        raise AssertionError("%s produced no safe candidate" % name)
    return safe[0].name, safe[0].score, margin


def run_candidate_selection_checks(module):
    cases = [
        {
            "name": "open_follow",
            "predicted": [2.0, 0.0, 1.0, 0.0],
            "vehicle_xyz": (0.0, 0.0, 1.0),
            "expected": {"behind", "left", "right", "far_safe"},
        },
        {
            "name": "behind_blocked",
            "predicted": [2.0, 0.0, 1.0, 0.0],
            "vehicle_xyz": (0.0, 0.0, 1.0),
            "occupied_points": [(-1.5, 0.0, 1.0)],
            "expected": {"left", "right", "far_safe"},
        },
        {
            "name": "switching_bias",
            "predicted": [2.0, 0.0, 1.0, 0.0],
            "vehicle_xyz": (0.0, 0.0, 1.0),
            "last_candidate_name": "left",
            "expected": {"left", "behind", "right", "far_safe"},
        },
    ]

    results = []
    for case in cases:
        best_name, score, margin = _run_candidate_case(
            module,
            case["name"],
            case["predicted"],
            case["vehicle_xyz"],
            case.get("occupied_points"),
            case.get("last_candidate_name", ""),
        )
        if best_name not in case["expected"]:
            raise AssertionError("%s selected unexpected candidate %s" % (case["name"], best_name))
        results.append("%s:%s:%.3f:%.3f" % (case["name"], best_name, score, margin))
    return results


def run_state_and_cooldown_checks(module):
    node = make_scenario_node(module)
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


def main():
    install_stubs()
    module = load_adapter()
    candidate_results = run_candidate_selection_checks(module)
    run_state_and_cooldown_checks(module)
    print(
        "scenario smoke PASS "
        + " ".join(candidate_results)
        + " states=follow,predict_hold,search_safe_viewpoint,hold_safe "
        + "hard_filter=behind_rejected cooldown=blocked_then_expired"
    )


if __name__ == "__main__":
    main()
