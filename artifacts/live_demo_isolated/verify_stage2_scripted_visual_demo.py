#!/usr/bin/env python3
import argparse
import json
import math
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from human_follow_msgs.msg import FollowState


MODE_CONFIG = {
    "autoplay": {
        "start_label": "ego_follow_right_bias",
        "required_labels": [
            "ego_follow_right_bias",
            "ego_follow_center_obstacle",
            "ego_follow_left_bias",
        ],
        "window_sec": 27.8,
        "min_path_length_m": 8.0,
        "min_y_span_m": 1.8,
        "obstacle_layout": "",
        "min_detour_phase_count": 0,
    },
    "replan_showcase": {
        "start_label": "slalom_prime_right",
        "required_labels": [
            "slalom_prime_right",
            "slalom_cross_left",
            "slalom_cross_right",
            "slalom_cross_left_again",
            "slalom_settle_center",
        ],
        "window_sec": 24.8,
        "min_path_length_m": 10.0,
        "min_y_span_m": 2.8,
        "obstacle_layout": "ego_demo_replan_slalom",
        "min_detour_phase_count": 1,
    },
}


OBSTACLE_LAYOUT_BOXES = {
    "ego_demo_replan_slalom": [
        ((4.0, 1.55, 1.0), (0.8, 1.2, 2.0)),
        ((4.0, -1.55, 1.0), (0.8, 1.2, 2.0)),
        ((5.8, 0.55, 1.0), (0.9, 1.1, 2.0)),
        ((7.6, -0.55, 1.0), (0.9, 1.1, 2.0)),
        ((9.4, 1.55, 1.0), (0.8, 1.2, 2.0)),
        ((9.4, -1.55, 1.0), (0.8, 1.2, 2.0)),
    ],
}


def parse_phase_descriptor(text):
    parsed = {
        "label": "",
        "visible": False,
        "truth_valid": False,
    }
    if not text:
        return parsed

    for item in str(text).split(";"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        key = key.strip()
        value = value.strip()
        if key == "label":
            parsed["label"] = value
        elif key == "visible":
            parsed["visible"] = value in ("1", "true", "True", "yes")
        elif key == "truth_valid":
            parsed["truth_valid"] = value in ("1", "true", "True", "yes")
    return parsed


class ScriptedVisualDemoVerifier:
    def __init__(self):
        self.last_odom = None
        self.last_state = None
        self.last_phase = {"label": "", "visible": False, "truth_valid": False}

        self.obstacle_margin_m = max(
            float(rospy.get_param("/ego_planner_node/grid_map/obstacles_inflation", 0.1)) + 0.05,
            0.10,
        )

        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/state", FollowState, self._state_cb, queue_size=50)
        rospy.Subscriber("/follow/sim/truth_phase", String, self._phase_cb, queue_size=50)

    def _odom_cb(self, msg):
        self.last_odom = msg

    def _state_cb(self, msg):
        self.last_state = msg

    def _phase_cb(self, msg):
        self.last_phase = parse_phase_descriptor(msg.data)

    def wait_ready(self, timeout_sec=10.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_odom is not None and self.last_state is not None and self.last_phase["label"]:
                return True
            time.sleep(0.05)
        return False

    def wait_for_start_label(self, label, timeout_sec=12.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_phase.get("label", "") == label:
                return True
            time.sleep(0.05)
        return False

    def _sample_xyz(self):
        if self.last_odom is None:
            return None
        point = self.last_odom.pose.pose.position
        return (float(point.x), float(point.y), float(point.z))

    def _point_in_layout_box(self, layout_name, point_xyz):
        boxes = OBSTACLE_LAYOUT_BOXES.get(layout_name, [])
        if not boxes:
            return False

        x_value, y_value, z_value = point_xyz
        margin = self.obstacle_margin_m
        for center_xyz, size_xyz in boxes:
            half_x = 0.5 * float(size_xyz[0]) + margin
            half_y = 0.5 * float(size_xyz[1]) + margin
            half_z = 0.5 * float(size_xyz[2]) + margin
            if (
                abs(x_value - float(center_xyz[0])) <= half_x
                and abs(y_value - float(center_xyz[1])) <= half_y
                and abs(z_value - float(center_xyz[2])) <= half_z
            ):
                return True
        return False

    def run(self, mode_name):
        config = MODE_CONFIG[mode_name]
        if not self.wait_ready():
            return {"status": "failed", "reason": "wait_ready_timeout", "mode": mode_name}
        if not self.wait_for_start_label(config["start_label"]):
            return {"status": "failed", "reason": "wait_start_label_timeout", "mode": mode_name}

        start_xyz = self._sample_xyz()
        if start_xyz is None:
            return {"status": "failed", "reason": "missing_start_odom", "mode": mode_name}

        points = []
        phase_labels_seen = []
        phase_label_set = set()
        follow_phase_labels = set()
        detour_phase_labels = set()
        state_counts = {}
        detail_examples = []
        last_detail = None
        entered_obstacle_box = False

        deadline = time.time() + float(config["window_sec"])
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.10)
            point_xyz = self._sample_xyz()
            if point_xyz is None or self.last_state is None:
                continue

            label = str(self.last_phase.get("label", "") or "")
            if label and label not in phase_label_set:
                phase_label_set.add(label)
                phase_labels_seen.append(label)

            state_name = str(getattr(self.last_state, "state_name", "") or "")
            detail = str(getattr(self.last_state, "detail", "") or "")
            state_counts[state_name] = state_counts.get(state_name, 0) + 1

            if label in config["required_labels"] and state_name == "follow":
                follow_phase_labels.add(label)
            if label in config["required_labels"] and (
                "follow_detour" in detail or "follow_path_blocked_escape" in detail
            ):
                detour_phase_labels.add(label)
            if detail and detail != last_detail and len(detail_examples) < 8:
                detail_examples.append(detail)
                last_detail = detail

            if config["obstacle_layout"] and self._point_in_layout_box(config["obstacle_layout"], point_xyz):
                entered_obstacle_box = True

            points.append(point_xyz)

        if not points:
            return {"status": "failed", "reason": "no_runtime_samples", "mode": mode_name}

        path_length_m = 0.0
        max_step_xy_m = 0.0
        x_values = [point[0] for point in points]
        y_values = [point[1] for point in points]
        for previous, current in zip(points, points[1:]):
            step_xy = math.hypot(current[0] - previous[0], current[1] - previous[1])
            path_length_m += step_xy
            max_step_xy_m = max(max_step_xy_m, step_xy)

        result = {
            "mode": mode_name,
            "start_xyz": [round(value, 3) for value in start_xyz],
            "end_xyz": [round(value, 3) for value in points[-1]],
            "path_length_xy_m": round(path_length_m, 3),
            "x_span_m": round(max(x_values) - min(x_values), 3),
            "y_span_m": round(max(y_values) - min(y_values), 3),
            "max_step_xy_m": round(max_step_xy_m, 3),
            "phase_labels_seen": phase_labels_seen,
            "follow_phase_labels": sorted(follow_phase_labels),
            "detour_phase_labels": sorted(detour_phase_labels),
            "entered_obstacle_box": entered_obstacle_box,
            "state_counts": state_counts,
            "detail_examples": detail_examples,
        }

        saw_required_labels = all(label in phase_label_set for label in config["required_labels"])
        follow_observed_all = all(label in follow_phase_labels for label in config["required_labels"])
        path_ok = path_length_m >= float(config["min_path_length_m"])
        lateral_ok = (max(y_values) - min(y_values)) >= float(config["min_y_span_m"])
        detour_ok = len(detour_phase_labels) >= int(config["min_detour_phase_count"])
        clear_ok = not entered_obstacle_box

        checks = {
            "required_labels_seen": saw_required_labels,
            "follow_observed_all_required_labels": follow_observed_all,
            "path_length_ok": path_ok,
            "lateral_span_ok": lateral_ok,
            "detour_phase_count_ok": detour_ok,
            "obstacle_box_clear": clear_ok,
        }

        passed = all(checks.values())
        return {
            "status": "passed" if passed else "failed",
            "mode": mode_name,
            "result": result,
            "checks": checks,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=sorted(MODE_CONFIG.keys()), required=True)
    args = parser.parse_args()

    rospy.init_node("stage2_scripted_visual_demo_verifier", anonymous=True)
    verifier = ScriptedVisualDemoVerifier()
    result = verifier.run(args.mode)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result["status"] != "passed":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
