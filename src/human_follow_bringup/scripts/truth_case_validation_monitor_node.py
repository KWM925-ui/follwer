#!/usr/bin/env python3
import math
import sys
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from human_follow_msgs.msg import FollowCommand, FollowState, Target3D, TrackerStatus


def wrap_angle(rad_value):
    while rad_value > math.pi:
        rad_value -= 2.0 * math.pi
    while rad_value < -math.pi:
        rad_value += 2.0 * math.pi
    return rad_value


def yaw_from_quaternion(x_value, y_value, z_value, w_value):
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_phase_descriptor(text):
    parsed = {
        "label": "",
        "visible": False,
        "truth_valid": False,
        "phase_index": -1,
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
        elif key == "phase_index":
            try:
                parsed["phase_index"] = int(value)
            except ValueError:
                parsed["phase_index"] = -1
    return parsed


CASE_ALIASES = {
    "search_right": "search_reacquire_right",
    "search_left": "search_reacquire_left",
    "approach_retreat": "person_approach_retreat",
    "retreat_follow": "person_depart_follow",
    "depart_follow": "person_depart_follow",
    "left_track": "lateral_left_track",
    "right_track": "lateral_right_track",
}


CASE_CONFIG = {
    "acquire_center": {
        "main_labels": ["acquire_center", "settle_center"],
        "max_duration_sec": 10.0,
    },
    "search_reacquire_right": {
        "search_label": "search_loss_right",
        "reacquire_label": "reacquire_right",
        "expected_search_yaw_sign": -1.0,
        "max_duration_sec": 12.0,
    },
    "search_reacquire_left": {
        "search_label": "search_loss_left",
        "reacquire_label": "reacquire_left",
        "expected_search_yaw_sign": 1.0,
        "max_duration_sec": 12.0,
    },
    "person_approach_retreat": {
        "main_label": "approach_retreat_center",
        "hold_label": "retreat_hold_center",
        "expected_forward_sign": -1.0,
        "max_duration_sec": 12.0,
    },
    "person_depart_follow": {
        "main_label": "depart_follow_center",
        "hold_label": "depart_hold_center",
        "expected_forward_sign": 1.0,
        "max_duration_sec": 12.0,
    },
    "lateral_left_track": {
        "main_label": "lateral_left_track",
        "hold_label": "settle_left",
        "expected_lateral_sign": 1.0,
        "max_duration_sec": 12.0,
    },
    "lateral_right_track": {
        "main_label": "lateral_right_track",
        "hold_label": "settle_right",
        "expected_lateral_sign": -1.0,
        "max_duration_sec": 12.0,
    },
}


class TruthCaseValidationMonitorNode:
    def __init__(self):
        requested_case = str(rospy.get_param("~validation_case", "")).strip().lower()
        self.validation_case = CASE_ALIASES.get(requested_case, requested_case)
        if self.validation_case not in CASE_CONFIG:
            supported = ",".join(sorted(CASE_CONFIG.keys()))
            raise RuntimeError("unsupported validation_case=%s supported=%s" % (self.validation_case, supported))

        self.case_config = CASE_CONFIG[self.validation_case]
        self.phase_topic = rospy.get_param("~phase_topic", "/follow/sim/truth_phase")
        self.truth_topic = rospy.get_param("~truth_topic", "/follow/sim/truth_target_body")
        self.target_body_topic = rospy.get_param("~target_body_topic", "/follow/fusion/target_body")
        self.follow_state_topic = rospy.get_param("~follow_state_topic", "/follow/state")
        self.follow_command_topic = rospy.get_param("~follow_command_topic", "/follow/control/cmd_body")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.tracker_status_topic = rospy.get_param("~tracker_status_topic", "/follow/tracker/status")
        self.startup_grace_sec = float(rospy.get_param("~startup_grace_sec", 0.5))
        self.max_msg_age_sec = float(rospy.get_param("~max_msg_age_sec", 0.6))
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", self.case_config["max_duration_sec"]))
        self.check_rate_hz = float(rospy.get_param("~check_rate_hz", 20.0))
        self.desired_distance_m = float(rospy.get_param("~desired_distance_m", 4.0))
        self.distance_margin_m = float(rospy.get_param("~distance_margin_m", 0.35))
        self.min_forward_command_mps = float(rospy.get_param("~min_forward_command_mps", 0.22))
        self.min_lateral_command_mps = float(rospy.get_param("~min_lateral_command_mps", 0.12))
        self.min_search_yaw_rate_rps = float(rospy.get_param("~min_search_yaw_rate_rps", 0.14))
        self.min_vehicle_translation_m = float(rospy.get_param("~min_vehicle_translation_m", 0.28))
        self.min_vehicle_lateral_translation_m = float(
            rospy.get_param("~min_vehicle_lateral_translation_m", 0.22)
        )
        self.min_search_yaw_delta_rad = float(rospy.get_param("~min_search_yaw_delta_rad", 0.10))
        self.accept_search_yaw_sign_flip = bool(rospy.get_param("~accept_search_yaw_sign_flip", False))

        self.start_time = time.monotonic()
        self.finished = False
        self.exit_code = 0
        self.failure_reason = None

        self.last_phase = parse_phase_descriptor("")
        self.last_phase_time = None
        self.last_truth = None
        self.last_truth_time = None
        self.last_target_body = None
        self.last_target_body_time = None
        self.last_follow_state = None
        self.last_follow_state_time = None
        self.last_command = None
        self.last_command_time = None
        self.last_odom = None
        self.last_odom_time = None
        self.last_tracker_status = None

        self.seen_phase_labels = set()
        self.seen_follow_states = set()
        self.follow_valid_seen = False
        self.target_valid_seen = False
        self.search_command_seen = False
        self.search_reacquire_seen = False
        self.search_phase_active = False
        self.search_start_yaw = None
        self.min_search_yaw_delta = 0.0
        self.max_search_yaw_delta = 0.0
        self.min_forward_command = 0.0
        self.max_forward_command = 0.0
        self.min_lateral_command = 0.0
        self.max_lateral_command = 0.0
        self.min_yaw_rate_command = 0.0
        self.max_yaw_rate_command = 0.0
        self.min_target_x = float("inf")
        self.max_target_x = float("-inf")
        self.main_phase_origin = None
        self.vehicle_dx_min = 0.0
        self.vehicle_dx_max = 0.0
        self.vehicle_dy_min = 0.0
        self.vehicle_dy_max = 0.0

        self.phase_sub = rospy.Subscriber(self.phase_topic, String, self._phase_callback, queue_size=20)
        self.truth_sub = rospy.Subscriber(self.truth_topic, Target3D, self._truth_callback, queue_size=20)
        self.target_body_sub = rospy.Subscriber(self.target_body_topic, Target3D, self._target_body_callback, queue_size=20)
        self.follow_state_sub = rospy.Subscriber(
            self.follow_state_topic, FollowState, self._follow_state_callback, queue_size=20
        )
        self.follow_command_sub = rospy.Subscriber(
            self.follow_command_topic, FollowCommand, self._follow_command_callback, queue_size=20
        )
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        self.tracker_status_sub = rospy.Subscriber(
            self.tracker_status_topic, TrackerStatus, self._tracker_status_callback, queue_size=20
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.check_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "truth_case_validation ready: case=%s phase=%s target_body=%s cmd=%s odom=%s",
            self.validation_case,
            self.phase_topic,
            self.target_body_topic,
            self.follow_command_topic,
            self.odom_topic,
        )

    def _is_recent(self, stamp, now):
        if stamp is None:
            return False
        return (now - stamp) <= self.max_msg_age_sec

    def _record_main_phase_origin_if_needed(self):
        if self.last_odom is None or self.main_phase_origin is not None:
            return
        main_label = self.case_config.get("main_label")
        main_labels = self.case_config.get("main_labels", [])
        active_labels = set(main_labels)
        if main_label:
            active_labels.add(main_label)
        if self.last_phase.get("label", "") not in active_labels:
            return
        self.main_phase_origin = (
            float(self.last_odom.pose.pose.position.x),
            float(self.last_odom.pose.pose.position.y),
        )

    def _phase_callback(self, msg):
        previous_label = self.last_phase.get("label", "")
        self.last_phase = parse_phase_descriptor(msg.data)
        self.last_phase_time = time.monotonic()
        label = self.last_phase.get("label", "")
        if label:
            self.seen_phase_labels.add(label)
        search_label = self.case_config.get("search_label")
        if search_label:
            if label == search_label and previous_label != search_label:
                self.search_phase_active = True
                if self.last_odom is not None:
                    self.search_start_yaw = yaw_from_quaternion(
                        self.last_odom.pose.pose.orientation.x,
                        self.last_odom.pose.pose.orientation.y,
                        self.last_odom.pose.pose.orientation.z,
                        self.last_odom.pose.pose.orientation.w,
                    )
            elif self.search_phase_active and label != search_label and previous_label == search_label:
                self.search_phase_active = False

    def _truth_callback(self, msg):
        self.last_truth = msg
        self.last_truth_time = time.monotonic()

    def _target_body_callback(self, msg):
        self.last_target_body = msg
        self.last_target_body_time = time.monotonic()
        if msg.valid:
            self.target_valid_seen = True
            self.min_target_x = min(self.min_target_x, float(msg.position.x))
            self.max_target_x = max(self.max_target_x, float(msg.position.x))

    def _follow_state_callback(self, msg):
        self.last_follow_state = msg
        self.last_follow_state_time = time.monotonic()
        if msg.state_name:
            self.seen_follow_states.add(msg.state_name)
        reacquire_label = self.case_config.get("reacquire_label")
        if (
            reacquire_label
            and self.last_phase.get("label", "") == reacquire_label
            and msg.state_name in ("target_acquired", "follow")
        ):
            self.search_reacquire_seen = True

    def _follow_command_callback(self, msg):
        self.last_command = msg
        self.last_command_time = time.monotonic()
        if not msg.valid:
            return
        if msg.mode == "body_velocity_yaw_rate":
            self.follow_valid_seen = True
        self.min_forward_command = min(self.min_forward_command, float(msg.forward_mps))
        self.max_forward_command = max(self.max_forward_command, float(msg.forward_mps))
        self.min_lateral_command = min(self.min_lateral_command, float(msg.lateral_mps))
        self.max_lateral_command = max(self.max_lateral_command, float(msg.lateral_mps))
        self.min_yaw_rate_command = min(self.min_yaw_rate_command, float(msg.yaw_rate_rps))
        self.max_yaw_rate_command = max(self.max_yaw_rate_command, float(msg.yaw_rate_rps))
        if msg.mode == "search_yaw_only":
            self.search_command_seen = True

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_time = time.monotonic()
        self._record_main_phase_origin_if_needed()

        if self.main_phase_origin is not None:
            dx_value = float(msg.pose.pose.position.x) - self.main_phase_origin[0]
            dy_value = float(msg.pose.pose.position.y) - self.main_phase_origin[1]
            self.vehicle_dx_min = min(self.vehicle_dx_min, dx_value)
            self.vehicle_dx_max = max(self.vehicle_dx_max, dx_value)
            self.vehicle_dy_min = min(self.vehicle_dy_min, dy_value)
            self.vehicle_dy_max = max(self.vehicle_dy_max, dy_value)

        if self.search_phase_active and self.search_start_yaw is not None:
            current_yaw = yaw_from_quaternion(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            yaw_delta = wrap_angle(current_yaw - self.search_start_yaw)
            self.min_search_yaw_delta = min(self.min_search_yaw_delta, yaw_delta)
            self.max_search_yaw_delta = max(self.max_search_yaw_delta, yaw_delta)

    def _tracker_status_callback(self, msg):
        self.last_tracker_status = msg

    def _finish(self, success, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0 if success else 1
        if success:
            rospy.loginfo(
                "truth_case_validation PASS case=%s phases=%s states=%s dx=[%.3f,%.3f] dy=[%.3f,%.3f] cmd_forward=[%.3f,%.3f] cmd_lateral=[%.3f,%.3f] yaw_delta=[%.3f,%.3f]",
                self.validation_case,
                sorted(self.seen_phase_labels),
                sorted(self.seen_follow_states),
                self.vehicle_dx_min,
                self.vehicle_dx_max,
                self.vehicle_dy_min,
                self.vehicle_dy_max,
                self.min_forward_command,
                self.max_forward_command,
                self.min_lateral_command,
                self.max_lateral_command,
                self.min_search_yaw_delta,
                self.max_search_yaw_delta,
            )
        else:
            self.failure_reason = reason
            rospy.logerr(
                "truth_case_validation FAIL case=%s reason=%s dx=[%.3f,%.3f] dy=[%.3f,%.3f] cmd_forward=[%.3f,%.3f] cmd_lateral=[%.3f,%.3f] yaw_cmd=[%.3f,%.3f] yaw_delta=[%.3f,%.3f]",
                self.validation_case,
                reason,
                self.vehicle_dx_min,
                self.vehicle_dx_max,
                self.vehicle_dy_min,
                self.vehicle_dy_max,
                self.min_forward_command,
                self.max_forward_command,
                self.min_lateral_command,
                self.max_lateral_command,
                self.min_yaw_rate_command,
                self.max_yaw_rate_command,
                self.min_search_yaw_delta,
                self.max_search_yaw_delta,
            )
        rospy.signal_shutdown("truth case validation complete")

    def _search_case_checks(self):
        missing = []
        search_label = self.case_config["search_label"]
        reacquire_label = self.case_config["reacquire_label"]
        expected_sign = float(self.case_config["expected_search_yaw_sign"])

        if search_label not in self.seen_phase_labels:
            missing.append("phase:%s" % search_label)
        if reacquire_label not in self.seen_phase_labels:
            missing.append("phase:%s" % reacquire_label)
        if "search" not in self.seen_follow_states:
            missing.append("state:search")
        if not self.search_command_seen:
            missing.append("search_cmd")
        if not self.search_reacquire_seen:
            missing.append("reacquire_follow")
        if self.last_tracker_status is None or self.last_tracker_status.timeout_count < 1:
            missing.append("timeout_count")
        if self.last_tracker_status is None or self.last_tracker_status.reacquire_count < 1:
            missing.append("reacquire_count")

        if self.accept_search_yaw_sign_flip:
            if expected_sign < 0.0:
                if self.min_yaw_rate_command > -self.min_search_yaw_rate_rps:
                    missing.append("yaw_rate_negative")
                if self.min_search_yaw_delta > -self.min_search_yaw_delta_rad:
                    missing.append("yaw_delta_negative")
                if self.max_yaw_rate_command < self.min_search_yaw_rate_rps:
                    missing.append("yaw_rate_positive_return")
                if self.max_search_yaw_delta < self.min_search_yaw_delta_rad:
                    missing.append("yaw_delta_positive_return")
            else:
                if self.max_yaw_rate_command < self.min_search_yaw_rate_rps:
                    missing.append("yaw_rate_positive")
                if self.max_search_yaw_delta < self.min_search_yaw_delta_rad:
                    missing.append("yaw_delta_positive")
                if self.min_yaw_rate_command > -self.min_search_yaw_rate_rps:
                    missing.append("yaw_rate_negative_return")
                if self.min_search_yaw_delta > -self.min_search_yaw_delta_rad:
                    missing.append("yaw_delta_negative_return")
        elif expected_sign < 0.0:
            if self.min_yaw_rate_command > -self.min_search_yaw_rate_rps:
                missing.append("yaw_rate_negative")
            if self.min_search_yaw_delta > -self.min_search_yaw_delta_rad:
                missing.append("yaw_delta_negative")
        else:
            if self.max_yaw_rate_command < self.min_search_yaw_rate_rps:
                missing.append("yaw_rate_positive")
            if self.max_search_yaw_delta < self.min_search_yaw_delta_rad:
                missing.append("yaw_delta_positive")
        return missing

    def _approach_or_depart_checks(self):
        missing = []
        expected_sign = float(self.case_config["expected_forward_sign"])
        main_label = self.case_config["main_label"]
        hold_label = self.case_config["hold_label"]

        if main_label not in self.seen_phase_labels:
            missing.append("phase:%s" % main_label)
        if hold_label not in self.seen_phase_labels:
            missing.append("phase:%s" % hold_label)
        if not self.target_valid_seen:
            missing.append("target_valid")
        if not self.follow_valid_seen:
            missing.append("follow_cmd")
        if "target_acquired" not in self.seen_follow_states and "follow" not in self.seen_follow_states:
            missing.append("state:follow")

        if expected_sign < 0.0:
            if self.min_target_x > (self.desired_distance_m - self.distance_margin_m):
                missing.append("target_not_close")
            if self.min_forward_command > -self.min_forward_command_mps:
                missing.append("retreat_cmd")
            if self.vehicle_dx_min > -self.min_vehicle_translation_m:
                missing.append("vehicle_backoff")
        else:
            if self.max_target_x < (self.desired_distance_m + self.distance_margin_m):
                missing.append("target_not_far")
            if self.max_forward_command < self.min_forward_command_mps:
                missing.append("follow_cmd_forward")
            if self.vehicle_dx_max < self.min_vehicle_translation_m:
                missing.append("vehicle_follow")
        return missing

    def _lateral_checks(self):
        missing = []
        expected_sign = float(self.case_config["expected_lateral_sign"])
        main_label = self.case_config["main_label"]
        hold_label = self.case_config["hold_label"]

        if main_label not in self.seen_phase_labels:
            missing.append("phase:%s" % main_label)
        if hold_label not in self.seen_phase_labels:
            missing.append("phase:%s" % hold_label)
        if not self.target_valid_seen:
            missing.append("target_valid")
        if not self.follow_valid_seen:
            missing.append("follow_cmd")
        if "target_acquired" not in self.seen_follow_states and "follow" not in self.seen_follow_states:
            missing.append("state:follow")

        if expected_sign > 0.0:
            if self.max_lateral_command < self.min_lateral_command_mps:
                missing.append("lateral_cmd_positive")
            if self.max_yaw_rate_command < self.min_search_yaw_rate_rps:
                missing.append("yaw_cmd_positive")
            if self.vehicle_dy_max < self.min_vehicle_lateral_translation_m:
                missing.append("vehicle_left")
        else:
            if self.min_lateral_command > -self.min_lateral_command_mps:
                missing.append("lateral_cmd_negative")
            if self.min_yaw_rate_command > -self.min_search_yaw_rate_rps:
                missing.append("yaw_cmd_negative")
            if self.vehicle_dy_min > -self.min_vehicle_lateral_translation_m:
                missing.append("vehicle_right")
        return missing

    def _acquire_center_checks(self):
        missing = []
        if "acquire_center" not in self.seen_phase_labels:
            missing.append("phase:acquire_center")
        if "settle_center" not in self.seen_phase_labels:
            missing.append("phase:settle_center")
        if not self.target_valid_seen:
            missing.append("target_valid")
        if not self.follow_valid_seen:
            missing.append("follow_cmd")
        if "target_acquired" not in self.seen_follow_states:
            missing.append("state:target_acquired")
        if "follow" not in self.seen_follow_states:
            missing.append("state:follow")
        if self.max_forward_command < 0.05:
            missing.append("vehicle_cutin_cmd")
        return missing

    def _missing_requirements(self):
        if self.validation_case == "acquire_center":
            return self._acquire_center_checks()
        if self.validation_case in ("search_reacquire_right", "search_reacquire_left"):
            return self._search_case_checks()
        if self.validation_case in ("person_approach_retreat", "person_depart_follow"):
            return self._approach_or_depart_checks()
        if self.validation_case in ("lateral_left_track", "lateral_right_track"):
            return self._lateral_checks()
        return ["unknown_case_logic"]

    def _tick(self, _event):
        if self.finished:
            return

        elapsed_sec = time.monotonic() - self.start_time
        if elapsed_sec < self.startup_grace_sec:
            return

        now = time.monotonic()
        required_stamps = [
            ("phase", self.last_phase_time),
            ("truth", self.last_truth_time),
            ("target_body", self.last_target_body_time),
            ("state", self.last_follow_state_time),
            ("command", self.last_command_time),
            ("odom", self.last_odom_time),
        ]
        stale = [name for name, stamp in required_stamps if not self._is_recent(stamp, now)]
        if not stale:
            missing = self._missing_requirements()
            if not missing:
                self._finish(True, "all requirements satisfied")
                return
        else:
            missing = ["stale:" + ",".join(stale)]

        if elapsed_sec >= self.max_duration_sec:
            self._finish(False, ",".join(missing))


def main():
    rospy.init_node("truth_case_validation_monitor")
    node = TruthCaseValidationMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
