#!/usr/bin/env python3
import glob
import math
import os
import sys
import time
from collections import deque

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

try:
    from runtime_mavros_import import ensure_mavros_python_path
except ImportError:
    def ensure_mavros_python_path():
        candidates = []
        env_prefix = os.environ.get("MAVROS_OVERLAY_PREFIX", "").strip()
        if env_prefix:
            candidates.append(env_prefix)
        candidates.append("/home/coco/.local/ros_noetic_overlay/opt/ros/noetic")

        for prefix in candidates:
            if not prefix:
                continue
            for dist_path in glob.glob(os.path.join(prefix, "lib", "python3*", "dist-packages")):
                if os.path.isdir(os.path.join(dist_path, "mavros_msgs")) and dist_path not in sys.path:
                    sys.path.insert(0, dist_path)


ensure_mavros_python_path()

import numpy as np
import rospy
from std_msgs.msg import String

from human_follow_msgs.msg import FollowCommand, FollowState, Target2D, Target3D, TrackerStatus
from mavros_msgs.msg import PositionTarget


def sign(value, eps=1e-6):
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


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


class TruthChainRegressionMonitorNode:
    def __init__(self):
        self.detector_topic = rospy.get_param("~detector_topic", "/follow/detector/person_target")
        self.tracker_status_topic = rospy.get_param("~tracker_status_topic", "/follow/tracker/status")
        self.follow_state_topic = rospy.get_param("~follow_state_topic", "/follow/state")
        self.follow_command_topic = rospy.get_param("~follow_command_topic", "/follow/control/cmd_body")
        self.tracker_target_topic = rospy.get_param("~tracker_target_topic", "/follow/tracker/selected_target")
        self.target_body_topic = rospy.get_param("~target_body_topic", "/follow/fusion/target_body")
        self.truth_target_topic = rospy.get_param("~truth_target_topic", "/follow/sim/truth_target_body")
        self.truth_phase_topic = rospy.get_param("~truth_phase_topic", "/follow/sim/truth_phase")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "/follow/offboard/setpoint")
        self.check_rate_hz = float(rospy.get_param("~check_rate_hz", 20.0))
        self.startup_grace_sec = float(rospy.get_param("~startup_grace_sec", 0.5))
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 20.0))
        self.max_msg_age_sec = float(rospy.get_param("~max_msg_age_sec", 0.5))
        self.max_command_age_sec = float(rospy.get_param("~max_command_age_sec", 0.4))
        self.min_abs_target_y_for_sign_check = float(rospy.get_param("~min_abs_target_y_for_sign_check", 0.05))
        self.min_timeout_count = int(rospy.get_param("~min_timeout_count", 1))
        self.min_reacquire_count = int(rospy.get_param("~min_reacquire_count", 1))
        self.require_idle_state = bool(rospy.get_param("~require_idle_state", True))
        self.require_hold_state = bool(rospy.get_param("~require_hold_state", True))
        self.output_frame_id = rospy.get_param("~output_frame_id", "base_link")
        self.flip_lateral_sign = bool(rospy.get_param("~flip_lateral_sign", True))
        self.flip_yaw_rate_sign = bool(rospy.get_param("~flip_yaw_rate_sign", True))
        self.forward_scale = float(rospy.get_param("~forward_scale", 1.0))
        self.lateral_scale = float(rospy.get_param("~lateral_scale", 1.0))
        self.yaw_rate_scale = float(rospy.get_param("~yaw_rate_scale", 1.0))
        self.velocity_tolerance = float(rospy.get_param("~velocity_tolerance", 1e-5))
        self.yaw_rate_tolerance = float(rospy.get_param("~yaw_rate_tolerance", 1e-5))
        self.max_truth_position_error_m = float(rospy.get_param("~max_truth_position_error_m", 0.35))
        self.min_truth_visible_success_count = int(rospy.get_param("~min_truth_visible_success_count", 8))
        self.min_truth_hidden_invalid_count = int(rospy.get_param("~min_truth_hidden_invalid_count", 4))
        self.required_visible_phase_labels = list(
            rospy.get_param("~required_visible_phase_labels", ["approach_center", "reacquire_right", "reacquire_left"])
        )
        self.required_hidden_phase_labels = list(
            rospy.get_param("~required_hidden_phase_labels", ["short_occlusion", "long_loss"])
        )
        self.require_detector_valid_detection = bool(rospy.get_param("~require_detector_valid_detection", False))
        self.min_detector_valid_count = int(rospy.get_param("~min_detector_valid_count", 1))
        self.required_detector_source_token = rospy.get_param("~required_detector_source_token", "").strip()
        self.required_tracker_source_token = rospy.get_param("~required_tracker_source_token", "").strip()

        self.start_time = time.monotonic()
        self.finished = False
        self.exit_code = 0

        self.last_detector_target = None
        self.last_detector_target_time = None
        self.last_tracker_status = None
        self.last_tracker_status_time = None
        self.last_follow_state = None
        self.last_follow_state_time = None
        self.last_tracker_target = None
        self.last_tracker_target_time = None
        self.last_target_body = None
        self.last_target_body_time = None
        self.last_valid_target_body = None
        self.last_valid_target_body_time = None
        self.last_truth_target = None
        self.last_truth_target_time = None
        self.last_truth_phase = parse_phase_descriptor("")
        self.last_truth_phase_time = None
        self.recent_commands = deque()

        self.seen_tracker_states = set()
        self.seen_follow_states = set()
        self.follow_yaw_sign_match_seen = False
        self.search_yaw_sign_match_seen = False
        self.follow_nonzero_command_seen = False
        self.search_command_seen = False
        self.reacquire_right_seen = False
        self.reacquire_left_seen = False
        self.bridge_track_right_seen = False
        self.bridge_track_left_seen = False
        self.bridge_search_seen = False
        self.bridge_hold_seen = False
        self.detector_valid_count = 0
        self.detector_source_token_seen = not bool(self.required_detector_source_token)
        self.tracker_source_token_seen = not bool(self.required_tracker_source_token)

        self.truth_phase_labels_seen = set()
        self.truth_visible_phase_success_labels = set()
        self.truth_hidden_phase_success_labels = set()
        self.truth_visible_success_count = 0
        self.truth_hidden_invalid_count = 0
        self.max_seen_truth_error_m = 0.0

        self.expected_type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )

        self.detector_target_sub = rospy.Subscriber(
            self.detector_topic, Target2D, self._detector_target_callback, queue_size=20
        )
        self.tracker_status_sub = rospy.Subscriber(
            self.tracker_status_topic, TrackerStatus, self._tracker_status_callback, queue_size=20
        )
        self.follow_state_sub = rospy.Subscriber(
            self.follow_state_topic, FollowState, self._follow_state_callback, queue_size=20
        )
        self.follow_command_sub = rospy.Subscriber(
            self.follow_command_topic, FollowCommand, self._follow_command_callback, queue_size=20
        )
        self.tracker_target_sub = rospy.Subscriber(
            self.tracker_target_topic, Target2D, self._tracker_target_callback, queue_size=20
        )
        self.target_body_sub = rospy.Subscriber(
            self.target_body_topic, Target3D, self._target_body_callback, queue_size=20
        )
        self.truth_target_sub = rospy.Subscriber(
            self.truth_target_topic, Target3D, self._truth_target_callback, queue_size=20
        )
        self.truth_phase_sub = rospy.Subscriber(
            self.truth_phase_topic, String, self._truth_phase_callback, queue_size=20
        )
        self.setpoint_sub = rospy.Subscriber(
            self.setpoint_topic, PositionTarget, self._setpoint_callback, queue_size=20
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.check_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "truth_chain_regression_monitor ready: detector=%s truth=%s phase=%s target_body=%s setpoint=%s",
            self.detector_topic,
            self.truth_target_topic,
            self.truth_phase_topic,
            self.target_body_topic,
            self.setpoint_topic,
        )

    def _detector_target_callback(self, msg):
        self.last_detector_target = msg
        self.last_detector_target_time = time.monotonic()
        if msg.valid:
            self.detector_valid_count += 1
            if self.required_detector_source_token and self.required_detector_source_token in (msg.source or ""):
                self.detector_source_token_seen = True

    def _tracker_status_callback(self, msg):
        self.last_tracker_status = msg
        self.last_tracker_status_time = time.monotonic()
        if msg.tracking_state:
            self.seen_tracker_states.add(msg.tracking_state)

    def _follow_state_callback(self, msg):
        self.last_follow_state = msg
        self.last_follow_state_time = time.monotonic()
        if msg.state_name:
            self.seen_follow_states.add(msg.state_name)

    def _tracker_target_callback(self, msg):
        self.last_tracker_target = msg
        self.last_tracker_target_time = time.monotonic()
        source = msg.source or ""
        if self.required_tracker_source_token and self.required_tracker_source_token in source:
            self.tracker_source_token_seen = True
        if "reacquire_right" in source:
            self.reacquire_right_seen = True
        if "reacquire_left" in source:
            self.reacquire_left_seen = True

    def _truth_target_callback(self, msg):
        self.last_truth_target = msg
        self.last_truth_target_time = time.monotonic()

    def _truth_phase_callback(self, msg):
        self.last_truth_phase = parse_phase_descriptor(msg.data)
        self.last_truth_phase_time = time.monotonic()
        label = self.last_truth_phase.get("label", "")
        if label:
            self.truth_phase_labels_seen.add(label)

    def _is_recent(self, stamp, now):
        if stamp is None:
            return False
        return (now - stamp) <= self.max_msg_age_sec

    def _tracker_source_matches_phase(self, phase_label, now):
        if not phase_label or self.last_tracker_target is None:
            return False
        if not self._is_recent(self.last_tracker_target_time, now):
            return False
        return phase_label in (self.last_tracker_target.source or "")

    def _follow_command_callback(self, msg):
        now = time.monotonic()
        self.recent_commands.append((now, msg))
        self._prune_old_commands(now)

        if msg.mode == "body_velocity_yaw_rate" and msg.valid:
            if abs(msg.forward_mps) > 1e-3 or abs(msg.lateral_mps) > 1e-3 or abs(msg.yaw_rate_rps) > 1e-3:
                self.follow_nonzero_command_seen = True

            if self.last_target_body and self._is_recent(self.last_target_body_time, now) and self.last_target_body.valid:
                target_sign = sign(self.last_target_body.position.y, self.min_abs_target_y_for_sign_check)
                yaw_sign = sign(msg.yaw_rate_rps)
                if target_sign != 0 and yaw_sign == target_sign:
                    self.follow_yaw_sign_match_seen = True

        if msg.mode == "search_yaw_only" and msg.valid:
            self.search_command_seen = True
            if self.last_valid_target_body and self._is_recent(self.last_valid_target_body_time, now):
                target_sign = sign(self.last_valid_target_body.position.y, self.min_abs_target_y_for_sign_check)
                yaw_sign = sign(msg.yaw_rate_rps)
                if target_sign != 0 and yaw_sign == target_sign:
                    self.search_yaw_sign_match_seen = True

    def _prune_old_commands(self, now):
        while self.recent_commands and (now - self.recent_commands[0][0]) > self.max_command_age_sec:
            self.recent_commands.popleft()

    def _close(self, a, b, tol):
        return math.isclose(float(a), float(b), abs_tol=tol)

    def _mapped_lateral(self, value):
        scale = -1.0 if self.flip_lateral_sign else 1.0
        return float(value) * self.lateral_scale * scale

    def _mapped_yaw_rate(self, value):
        scale = -1.0 if self.flip_yaw_rate_sign else 1.0
        return float(value) * self.yaw_rate_scale * scale

    def _match_command(self, setpoint_msg, cmd):
        if cmd.valid and cmd.mode == "body_velocity_yaw_rate":
            expected_vx = float(cmd.forward_mps) * self.forward_scale
            expected_vy = self._mapped_lateral(cmd.lateral_mps)
            expected_yaw_rate = self._mapped_yaw_rate(cmd.yaw_rate_rps)
            if (
                self._close(setpoint_msg.velocity.x, expected_vx, self.velocity_tolerance)
                and self._close(setpoint_msg.velocity.y, expected_vy, self.velocity_tolerance)
                and self._close(setpoint_msg.yaw_rate, expected_yaw_rate, self.yaw_rate_tolerance)
            ):
                if cmd.lateral_mps > 0.0:
                    self.bridge_track_left_seen = True
                if cmd.lateral_mps < 0.0:
                    self.bridge_track_right_seen = True
                return True
            return False

        if cmd.valid and cmd.mode == "search_yaw_only":
            expected_yaw_rate = self._mapped_yaw_rate(cmd.yaw_rate_rps)
            if (
                self._close(setpoint_msg.velocity.x, 0.0, self.velocity_tolerance)
                and self._close(setpoint_msg.velocity.y, 0.0, self.velocity_tolerance)
                and self._close(setpoint_msg.yaw_rate, expected_yaw_rate, self.yaw_rate_tolerance)
            ):
                self.bridge_search_seen = True
                return True
            return False

        if (
            self._close(setpoint_msg.velocity.x, 0.0, self.velocity_tolerance)
            and self._close(setpoint_msg.velocity.y, 0.0, self.velocity_tolerance)
            and self._close(setpoint_msg.yaw_rate, 0.0, self.yaw_rate_tolerance)
        ):
            self.bridge_hold_seen = True
            return True
        return False

    def _evaluate_truth_constraints(self, msg, now):
        if not self._is_recent(self.last_truth_phase_time, now) or not self._is_recent(self.last_truth_target_time, now):
            return

        label = self.last_truth_phase.get("label", "")
        phase_visible = bool(self.last_truth_phase.get("visible", False))
        truth_valid = bool(self.last_truth_phase.get("truth_valid", False)) and bool(self.last_truth_target.valid)

        if phase_visible and truth_valid:
            if not msg.valid:
                return
            if not self._tracker_source_matches_phase(label, now):
                return
            if msg.frame_id != self.last_truth_target.frame_id:
                self._fail("truth frame mismatch: fused=%s truth=%s" % (msg.frame_id, self.last_truth_target.frame_id))
                return

            fused = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=np.float64)
            truth = np.array(
                [self.last_truth_target.position.x, self.last_truth_target.position.y, self.last_truth_target.position.z],
                dtype=np.float64,
            )
            error = float(np.linalg.norm(fused - truth))
            self.max_seen_truth_error_m = max(self.max_seen_truth_error_m, error)
            if error <= self.max_truth_position_error_m:
                self.truth_visible_success_count += 1
                self.truth_visible_phase_success_labels.add(label)
        elif label and label != "idle_boot":
            if not msg.valid:
                self.truth_hidden_invalid_count += 1
                self.truth_hidden_phase_success_labels.add(label)

    def _target_body_callback(self, msg):
        now = time.monotonic()
        self.last_target_body = msg
        self.last_target_body_time = now
        if msg.valid:
            self.last_valid_target_body = msg
            self.last_valid_target_body_time = now
        self._evaluate_truth_constraints(msg, now)

    def _fail(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 1
        rospy.logerr("truth_chain regression FAIL reason=%s", reason)
        rospy.signal_shutdown(reason)

    def _pass(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0
        rospy.loginfo(
            "truth_chain regression PASS tracker_states=%s follow_states=%s truth_visible=%d truth_hidden=%d max_error=%.3f",
            sorted(self.seen_tracker_states),
            sorted(self.seen_follow_states),
            self.truth_visible_success_count,
            self.truth_hidden_invalid_count,
            self.max_seen_truth_error_m,
        )
        rospy.signal_shutdown(reason)

    def _setpoint_callback(self, msg):
        if self.finished:
            return

        now = time.monotonic()
        self._prune_old_commands(now)
        if not self.recent_commands:
            return

        if msg.coordinate_frame != PositionTarget.FRAME_BODY_NED:
            self._fail("coordinate_frame mismatch")
            return
        if msg.type_mask != self.expected_type_mask:
            self._fail("type_mask mismatch: %d" % msg.type_mask)
            return
        if msg.header.frame_id != self.output_frame_id:
            self._fail("frame_id mismatch: %s" % msg.header.frame_id)
            return
        if not self._close(msg.velocity.z, 0.0, self.velocity_tolerance):
            self._fail("velocity.z not zero")
            return

        for _stamp, cmd in reversed(self.recent_commands):
            if self._match_command(msg, cmd):
                return

        self._fail(
            "setpoint did not match any recent command vx=%.3f vy=%.3f yaw_rate=%.3f recent=%d"
            % (msg.velocity.x, msg.velocity.y, msg.yaw_rate, len(self.recent_commands))
        )

    def _missing_requirements(self):
        missing = []

        for tracker_state in ("valid", "timeout"):
            if tracker_state not in self.seen_tracker_states:
                missing.append("tracker_state:%s" % tracker_state)
        if self.require_hold_state and "hold" not in self.seen_tracker_states:
            missing.append("tracker_state:hold")

        for state_name in ("target_acquired", "follow", "search", "lost"):
            if state_name not in self.seen_follow_states:
                missing.append("follow_state:%s" % state_name)
        if self.require_idle_state and "idle" not in self.seen_follow_states:
            missing.append("follow_state:idle")

        if not self.follow_nonzero_command_seen:
            missing.append("follow_command:nonzero")
        if not self.search_command_seen:
            missing.append("search_command:seen")
        if not self.follow_yaw_sign_match_seen:
            missing.append("follow_yaw_sign:match")
        if not self.search_yaw_sign_match_seen:
            missing.append("search_yaw_sign:match")
        if not self.reacquire_right_seen:
            missing.append("phase:reacquire_right")
        if not self.reacquire_left_seen:
            missing.append("phase:reacquire_left")
        if self.require_detector_valid_detection and self.detector_valid_count < self.min_detector_valid_count:
            missing.append("detector_valid_count<%d" % self.min_detector_valid_count)
        if self.required_detector_source_token and not self.detector_source_token_seen:
            missing.append("detector_source_token:%s" % self.required_detector_source_token)
        if self.required_tracker_source_token and not self.tracker_source_token_seen:
            missing.append("tracker_source_token:%s" % self.required_tracker_source_token)

        if not self.bridge_track_right_seen:
            missing.append("bridge:track_right")
        if not self.bridge_track_left_seen:
            missing.append("bridge:track_left")
        if not self.bridge_search_seen:
            missing.append("bridge:search")
        if not self.bridge_hold_seen:
            missing.append("bridge:hold")

        tracker_status = self.last_tracker_status
        if tracker_status is None:
            missing.append("tracker_status:missing")
        else:
            if tracker_status.timeout_count < self.min_timeout_count:
                missing.append("timeout_count<%d" % self.min_timeout_count)
            if tracker_status.reacquire_count < self.min_reacquire_count:
                missing.append("reacquire_count<%d" % self.min_reacquire_count)

        if self.truth_visible_success_count < self.min_truth_visible_success_count:
            missing.append("truth_visible_success_count<%d" % self.min_truth_visible_success_count)
        if self.truth_hidden_invalid_count < self.min_truth_hidden_invalid_count:
            missing.append("truth_hidden_invalid_count<%d" % self.min_truth_hidden_invalid_count)

        for label in self.required_visible_phase_labels:
            if label not in self.truth_visible_phase_success_labels:
                missing.append("truth_visible_phase:%s" % label)

        for label in self.required_hidden_phase_labels:
            if label not in self.truth_hidden_phase_success_labels:
                missing.append("truth_hidden_phase:%s" % label)

        return missing

    def _tick(self, _event):
        if self.finished:
            return

        now = time.monotonic()
        elapsed_sec = now - self.start_time
        if elapsed_sec < self.startup_grace_sec:
            return

        missing = self._missing_requirements()
        if not missing:
            self._pass("all truth-chain requirements satisfied")
            return

        if elapsed_sec >= self.max_duration_sec:
            self._fail("missing=" + ",".join(missing))


def main():
    rospy.init_node("truth_chain_regression_monitor")
    node = TruthChainRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
