#!/usr/bin/env python3
import math
import sys
import time

import rospy

from human_follow_msgs.msg import FollowCommand, FollowState, Target2D, Target3D, TrackerStatus


def sign(value, eps=1e-6):
    if value > eps:
        return 1
    if value < -eps:
        return -1
    return 0


class SequenceRegressionMonitorNode:
    def __init__(self):
        self.tracker_status_topic = rospy.get_param("~tracker_status_topic", "/follow/tracker/status")
        self.follow_state_topic = rospy.get_param("~follow_state_topic", "/follow/state")
        self.follow_command_topic = rospy.get_param("~follow_command_topic", "/follow/control/cmd_body")
        self.tracker_target_topic = rospy.get_param("~tracker_target_topic", "/follow/tracker/selected_target")
        self.target_body_topic = rospy.get_param("~target_body_topic", "/follow/fusion/target_body")
        self.check_rate_hz = float(rospy.get_param("~check_rate_hz", 20.0))
        self.startup_grace_sec = float(rospy.get_param("~startup_grace_sec", 0.5))
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 20.0))
        self.max_msg_age_sec = float(rospy.get_param("~max_msg_age_sec", 0.5))
        self.min_abs_target_y_for_sign_check = float(rospy.get_param("~min_abs_target_y_for_sign_check", 0.05))
        self.min_timeout_count = int(rospy.get_param("~min_timeout_count", 1))
        self.min_reacquire_count = int(rospy.get_param("~min_reacquire_count", 1))
        self.require_idle_state = bool(rospy.get_param("~require_idle_state", True))
        self.require_hold_state = bool(rospy.get_param("~require_hold_state", True))

        self.start_time = time.monotonic()
        self.last_tracker_status = None
        self.last_tracker_status_time = None
        self.last_follow_state = None
        self.last_follow_state_time = None
        self.last_follow_command = None
        self.last_follow_command_time = None
        self.last_tracker_target = None
        self.last_tracker_target_time = None
        self.last_target_body = None
        self.last_target_body_time = None
        self.last_valid_target_body = None
        self.last_valid_target_body_time = None

        self.seen_tracker_states = set()
        self.seen_follow_states = set()
        self.follow_yaw_sign_match_seen = False
        self.search_yaw_sign_match_seen = False
        self.follow_nonzero_command_seen = False
        self.search_command_seen = False
        self.reacquire_right_seen = False
        self.reacquire_left_seen = False
        self.failure_reason = None
        self.exit_code = 0
        self.finished = False

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
        self.target_body_sub = rospy.Subscriber(self.target_body_topic, Target3D, self._target_body_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.check_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "sequence_regression_monitor ready: tracker_status=%s follow_state=%s follow_cmd=%s target_body=%s",
            self.tracker_status_topic,
            self.follow_state_topic,
            self.follow_command_topic,
            self.target_body_topic,
        )

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
        if "reacquire_right" in source:
            self.reacquire_right_seen = True
        if "reacquire_left" in source:
            self.reacquire_left_seen = True

    def _target_body_callback(self, msg):
        self.last_target_body = msg
        self.last_target_body_time = time.monotonic()
        if msg.valid:
            self.last_valid_target_body = msg
            self.last_valid_target_body_time = self.last_target_body_time

    def _is_recent(self, stamp, now):
        if stamp is None:
            return False
        return (now - stamp) <= self.max_msg_age_sec

    def _follow_command_callback(self, msg):
        now = time.monotonic()
        self.last_follow_command = msg
        self.last_follow_command_time = now

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

        tracker_status = self.last_tracker_status
        if tracker_status is None:
            missing.append("tracker_status:missing")
        else:
            if tracker_status.timeout_count < self.min_timeout_count:
                missing.append("timeout_count<%d" % self.min_timeout_count)
            if tracker_status.reacquire_count < self.min_reacquire_count:
                missing.append("reacquire_count<%d" % self.min_reacquire_count)

        return missing

    def _finish(self, success, reason):
        if self.finished:
            return

        self.finished = True
        self.exit_code = 0 if success else 1
        if success:
            rospy.loginfo(
                "sequence regression PASS tracker_states=%s follow_states=%s timeout_count=%d reacquire_count=%d",
                sorted(self.seen_tracker_states),
                sorted(self.seen_follow_states),
                self.last_tracker_status.timeout_count if self.last_tracker_status else -1,
                self.last_tracker_status.reacquire_count if self.last_tracker_status else -1,
            )
        else:
            self.failure_reason = reason
            rospy.logerr("sequence regression FAIL reason=%s", reason)

        rospy.signal_shutdown("sequence regression complete")

    def _tick(self, _event):
        if self.finished:
            return

        elapsed_sec = time.monotonic() - self.start_time
        if elapsed_sec < self.startup_grace_sec:
            return

        missing = self._missing_requirements()
        if not missing:
            self._finish(True, "all requirements satisfied")
            return

        if elapsed_sec >= self.max_duration_sec:
            self._finish(False, "missing=" + ",".join(missing))


def main():
    rospy.init_node("sequence_regression_monitor")
    node = SequenceRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
