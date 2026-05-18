#!/usr/bin/env python3
import copy
import math

import rospy

from human_follow_msgs.msg import Target2D, TrackerStatus


TRACKER_STATE_VALID = "valid"
TRACKER_STATE_HOLD = "hold"
TRACKER_STATE_TIMEOUT = "timeout"


class TrackerNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/detector/person_target")
        self.output_topic = rospy.get_param("~output_topic", "/follow/tracker/selected_target")
        self.status_topic = rospy.get_param("~status_topic", "/follow/tracker/status")
        self.default_frame_id = rospy.get_param("~default_frame_id", "camera")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.live_timeout_sec = float(rospy.get_param("~live_timeout_sec", 0.2))
        self.hold_timeout_sec = float(rospy.get_param("~hold_timeout_sec", 0.5))
        self.min_hold_confidence = float(rospy.get_param("~min_hold_confidence", 0.15))
        self.min_consecutive_valid_inputs = max(1, int(rospy.get_param("~min_consecutive_valid_inputs", 1)))
        self.status_log_period_sec = float(rospy.get_param("~status_log_period_sec", 1.0))

        if self.hold_timeout_sec < self.live_timeout_sec:
            rospy.logwarn(
                "tracker hold_timeout_sec %.3f < live_timeout_sec %.3f, clamping hold timeout",
                self.hold_timeout_sec,
                self.live_timeout_sec,
            )
            self.hold_timeout_sec = self.live_timeout_sec

        self.last_valid_target = None
        self.last_valid_stamp = None
        self.last_observed_frame_id = self.default_frame_id
        self.candidate_target = None
        self.current_state = None
        self.current_detail = ""
        self.has_ever_emitted_valid_state = False
        self.valid_input_streak = 0
        self.invalid_input_streak = 0
        self.reacquire_count = 0
        self.timeout_count = 0
        self.last_status_log_stamp = None

        self.publisher = rospy.Publisher(self.output_topic, Target2D, queue_size=10)
        self.status_publisher = rospy.Publisher(self.status_topic, TrackerStatus, queue_size=10)
        self.subscriber = rospy.Subscriber(self.input_topic, Target2D, self._target_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._publish)

        rospy.loginfo(
            "human_follow_tracker ready: input=%s output=%s status=%s",
            self.input_topic,
            self.output_topic,
            self.status_topic,
        )

    def _target_callback(self, msg):
        now = rospy.Time.now()
        if msg.header.frame_id:
            self.last_observed_frame_id = msg.header.frame_id
        if msg.valid:
            self.valid_input_streak += 1
            self.invalid_input_streak = 0
            self.candidate_target = copy.deepcopy(msg)
            if self.valid_input_streak >= self.min_consecutive_valid_inputs:
                self.last_valid_target = copy.deepcopy(msg)
                self.last_valid_stamp = now
        else:
            self.invalid_input_streak += 1
            self.valid_input_streak = 0
            self.candidate_target = None

    def _classify_state(self, now):
        if self.last_valid_target is None or self.last_valid_stamp is None:
            if self.valid_input_streak > 0 and self.candidate_target is not None:
                return (
                    TRACKER_STATE_TIMEOUT,
                    "warming_up_valid_target_%d_of_%d"
                    % (self.valid_input_streak, self.min_consecutive_valid_inputs),
                    math.inf,
                )
            return TRACKER_STATE_TIMEOUT, "no_valid_target_yet", math.inf

        age = max(0.0, (now - self.last_valid_stamp).to_sec())
        if age <= self.live_timeout_sec:
            return TRACKER_STATE_VALID, "detector_target_fresh", age
        if age <= self.hold_timeout_sec:
            return TRACKER_STATE_HOLD, "holding_last_valid_target", age
        return TRACKER_STATE_TIMEOUT, "hold_timeout_expired", age

    def _compose_output_source(self, state, target):
        detector_source = target.source or "detector"
        return "tracker_%s:%s" % (state, detector_source)

    def _decay_confidence_for_hold(self, base_confidence, age):
        if age <= self.live_timeout_sec:
            return max(0.0, float(base_confidence))

        hold_window = max(self.hold_timeout_sec - self.live_timeout_sec, 1e-3)
        hold_age = min(max(age - self.live_timeout_sec, 0.0), hold_window)
        alpha = 1.0 - (hold_age / hold_window)
        decayed = max(0.0, float(base_confidence)) * alpha
        return min(max(0.0, float(base_confidence)), max(0.0, self.min_hold_confidence, decayed))

    def _resolve_output_frame_id(self):
        if self.last_valid_target is not None and self.last_valid_target.header.frame_id:
            return self.last_valid_target.header.frame_id
        if self.candidate_target is not None and self.candidate_target.header.frame_id:
            return self.candidate_target.header.frame_id
        return self.last_observed_frame_id or self.default_frame_id

    def _make_timeout_target(self, now):
        out = Target2D()
        out.header.stamp = now
        out.header.frame_id = self._resolve_output_frame_id()
        out.track_id = -1
        out.valid = False
        out.confidence = 0.0
        out.cx = 0.0
        out.cy = 0.0
        out.width = 0.0
        out.height = 0.0
        out.source = "tracker_timeout"
        return out

    def _handle_state_transition(self, new_state, detail, age):
        old_state = self.current_state
        if new_state == old_state and detail == self.current_detail:
            return

        if new_state == TRACKER_STATE_VALID:
            if old_state in (TRACKER_STATE_HOLD, TRACKER_STATE_TIMEOUT) and self.has_ever_emitted_valid_state:
                self.reacquire_count += 1
            self.has_ever_emitted_valid_state = True
        elif new_state == TRACKER_STATE_TIMEOUT and old_state in (TRACKER_STATE_VALID, TRACKER_STATE_HOLD):
            self.timeout_count += 1

        rospy.loginfo(
            "tracker state %s -> %s age=%.3f detail=%s valid_streak=%d invalid_streak=%d reacquire=%d timeout=%d",
            old_state or "none",
            new_state,
            age if math.isfinite(age) else -1.0,
            detail,
            self.valid_input_streak,
            self.invalid_input_streak,
            self.reacquire_count,
            self.timeout_count,
        )
        self.current_state = new_state
        self.current_detail = detail

    def _build_status(self, now, state, detail, age, output_target):
        status = TrackerStatus()
        status.header.stamp = now
        status.tracking_state = state
        status.detail = detail
        status.track_id = output_target.track_id
        status.output_valid = output_target.valid
        status.output_confidence = output_target.confidence
        status.age_since_valid_sec = age if math.isfinite(age) else -1.0
        status.remaining_hold_sec = (
            max(0.0, self.hold_timeout_sec - age) if math.isfinite(age) else 0.0
        )
        status.valid_input_streak = int(self.valid_input_streak)
        status.invalid_input_streak = int(self.invalid_input_streak)
        status.reacquire_count = int(self.reacquire_count)
        status.timeout_count = int(self.timeout_count)
        status.output_source = output_target.source
        return status

    def _maybe_log_status(self, now, status):
        if self.status_log_period_sec <= 0.0:
            return

        if self.last_status_log_stamp is None:
            should_log = True
        else:
            should_log = (now - self.last_status_log_stamp).to_sec() >= self.status_log_period_sec

        if not should_log:
            return

        rospy.loginfo(
            "tracker status state=%s age=%.3f hold_left=%.3f output_valid=%s conf=%.2f reacquire=%d timeout=%d",
            status.tracking_state,
            status.age_since_valid_sec,
            status.remaining_hold_sec,
            status.output_valid,
            status.output_confidence,
            status.reacquire_count,
            status.timeout_count,
        )
        self.last_status_log_stamp = now

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        state, detail, age = self._classify_state(now)
        self._handle_state_transition(state, detail, age)

        if state == TRACKER_STATE_VALID and self.last_valid_target is not None:
            out = copy.deepcopy(self.last_valid_target)
            out.header.stamp = now
            out.source = self._compose_output_source(TRACKER_STATE_VALID, self.last_valid_target)
        elif state == TRACKER_STATE_HOLD and self.last_valid_target is not None:
            out = copy.deepcopy(self.last_valid_target)
            out.header.stamp = now
            out.confidence = self._decay_confidence_for_hold(out.confidence, age)
            out.source = self._compose_output_source(TRACKER_STATE_HOLD, self.last_valid_target)
        else:
            out = self._make_timeout_target(now)

        status = self._build_status(now, state, detail, age, out)
        try:
            self.publisher.publish(out)
            self.status_publisher.publish(status)
        except rospy.ROSException:
            return
        self._maybe_log_status(now, status)


if __name__ == "__main__":
    rospy.init_node("human_follow_tracker")
    TrackerNode()
    rospy.spin()
