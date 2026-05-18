#!/usr/bin/env python3
import math

import rospy

from human_follow_msgs.msg import FollowCommand, FollowState, Target3D


STATE_NAME = {
    FollowState.IDLE: "idle",
    FollowState.TARGET_ACQUIRED: "target_acquired",
    FollowState.FOLLOW: "follow",
    FollowState.SEARCH: "search",
    FollowState.LOST: "lost",
}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def apply_deadband(value, deadband):
    if deadband <= 0.0:
        return float(value)
    if abs(value) <= deadband:
        return 0.0
    return float(value) - math.copysign(float(deadband), float(value))


class ControlAlgorithmTemplateNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/fusion/target_body")
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.state_topic = rospy.get_param("~state_topic", "/follow/state")
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.desired_distance_m = float(rospy.get_param("~desired_distance_m", 4.0))
        self.target_fresh_timeout_sec = float(rospy.get_param("~target_fresh_timeout_sec", 0.25))
        self.acquire_state_hold_sec = float(rospy.get_param("~acquire_state_hold_sec", 0.5))
        self.kp_forward = float(rospy.get_param("~kp_forward", 0.85))
        self.kp_lateral = float(rospy.get_param("~kp_lateral", 0.75))
        self.kp_yaw = float(rospy.get_param("~kp_yaw", 0.48))
        self.kd_forward = float(rospy.get_param("~kd_forward", 0.10))
        self.kd_lateral = float(rospy.get_param("~kd_lateral", 0.06))
        self.kd_yaw = float(rospy.get_param("~kd_yaw", 0.04))
        self.max_forward_mps = float(rospy.get_param("~max_forward_mps", 1.0))
        self.max_lateral_mps = float(rospy.get_param("~max_lateral_mps", 0.8))
        self.max_yaw_rate_rps = float(rospy.get_param("~max_yaw_rate_rps", 0.5))
        self.distance_deadband_m = float(rospy.get_param("~distance_deadband_m", 0.03))
        self.lateral_deadband_m = float(rospy.get_param("~lateral_deadband_m", 0.03))
        self.yaw_deadband_m = float(rospy.get_param("~yaw_deadband_m", 0.02))
        self.search_timeout_sec = float(rospy.get_param("~search_timeout_sec", 2.0))
        self.lost_timeout_sec = float(rospy.get_param("~lost_timeout_sec", 5.0))
        self.search_yaw_rate_rps = float(rospy.get_param("~search_yaw_rate_rps", 0.25))
        self.search_sweep_span_deg = float(rospy.get_param("~search_sweep_span_deg", 0.0))
        self.search_sweep_pass_count = max(int(rospy.get_param("~search_sweep_pass_count", 0)), 0)
        self.search_return_to_center = bool(rospy.get_param("~search_return_to_center", False))

        self.last_valid_target = None
        self.last_valid_stamp = None
        self.acquire_stamp = None
        self.last_seen_lateral_sign = 1.0
        self.last_logged_state = None
        self.last_control_stamp = None
        self.last_forward_error = 0.0
        self.last_lateral_error = 0.0
        self.last_yaw_error = 0.0

        self.command_pub = rospy.Publisher(self.command_topic, FollowCommand, queue_size=10)
        self.state_pub = rospy.Publisher(self.state_topic, FollowState, queue_size=10)
        self.target_sub = rospy.Subscriber(self.input_topic, Target3D, self._target_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "control_algorithm_template ready: input=%s command=%s state=%s",
            self.input_topic,
            self.command_topic,
            self.state_topic,
        )

    def _target_callback(self, msg):
        if not msg.valid:
            return
        now = rospy.Time.now()
        if self.last_valid_stamp is None or (now - self.last_valid_stamp).to_sec() > self.target_fresh_timeout_sec:
            self.acquire_stamp = now
        self.last_valid_target = msg
        self.last_valid_stamp = now
        if abs(msg.position.y) > 1e-3:
            self.last_seen_lateral_sign = 1.0 if msg.position.y >= 0.0 else -1.0

    def _current_state(self, now):
        if self.last_valid_target is None or self.last_valid_stamp is None:
            return FollowState.IDLE, "no active target"

        age_sec = (now - self.last_valid_stamp).to_sec()
        if age_sec <= self.target_fresh_timeout_sec:
            if self.acquire_stamp and (now - self.acquire_stamp).to_sec() <= self.acquire_state_hold_sec:
                return FollowState.TARGET_ACQUIRED, "fresh valid target"
            return FollowState.FOLLOW, "tracking valid target"

        lost_age_sec = max(0.0, age_sec - self.target_fresh_timeout_sec)
        if lost_age_sec <= self.search_timeout_sec:
            return FollowState.SEARCH, "target temporarily lost"
        if lost_age_sec <= self.search_timeout_sec + self.lost_timeout_sec:
            return FollowState.LOST, "target lost, waiting for reacquire"
        return FollowState.IDLE, "target stale"

    def _search_sweep_enabled(self):
        return (
            abs(self.search_yaw_rate_rps) > 1e-6
            and self.search_sweep_span_deg > 1e-6
            and self.search_sweep_pass_count > 0
        )

    def _search_segment_durations_sec(self):
        if not self._search_sweep_enabled():
            return []
        yaw_rate = abs(self.search_yaw_rate_rps)
        span_rad = math.radians(self.search_sweep_span_deg)
        half_sweep_sec = 0.5 * span_rad / yaw_rate
        full_sweep_sec = span_rad / yaw_rate
        durations = [half_sweep_sec]
        durations.extend(full_sweep_sec for _ in range(self.search_sweep_pass_count))
        if self.search_return_to_center:
            durations.append(half_sweep_sec)
        return durations

    def _search_yaw_rate_command(self, now):
        if self.last_valid_stamp is None:
            return 0.0
        if not self._search_sweep_enabled():
            return self.search_yaw_rate_rps * self.last_seen_lateral_sign

        elapsed_sec = max(0.0, (now - self.last_valid_stamp).to_sec() - self.target_fresh_timeout_sec)
        direction = 1.0 if self.last_seen_lateral_sign >= 0.0 else -1.0
        base_rate = abs(self.search_yaw_rate_rps)
        for duration_sec in self._search_segment_durations_sec():
            if elapsed_sec <= duration_sec:
                return base_rate * direction
            elapsed_sec -= duration_sec
            direction *= -1.0
        return 0.0

    def _pd_axis(self, error_value, last_error_value, dt_sec, kp, kd, limit_value):
        derivative_value = 0.0
        if dt_sec > 1e-4:
            derivative_value = (float(error_value) - float(last_error_value)) / float(dt_sec)
        raw_output = float(kp) * float(error_value) + float(kd) * derivative_value
        return clamp(raw_output, -float(limit_value), float(limit_value))

    def _make_hold_command(self, stamp):
        command = FollowCommand()
        command.header.stamp = stamp
        command.mode = "hold"
        command.valid = False
        return command

    def _tick(self, _event):
        if rospy.is_shutdown():
            return

        now = rospy.Time.now()
        dt_sec = 0.0
        if self.last_control_stamp is not None:
            dt_sec = max(0.0, (now - self.last_control_stamp).to_sec())
        self.last_control_stamp = now

        state, detail = self._current_state(now)
        if state != self.last_logged_state:
            rospy.loginfo("control_algorithm_template state=%s detail=%s", STATE_NAME[state], detail)
            self.last_logged_state = state

        command = self._make_hold_command(now)
        if state in (FollowState.TARGET_ACQUIRED, FollowState.FOLLOW) and self.last_valid_target is not None:
            target = self.last_valid_target
            forward_error = apply_deadband(target.position.x - self.desired_distance_m, self.distance_deadband_m)
            lateral_error = apply_deadband(target.position.y, self.lateral_deadband_m)
            yaw_error = apply_deadband(target.position.y, self.yaw_deadband_m)

            command.forward_mps = self._pd_axis(
                forward_error,
                self.last_forward_error,
                dt_sec,
                self.kp_forward,
                self.kd_forward,
                self.max_forward_mps,
            )
            command.lateral_mps = self._pd_axis(
                lateral_error,
                self.last_lateral_error,
                dt_sec,
                self.kp_lateral,
                self.kd_lateral,
                self.max_lateral_mps,
            )
            command.yaw_rate_rps = self._pd_axis(
                yaw_error,
                self.last_yaw_error,
                dt_sec,
                self.kp_yaw,
                self.kd_yaw,
                self.max_yaw_rate_rps,
            )
            command.mode = "body_velocity_yaw_rate"
            command.valid = True
            self.last_forward_error = float(forward_error)
            self.last_lateral_error = float(lateral_error)
            self.last_yaw_error = float(yaw_error)
        elif state == FollowState.SEARCH:
            self.last_forward_error = 0.0
            self.last_lateral_error = 0.0
            self.last_yaw_error = 0.0
            command.forward_mps = 0.0
            command.lateral_mps = 0.0
            command.yaw_rate_rps = self._search_yaw_rate_command(now)
            command.mode = "search_yaw_only"
            command.valid = True
        else:
            self.last_forward_error = 0.0
            self.last_lateral_error = 0.0
            self.last_yaw_error = 0.0

        follow_state = FollowState()
        follow_state.header.stamp = now
        follow_state.state = state
        follow_state.state_name = STATE_NAME[state]
        follow_state.detail = detail

        try:
            self.command_pub.publish(command)
            self.state_pub.publish(follow_state)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("human_follow_controller")
    ControlAlgorithmTemplateNode()
    rospy.spin()
