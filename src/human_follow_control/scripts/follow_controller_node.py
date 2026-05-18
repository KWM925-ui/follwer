#!/usr/bin/env python3
import math

import rospy

from human_follow_msgs.msg import FollowCommand, FollowState, Target3D


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def apply_deadband(value, deadband):
    if deadband <= 0.0:
        return float(value)
    if abs(value) <= deadband:
        return 0.0
    return float(value) - math.copysign(float(deadband), float(value))


STATE_NAME = {
    FollowState.IDLE: "idle",
    FollowState.TARGET_ACQUIRED: "target_acquired",
    FollowState.FOLLOW: "follow",
    FollowState.SEARCH: "search",
    FollowState.LOST: "lost",
}


class FollowControllerNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/fusion/target_body")
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.state_topic = rospy.get_param("~state_topic", "/follow/state")
        self.rate_hz = rospy.get_param("~rate_hz", 20.0)
        self.desired_distance_m = rospy.get_param("~desired_distance_m", 4.0)
        self.target_fresh_timeout_sec = rospy.get_param("~target_fresh_timeout_sec", 0.25)
        self.acquire_state_hold_sec = rospy.get_param("~acquire_state_hold_sec", 0.5)
        self.kp_forward = rospy.get_param("~kp_forward", 0.6)
        self.kp_lateral = rospy.get_param("~kp_lateral", 0.6)
        self.kp_yaw = rospy.get_param("~kp_yaw", 0.4)
        self.ki_forward = rospy.get_param("~ki_forward", 0.0)
        self.ki_lateral = rospy.get_param("~ki_lateral", 0.0)
        self.ki_yaw = rospy.get_param("~ki_yaw", 0.0)
        self.kd_forward = rospy.get_param("~kd_forward", 0.0)
        self.kd_lateral = rospy.get_param("~kd_lateral", 0.0)
        self.kd_yaw = rospy.get_param("~kd_yaw", 0.0)
        self.max_forward_mps = rospy.get_param("~max_forward_mps", 1.0)
        self.max_lateral_mps = rospy.get_param("~max_lateral_mps", 0.8)
        self.max_yaw_rate_rps = rospy.get_param("~max_yaw_rate_rps", 0.5)
        self.forward_integral_limit = rospy.get_param("~forward_integral_limit", 1.2)
        self.lateral_integral_limit = rospy.get_param("~lateral_integral_limit", 1.0)
        self.yaw_integral_limit = rospy.get_param("~yaw_integral_limit", 0.8)
        self.distance_deadband_m = rospy.get_param("~distance_deadband_m", 0.03)
        self.lateral_deadband_m = rospy.get_param("~lateral_deadband_m", 0.03)
        self.yaw_deadband_m = rospy.get_param("~yaw_deadband_m", 0.02)
        self.search_timeout_sec = rospy.get_param("~search_timeout_sec", 2.0)
        self.lost_timeout_sec = rospy.get_param("~lost_timeout_sec", 5.0)
        self.search_yaw_rate_rps = rospy.get_param("~search_yaw_rate_rps", 0.25)
        self.search_sweep_span_deg = rospy.get_param("~search_sweep_span_deg", 0.0)
        self.search_sweep_pass_count = max(int(rospy.get_param("~search_sweep_pass_count", 0)), 0)
        self.search_return_to_center = bool(rospy.get_param("~search_return_to_center", False))

        self.last_valid_target = None
        self.last_valid_stamp = None
        self.last_seen_lateral_sign = 1.0
        self.acquire_stamp = None
        self.state = FollowState.IDLE
        self.last_logged_state = None
        self.last_control_stamp = None
        self.forward_integral = 0.0
        self.lateral_integral = 0.0
        self.yaw_integral = 0.0
        self.last_forward_error = 0.0
        self.last_lateral_error = 0.0
        self.last_yaw_error = 0.0

        self.command_pub = rospy.Publisher(self.command_topic, FollowCommand, queue_size=10)
        self.state_pub = rospy.Publisher(self.state_topic, FollowState, queue_size=10)
        self.subscriber = rospy.Subscriber(self.input_topic, Target3D, self._target_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1e-3)), self._tick)

        rospy.loginfo("human_follow_controller ready: input=%s command=%s", self.input_topic, self.command_topic)
        if self._search_sweep_enabled():
            rospy.loginfo(
                "human_follow_controller search profile: mode=sweep fresh_timeout=%.2f acquire_hold=%.2f search_timeout=%.2f lost_timeout=%.2f yaw_rate=%.2f span_deg=%.1f passes=%d return_to_center=%s",
                self.target_fresh_timeout_sec,
                self.acquire_state_hold_sec,
                self.search_timeout_sec,
                self.lost_timeout_sec,
                self.search_yaw_rate_rps,
                self.search_sweep_span_deg,
                self.search_sweep_pass_count,
                "true" if self.search_return_to_center else "false",
            )
        else:
            rospy.loginfo(
                "human_follow_controller search profile: mode=legacy_single_direction fresh_timeout=%.2f acquire_hold=%.2f search_timeout=%.2f lost_timeout=%.2f yaw_rate=%.2f",
                self.target_fresh_timeout_sec,
                self.acquire_state_hold_sec,
                self.search_timeout_sec,
                self.lost_timeout_sec,
                self.search_yaw_rate_rps,
            )

    def _target_callback(self, msg):
        if msg.valid:
            now = rospy.Time.now()
            if self.last_valid_stamp is None or (now - self.last_valid_stamp).to_sec() > self.target_fresh_timeout_sec:
                self.acquire_stamp = now
            self.last_valid_target = msg
            self.last_valid_stamp = now
            if abs(msg.position.y) > 1e-3:
                self.last_seen_lateral_sign = 1.0 if msg.position.y > 0.0 else -1.0

    def _current_state(self, now):
        if self.last_valid_target and self.last_valid_stamp:
            age = (now - self.last_valid_stamp).to_sec()
            if age <= self.target_fresh_timeout_sec:
                if self.acquire_stamp and (now - self.acquire_stamp).to_sec() <= self.acquire_state_hold_sec:
                    return FollowState.TARGET_ACQUIRED, "fresh valid target"
                return FollowState.FOLLOW, "tracking valid target"
            lost_age = max(0.0, age - self.target_fresh_timeout_sec)
            if lost_age <= self.search_timeout_sec:
                return FollowState.SEARCH, "target temporarily lost"
            if lost_age <= self.search_timeout_sec + self.lost_timeout_sec:
                return FollowState.LOST, "target lost, waiting for reacquire"
        return FollowState.IDLE, "no active target"

    def _search_sweep_enabled(self):
        return (
            abs(float(self.search_yaw_rate_rps)) > 1e-6
            and max(float(self.search_sweep_span_deg), 0.0) > 1e-6
            and self.search_sweep_pass_count > 0
        )

    def _search_segment_durations_sec(self):
        if not self._search_sweep_enabled():
            return []
        yaw_rate = abs(float(self.search_yaw_rate_rps))
        span_rad = math.radians(max(float(self.search_sweep_span_deg), 0.0))
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
            return float(self.search_yaw_rate_rps) * self.last_seen_lateral_sign
        lost_age = (now - self.last_valid_stamp).to_sec() - self.target_fresh_timeout_sec
        elapsed_sec = max(0.0, float(lost_age))
        durations = self._search_segment_durations_sec()

        base_yaw_rate = abs(float(self.search_yaw_rate_rps))
        direction = 1.0 if self.last_seen_lateral_sign >= 0.0 else -1.0
        for duration_sec in durations:
            if elapsed_sec <= duration_sec:
                return base_yaw_rate * direction
            elapsed_sec -= duration_sec
            direction *= -1.0
        return 0.0

    def _reset_pid_state(self):
        self.last_control_stamp = None
        self.forward_integral = 0.0
        self.lateral_integral = 0.0
        self.yaw_integral = 0.0
        self.last_forward_error = 0.0
        self.last_lateral_error = 0.0
        self.last_yaw_error = 0.0

    def _pid_axis(self, error_value, dt_sec, integral_value, last_error_value, kp, ki, kd, integral_limit, output_limit):
        derivative_value = 0.0
        integral_candidate = float(integral_value)
        if dt_sec > 1e-4:
            derivative_value = (float(error_value) - float(last_error_value)) / float(dt_sec)
            integral_candidate = clamp(
                float(integral_value) + float(error_value) * float(dt_sec),
                -float(integral_limit),
                float(integral_limit),
            )

        raw_output = (
            float(kp) * float(error_value)
            + float(ki) * integral_candidate
            + float(kd) * derivative_value
        )
        output_value = clamp(raw_output, -float(output_limit), float(output_limit))

        if dt_sec > 1e-4 and float(ki) != 0.0:
            saturating_positive = raw_output > float(output_limit) and float(error_value) > 0.0
            saturating_negative = raw_output < -float(output_limit) and float(error_value) < 0.0
            if saturating_positive or saturating_negative:
                integral_candidate = float(integral_value)
                raw_output = (
                    float(kp) * float(error_value)
                    + float(ki) * integral_candidate
                    + float(kd) * derivative_value
                )
                output_value = clamp(raw_output, -float(output_limit), float(output_limit))

        return output_value, integral_candidate

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        state, detail = self._current_state(now)
        self.state = state
        dt_sec = 0.0
        if self.last_control_stamp is not None:
            dt_sec = max(0.0, (now - self.last_control_stamp).to_sec())
        self.last_control_stamp = now

        if state != self.last_logged_state:
            rospy.loginfo("follow_controller state %s detail=%s", STATE_NAME[state], detail)
            self.last_logged_state = state

        command = FollowCommand()
        command.header.stamp = now
        command.mode = "hold"
        command.valid = False

        if state in (FollowState.TARGET_ACQUIRED, FollowState.FOLLOW) and self.last_valid_target:
            target = self.last_valid_target
            forward_error = apply_deadband(target.position.x - self.desired_distance_m, self.distance_deadband_m)
            lateral_error = apply_deadband(target.position.y, self.lateral_deadband_m)
            yaw_error = apply_deadband(target.position.y, self.yaw_deadband_m)

            command.forward_mps, self.forward_integral = self._pid_axis(
                forward_error,
                dt_sec,
                self.forward_integral,
                self.last_forward_error,
                self.kp_forward,
                self.ki_forward,
                self.kd_forward,
                self.forward_integral_limit,
                self.max_forward_mps,
            )
            command.lateral_mps, self.lateral_integral = self._pid_axis(
                lateral_error,
                dt_sec,
                self.lateral_integral,
                self.last_lateral_error,
                self.kp_lateral,
                self.ki_lateral,
                self.kd_lateral,
                self.lateral_integral_limit,
                self.max_lateral_mps,
            )
            command.yaw_rate_rps, self.yaw_integral = self._pid_axis(
                yaw_error,
                dt_sec,
                self.yaw_integral,
                self.last_yaw_error,
                self.kp_yaw,
                self.ki_yaw,
                self.kd_yaw,
                self.yaw_integral_limit,
                self.max_yaw_rate_rps,
            )
            self.last_forward_error = float(forward_error)
            self.last_lateral_error = float(lateral_error)
            self.last_yaw_error = float(yaw_error)
            command.mode = "body_velocity_yaw_rate"
            command.valid = True
        elif state == FollowState.SEARCH:
            self._reset_pid_state()
            command.forward_mps = 0.0
            command.lateral_mps = 0.0
            command.yaw_rate_rps = self._search_yaw_rate_command(now)
            command.mode = "search_yaw_only"
            command.valid = True
        else:
            self._reset_pid_state()
            command.forward_mps = 0.0
            command.lateral_mps = 0.0
            command.yaw_rate_rps = 0.0

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
    FollowControllerNode()
    rospy.spin()
