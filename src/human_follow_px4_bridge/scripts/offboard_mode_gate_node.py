#!/usr/bin/env python3
import glob
import math
import os
import sys
import time

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

import rospy

from human_follow_msgs.msg import FollowCommand
from mavros_msgs.msg import EstimatorStatus, PositionTarget, State
from mavros_msgs.srv import SetMode


class OffboardModeGateNode:
    def __init__(self):
        self.input_setpoint_topic = rospy.get_param("~input_setpoint_topic", "/follow/offboard/setpoint")
        self.output_setpoint_topic = rospy.get_param("~output_setpoint_topic", "/mavros/setpoint_raw/local")
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.eligibility_source = rospy.get_param("~eligibility_source", "command").strip().lower()
        self.mavros_state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")
        self.estimator_status_topic = rospy.get_param("~estimator_status_topic", "/mavros/estimator_status")
        self.set_mode_service = rospy.get_param("~set_mode_service", "/mavros/set_mode")
        self.require_armed = bool(rospy.get_param("~require_armed", True))
        self.min_entry_command_norm = float(rospy.get_param("~min_entry_command_norm", 0.05))
        self.min_entry_setpoint_norm = float(rospy.get_param("~min_entry_setpoint_norm", 0.05))
        self.min_eligible_setpoints = int(rospy.get_param("~min_eligible_setpoints", 10))
        self.mode_request_retry_sec = float(rospy.get_param("~mode_request_retry_sec", 1.0))
        self.allow_search_entry = bool(rospy.get_param("~allow_search_entry", False))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.5))

        self.last_command = None
        self.last_command_time = None
        self.last_input_setpoint = None
        self.last_input_setpoint_time = None
        self.last_mavros_state = None
        self.last_estimator_status = None
        self.last_request_time = None
        self.eligible_setpoint_count = 0
        self.forwarded_setpoint_count = 0
        self.request_count = 0
        self.last_logged_reason = None

        self.output_publisher = rospy.Publisher(self.output_setpoint_topic, PositionTarget, queue_size=30)
        self.command_sub = None
        if self.command_topic:
            self.command_sub = rospy.Subscriber(
                self.command_topic, FollowCommand, self._command_callback, queue_size=20
            )
        self.state_sub = rospy.Subscriber(self.mavros_state_topic, State, self._state_callback, queue_size=20)
        self.estimator_sub = rospy.Subscriber(
            self.estimator_status_topic, EstimatorStatus, self._estimator_callback, queue_size=20
        )
        self.setpoint_sub = rospy.Subscriber(
            self.input_setpoint_topic, PositionTarget, self._setpoint_callback, queue_size=30
        )
        self.status_timer = rospy.Timer(rospy.Duration(0.2), self._status_tick)

        rospy.loginfo(
            "offboard_mode_gate ready input_setpoint=%s output_setpoint=%s command=%s set_mode=%s",
            self.input_setpoint_topic,
            self.output_setpoint_topic,
            self.command_topic,
            self.set_mode_service,
        )

    def _command_callback(self, msg):
        self.last_command = msg
        self.last_command_time = time.monotonic()

    def _state_callback(self, msg):
        self.last_mavros_state = msg

    def _estimator_callback(self, msg):
        self.last_estimator_status = msg

    def _command_is_fresh(self, now_wall):
        if self.last_command is None or self.last_command_time is None:
            return False
        return (now_wall - self.last_command_time) <= self.command_timeout_sec

    def _command_norm(self, cmd):
        return abs(float(cmd.forward_mps)) + abs(float(cmd.lateral_mps)) + abs(float(cmd.yaw_rate_rps))

    def _command_allows_entry(self, now_wall):
        if self.eligibility_source == "setpoint":
            return False
        if not self._command_is_fresh(now_wall):
            return False
        cmd = self.last_command
        if cmd is None or not cmd.valid:
            return False
        if cmd.mode == "body_velocity_yaw_rate":
            return self._command_norm(cmd) >= self.min_entry_command_norm
        if self.allow_search_entry and cmd.mode == "search_yaw_only":
            return abs(float(cmd.yaw_rate_rps)) >= self.min_entry_command_norm
        return False

    def _setpoint_is_fresh(self, now_wall):
        if self.last_input_setpoint is None or self.last_input_setpoint_time is None:
            return False
        return (now_wall - self.last_input_setpoint_time) <= self.command_timeout_sec

    def _setpoint_norm(self, msg):
        norm_value = 0.0
        if not (msg.type_mask & PositionTarget.IGNORE_PX):
            norm_value += abs(float(msg.position.x))
        if not (msg.type_mask & PositionTarget.IGNORE_PY):
            norm_value += abs(float(msg.position.y))
        if not (msg.type_mask & PositionTarget.IGNORE_PZ):
            norm_value += abs(float(msg.position.z))
        if not (msg.type_mask & PositionTarget.IGNORE_VX):
            norm_value += abs(float(msg.velocity.x))
        if not (msg.type_mask & PositionTarget.IGNORE_VY):
            norm_value += abs(float(msg.velocity.y))
        if not (msg.type_mask & PositionTarget.IGNORE_VZ):
            norm_value += abs(float(msg.velocity.z))
        if not (msg.type_mask & PositionTarget.IGNORE_AFX):
            norm_value += abs(float(msg.acceleration_or_force.x))
        if not (msg.type_mask & PositionTarget.IGNORE_AFY):
            norm_value += abs(float(msg.acceleration_or_force.y))
        if not (msg.type_mask & PositionTarget.IGNORE_AFZ):
            norm_value += abs(float(msg.acceleration_or_force.z))
        if not (msg.type_mask & PositionTarget.IGNORE_YAW):
            norm_value += abs(float(msg.yaw))
        if not (msg.type_mask & PositionTarget.IGNORE_YAW_RATE):
            norm_value += abs(float(msg.yaw_rate))
        return norm_value

    def _setpoint_allows_entry(self, now_wall):
        if self.eligibility_source != "setpoint":
            return False
        if not self._setpoint_is_fresh(now_wall):
            return False
        return self._setpoint_norm(self.last_input_setpoint) >= self.min_entry_setpoint_norm

    def _estimator_is_valid(self):
        status = self.last_estimator_status
        if status is None:
            return False
        return (
            status.velocity_horiz_status_flag
            and status.velocity_vert_status_flag
            and status.pos_horiz_rel_status_flag
            and status.pos_vert_abs_status_flag
        )

    def _current_reason(self, now_wall):
        if self.last_mavros_state is None:
            return "waiting_mavros_state"
        if not self.last_mavros_state.connected:
            return "waiting_fcu_connection"
        if self.last_mavros_state.mode == State.MODE_PX4_OFFBOARD:
            return "offboard_active"
        if self.require_armed and not self.last_mavros_state.armed:
            return "waiting_arm"
        if not self._estimator_is_valid():
            return "waiting_estimator"
        if self.eligibility_source == "setpoint":
            if not self._setpoint_is_fresh(now_wall):
                return "waiting_fresh_setpoint"
            if not self._setpoint_allows_entry(now_wall):
                return "waiting_nonzero_setpoint"
        else:
            if not self._command_is_fresh(now_wall):
                return "waiting_fresh_command"
            cmd = self.last_command
            if cmd is None:
                return "waiting_command"
            if not cmd.valid:
                return "waiting_valid_command"
            if cmd.mode != "body_velocity_yaw_rate":
                if cmd.mode == "search_yaw_only" and not self.allow_search_entry:
                    return "search_entry_blocked"
                return "waiting_entry_mode"
            if self._command_norm(cmd) < self.min_entry_command_norm:
                return "waiting_nonzero_command"
        if self.eligible_setpoint_count < self.min_eligible_setpoints:
            return "priming_setpoint_stream"
        return "ready_to_request_offboard"

    def _maybe_request_offboard(self, now_wall):
        reason = self._current_reason(now_wall)
        if reason != "ready_to_request_offboard":
            return
        if self.last_request_time is not None:
            if (now_wall - self.last_request_time) < self.mode_request_retry_sec:
                return

        try:
            rospy.wait_for_service(self.set_mode_service, timeout=0.2)
            set_mode = rospy.ServiceProxy(self.set_mode_service, SetMode)
            response = set_mode(base_mode=0, custom_mode="OFFBOARD")
            self.last_request_time = now_wall
            self.request_count += 1
            rospy.loginfo(
                "offboard_mode_gate request #%d mode_sent=%s eligible_setpoints=%d",
                self.request_count,
                response.mode_sent,
                self.eligible_setpoint_count,
            )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self.last_request_time = now_wall
            rospy.logwarn("offboard_mode_gate set_mode request failed: %s", exc)

    def _setpoint_callback(self, msg):
        if rospy.is_shutdown():
            return
        now_wall = time.monotonic()
        self.last_input_setpoint = msg
        self.last_input_setpoint_time = now_wall
        try:
            self.output_publisher.publish(msg)
        except rospy.ROSException:
            return
        self.forwarded_setpoint_count += 1

        if self._current_reason(now_wall) in ("priming_setpoint_stream", "ready_to_request_offboard"):
            self.eligible_setpoint_count += 1
        elif self.last_mavros_state is None or self.last_mavros_state.mode != State.MODE_PX4_OFFBOARD:
            self.eligible_setpoint_count = 0

        self._maybe_request_offboard(now_wall)

    def _status_tick(self, _event):
        if rospy.is_shutdown():
            return
        now_wall = time.monotonic()
        reason = self._current_reason(now_wall)
        if reason != self.last_logged_reason:
            state = self.last_mavros_state.mode if self.last_mavros_state is not None else "<none>"
            armed = self.last_mavros_state.armed if self.last_mavros_state is not None else False
            connected = self.last_mavros_state.connected if self.last_mavros_state is not None else False
            cmd_mode = self.last_command.mode if self.last_command is not None else "<none>"
            cmd_valid = self.last_command.valid if self.last_command is not None else False
            setpoint_norm = self._setpoint_norm(self.last_input_setpoint) if self.last_input_setpoint is not None else 0.0
            rospy.loginfo(
                "offboard_mode_gate state=%s source=%s mode=%s armed=%s connected=%s estimator_valid=%s cmd_mode=%s cmd_valid=%s setpoint_norm=%.3f eligible_setpoints=%d forwarded=%d requests=%d",
                reason,
                self.eligibility_source,
                state,
                armed,
                connected,
                self._estimator_is_valid(),
                cmd_mode,
                cmd_valid,
                setpoint_norm,
                self.eligible_setpoint_count,
                self.forwarded_setpoint_count,
                self.request_count,
            )
            self.last_logged_reason = reason


if __name__ == "__main__":
    rospy.init_node("human_follow_offboard_gate")
    OffboardModeGateNode()
    rospy.spin()
