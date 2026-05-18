#!/usr/bin/env python3
import glob
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
from std_msgs.msg import String, UInt32


class OffboardGateRegressionMonitorNode:
    def __init__(self):
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.eligibility_source = rospy.get_param("~eligibility_source", "command").strip().lower()
        self.forwarded_setpoint_topic = rospy.get_param("~forwarded_setpoint_topic", "/mavros/setpoint_raw/local")
        self.mavros_state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")
        self.estimator_topic = rospy.get_param("~estimator_topic", "/mavros/estimator_status")
        self.request_count_topic = rospy.get_param("~request_count_topic", "/mavros/fake/set_mode_request_count")
        self.last_request_topic = rospy.get_param("~last_request_topic", "/mavros/fake/last_mode_request")
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 8.0))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.5))
        self.min_eligible_setpoints = int(rospy.get_param("~min_eligible_setpoints", 10))
        self.min_entry_command_norm = float(rospy.get_param("~min_entry_command_norm", 0.05))
        self.min_entry_setpoint_norm = float(rospy.get_param("~min_entry_setpoint_norm", 0.05))
        self.exit_on_pass = bool(rospy.get_param("~exit_on_pass", True))

        self.start_time = time.monotonic()
        self.last_command = None
        self.last_command_time = None
        self.last_input_setpoint = None
        self.last_input_setpoint_time = None
        self.last_mavros_state = None
        self.last_estimator = None
        self.request_count = 0
        self.last_request = ""
        self.eligible_setpoint_count = 0
        self.output_setpoint_count = 0
        self.offboard_seen = False
        self.finished = False
        self.exit_code = 0

        self.command_sub = None
        if self.command_topic:
            self.command_sub = rospy.Subscriber(
                self.command_topic, FollowCommand, self._command_callback, queue_size=20
            )
        self.setpoint_sub = rospy.Subscriber(
            self.forwarded_setpoint_topic, PositionTarget, self._setpoint_callback, queue_size=20
        )
        self.state_sub = rospy.Subscriber(self.mavros_state_topic, State, self._state_callback, queue_size=20)
        self.estimator_sub = rospy.Subscriber(
            self.estimator_topic, EstimatorStatus, self._estimator_callback, queue_size=20
        )
        self.request_count_sub = rospy.Subscriber(
            self.request_count_topic, UInt32, self._request_count_callback, queue_size=20
        )
        self.last_request_sub = rospy.Subscriber(
            self.last_request_topic, String, self._last_request_callback, queue_size=20
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

        rospy.loginfo("offboard_gate_regression_monitor ready setpoint=%s", self.forwarded_setpoint_topic)

    def _command_callback(self, msg):
        self.last_command = msg
        self.last_command_time = time.monotonic()

    def _state_callback(self, msg):
        self.last_mavros_state = msg
        if msg.mode == State.MODE_PX4_OFFBOARD:
            self.offboard_seen = True

    def _estimator_callback(self, msg):
        self.last_estimator = msg

    def _request_count_callback(self, msg):
        self.request_count = int(msg.data)

    def _last_request_callback(self, msg):
        self.last_request = msg.data

    def _fail(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 1
        rospy.logerr("offboard_gate regression FAIL %s", reason)
        rospy.signal_shutdown(reason)

    def _pass(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0
        rospy.loginfo("offboard_gate regression PASS %s", reason)
        if self.exit_on_pass:
            rospy.signal_shutdown(reason)

    def _command_is_fresh(self, now_wall):
        if self.last_command is None or self.last_command_time is None:
            return False
        return (now_wall - self.last_command_time) <= self.command_timeout_sec

    def _command_active(self, now_wall):
        if self.eligibility_source == "setpoint":
            return False
        if not self._command_is_fresh(now_wall):
            return False
        cmd = self.last_command
        if cmd is None or not cmd.valid or cmd.mode != "body_velocity_yaw_rate":
            return False
        return abs(cmd.forward_mps) + abs(cmd.lateral_mps) + abs(cmd.yaw_rate_rps) >= self.min_entry_command_norm

    def _setpoint_is_fresh(self, now_wall):
        if self.last_input_setpoint is None or self.last_input_setpoint_time is None:
            return False
        return (now_wall - self.last_input_setpoint_time) <= self.command_timeout_sec

    def _setpoint_norm(self):
        if self.last_input_setpoint is None:
            return 0.0
        msg = self.last_input_setpoint
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

    def _estimator_valid(self):
        status = self.last_estimator
        if status is None:
            return False
        return (
            status.velocity_horiz_status_flag
            and status.velocity_vert_status_flag
            and status.pos_horiz_rel_status_flag
            and status.pos_vert_abs_status_flag
        )

    def _ready_for_entry(self, now_wall):
        if self.last_mavros_state is None:
            return False
        if self.eligibility_source == "setpoint":
            entry_ready = self._setpoint_is_fresh(now_wall) and self._setpoint_norm() >= self.min_entry_setpoint_norm
        else:
            entry_ready = self._command_active(now_wall)
        return (
            self.last_mavros_state.connected
            and self.last_mavros_state.armed
            and self._estimator_valid()
            and entry_ready
        )

    def _setpoint_callback(self, msg):
        now_wall = time.monotonic()
        self.last_input_setpoint = msg
        self.last_input_setpoint_time = now_wall
        self.output_setpoint_count += 1
        if self._ready_for_entry(now_wall) and not self.offboard_seen:
            self.eligible_setpoint_count += 1
        elif not self.offboard_seen:
            self.eligible_setpoint_count = 0

    def _tick(self, _event):
        if self.finished:
            return
        now_wall = time.monotonic()

        if self.request_count > 0:
            if self.last_request != "OFFBOARD":
                self._fail("unexpected mode request=%s" % self.last_request)
                return
            if not self._ready_for_entry(now_wall):
                self._fail("offboard requested before entry criteria were satisfied")
                return
            if self.eligible_setpoint_count < self.min_eligible_setpoints:
                self._fail(
                    "offboard requested before min eligible setpoints count=%d need=%d"
                    % (self.eligible_setpoint_count, self.min_eligible_setpoints)
                )
                return

        if self.offboard_seen:
            if self.request_count != 1:
                self._fail("expected exactly one offboard request count=%d" % self.request_count)
                return
            self._pass(
                "offboard entered after eligible setpoints=%d output_setpoints=%d"
                % (self.eligible_setpoint_count, self.output_setpoint_count)
            )
            return

        if (now_wall - self.start_time) >= self.max_duration_sec:
            self._fail(
                "timeout request_count=%d eligible_setpoints=%d output_setpoints=%d"
                % (self.request_count, self.eligible_setpoint_count, self.output_setpoint_count)
            )


if __name__ == "__main__":
    rospy.init_node("offboard_gate_regression_monitor")
    node = OffboardGateRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
