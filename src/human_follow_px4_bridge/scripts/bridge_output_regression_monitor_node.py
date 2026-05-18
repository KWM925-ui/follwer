#!/usr/bin/env python3
import glob
import os
import sys

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

import math
import sys
import time
from collections import deque

import rospy

from human_follow_msgs.msg import FollowCommand
from mavros_msgs.msg import PositionTarget


class BridgeOutputRegressionMonitorNode:
    def __init__(self):
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "/follow/offboard/setpoint")
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 8.0))
        self.max_command_age_sec = float(rospy.get_param("~max_command_age_sec", 0.4))
        self.output_frame_id = rospy.get_param("~output_frame_id", "base_link")
        self.flip_lateral_sign = bool(rospy.get_param("~flip_lateral_sign", True))
        self.flip_yaw_rate_sign = bool(rospy.get_param("~flip_yaw_rate_sign", True))
        self.forward_scale = float(rospy.get_param("~forward_scale", 1.0))
        self.lateral_scale = float(rospy.get_param("~lateral_scale", 1.0))
        self.yaw_rate_scale = float(rospy.get_param("~yaw_rate_scale", 1.0))
        self.velocity_tolerance = float(rospy.get_param("~velocity_tolerance", 1e-5))
        self.yaw_rate_tolerance = float(rospy.get_param("~yaw_rate_tolerance", 1e-5))

        self.start_time = time.monotonic()
        self.recent_commands = deque()
        self.seen_track_right = False
        self.seen_track_left = False
        self.seen_search = False
        self.seen_hold_invalid = False
        self.finished = False
        self.exit_code = 0

        self.command_sub = rospy.Subscriber(self.command_topic, FollowCommand, self._command_callback, queue_size=20)
        self.setpoint_sub = rospy.Subscriber(
            self.setpoint_topic, PositionTarget, self._setpoint_callback, queue_size=20
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

        self.expected_type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )

        rospy.loginfo("bridge_output_regression_monitor ready: setpoint=%s", self.setpoint_topic)

    def _command_callback(self, msg):
        now = time.monotonic()
        self.recent_commands.append((now, msg))
        self._prune_old_commands(now)

    def _prune_old_commands(self, now):
        while self.recent_commands and (now - self.recent_commands[0][0]) > self.max_command_age_sec:
            self.recent_commands.popleft()

    def _close(self, a, b, tol):
        return math.isclose(float(a), float(b), abs_tol=tol)

    def _fail(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 1
        rospy.logerr("bridge regression FAIL %s", reason)
        rospy.signal_shutdown(reason)

    def _pass(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0
        rospy.loginfo("bridge regression PASS %s", reason)
        rospy.signal_shutdown(reason)

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
                    self.seen_track_right = True
                if cmd.lateral_mps < 0.0:
                    self.seen_track_left = True
                return True
            return False

        if cmd.valid and cmd.mode == "search_yaw_only":
            expected_yaw_rate = self._mapped_yaw_rate(cmd.yaw_rate_rps)
            if (
                self._close(setpoint_msg.velocity.x, 0.0, self.velocity_tolerance)
                and self._close(setpoint_msg.velocity.y, 0.0, self.velocity_tolerance)
                and self._close(setpoint_msg.yaw_rate, expected_yaw_rate, self.yaw_rate_tolerance)
            ):
                self.seen_search = True
                return True
            return False

        if (
            self._close(setpoint_msg.velocity.x, 0.0, self.velocity_tolerance)
            and self._close(setpoint_msg.velocity.y, 0.0, self.velocity_tolerance)
            and self._close(setpoint_msg.yaw_rate, 0.0, self.yaw_rate_tolerance)
        ):
            self.seen_hold_invalid = True
            return True
        return False

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
                break
        else:
            self._fail(
                "setpoint did not match any recent command vx=%.3f vy=%.3f yaw_rate=%.3f recent=%d"
                % (msg.velocity.x, msg.velocity.y, msg.yaw_rate, len(self.recent_commands))
            )
            return

        if self.seen_track_right and self.seen_track_left and self.seen_search and self.seen_hold_invalid:
            self._pass("all bridge phases matched expected PositionTarget mapping")

    def _tick(self, _event):
        if self.finished:
            return
        if (time.monotonic() - self.start_time) >= self.max_duration_sec:
            self._fail("timeout before observing all bridge phases")


if __name__ == "__main__":
    rospy.init_node("bridge_output_regression_monitor")
    node = BridgeOutputRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
