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

import rospy
from mavros_msgs.msg import PositionTarget


class EgoBridgeRegressionMonitorNode:
    def __init__(self):
        self.command_topic = rospy.get_param("~command_topic", "/follow/stage2/ego_position_cmd")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "/follow/stage2/offboard/setpoint")
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 8.0))
        self.max_command_age_sec = float(rospy.get_param("~max_command_age_sec", 0.4))
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 1e-5))
        self.velocity_tolerance = float(rospy.get_param("~velocity_tolerance", 1e-5))
        self.accel_tolerance = float(rospy.get_param("~accel_tolerance", 1e-5))
        self.yaw_tolerance = float(rospy.get_param("~yaw_tolerance", 1e-5))

        self.position_command_class = self._resolve_position_command_type()
        self.start_time = time.monotonic()
        self.recent_commands = deque()
        self.seen_nonzero_position = False
        self.seen_nonzero_velocity = False
        self.seen_hold = False
        self.seen_first_match = False
        self.finished = False
        self.exit_code = 0

        self.command_sub = rospy.Subscriber(
            self.command_topic, self.position_command_class, self._command_callback, queue_size=20
        )
        self.setpoint_sub = rospy.Subscriber(self.setpoint_topic, PositionTarget, self._setpoint_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

        rospy.loginfo("ego_bridge_regression_monitor ready setpoint=%s", self.setpoint_topic)

    def _resolve_position_command_type(self):
        try:
            from quadrotor_msgs.msg import PositionCommand
            return PositionCommand
        except ImportError as exc:
            raise RuntimeError("quadrotor_msgs/PositionCommand unavailable") from exc

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
        rospy.logerr("ego bridge regression FAIL %s", reason)
        rospy.signal_shutdown(reason)

    def _pass(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0
        rospy.loginfo("ego bridge regression PASS %s", reason)
        rospy.signal_shutdown(reason)

    def _setpoint_is_effectively_zero(self, setpoint_msg):
        return (
            self._close(setpoint_msg.position.x, 0.0, self.position_tolerance)
            and self._close(setpoint_msg.position.y, 0.0, self.position_tolerance)
            and self._close(setpoint_msg.position.z, 0.0, self.position_tolerance)
            and self._close(setpoint_msg.velocity.x, 0.0, self.velocity_tolerance)
            and self._close(setpoint_msg.velocity.y, 0.0, self.velocity_tolerance)
            and self._close(setpoint_msg.velocity.z, 0.0, self.velocity_tolerance)
            and self._close(setpoint_msg.acceleration_or_force.x, 0.0, self.accel_tolerance)
            and self._close(setpoint_msg.acceleration_or_force.y, 0.0, self.accel_tolerance)
            and self._close(setpoint_msg.acceleration_or_force.z, 0.0, self.accel_tolerance)
            and self._close(setpoint_msg.yaw, 0.0, self.yaw_tolerance)
            and self._close(setpoint_msg.yaw_rate, 0.0, self.yaw_tolerance)
        )

    def _match_command(self, setpoint_msg, cmd):
        if (
            self._close(setpoint_msg.position.x, cmd.position.x, self.position_tolerance)
            and self._close(setpoint_msg.position.y, cmd.position.y, self.position_tolerance)
            and self._close(setpoint_msg.position.z, cmd.position.z, self.position_tolerance)
            and self._close(setpoint_msg.velocity.x, cmd.velocity.x, self.velocity_tolerance)
            and self._close(setpoint_msg.velocity.y, cmd.velocity.y, self.velocity_tolerance)
            and self._close(setpoint_msg.velocity.z, cmd.velocity.z, self.velocity_tolerance)
            and self._close(setpoint_msg.acceleration_or_force.x, 0.0, self.accel_tolerance)
            and self._close(setpoint_msg.acceleration_or_force.y, 0.0, self.accel_tolerance)
            and self._close(setpoint_msg.acceleration_or_force.z, 0.0, self.accel_tolerance)
            and self._close(setpoint_msg.yaw, cmd.yaw, self.yaw_tolerance)
            and self._close(setpoint_msg.yaw_rate, cmd.yaw_dot, self.yaw_tolerance)
        ):
            if abs(float(cmd.position.x)) + abs(float(cmd.position.y)) + abs(float(cmd.position.z)) > 1e-4:
                self.seen_nonzero_position = True
            if abs(float(cmd.velocity.x)) + abs(float(cmd.velocity.y)) + abs(float(cmd.velocity.z)) > 1e-4:
                self.seen_nonzero_velocity = True
            if (
                abs(float(cmd.velocity.x)) + abs(float(cmd.velocity.y)) + abs(float(cmd.velocity.z)) <= 1e-4
                and abs(float(cmd.yaw_dot)) <= 1e-4
            ):
                self.seen_hold = True
            return True
        return False

    def _setpoint_callback(self, msg):
        if self.finished:
            return

        now = time.monotonic()
        self._prune_old_commands(now)
        if not self.recent_commands:
            return

        if msg.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            self._fail("coordinate_frame mismatch")
            return

        for _stamp, cmd in reversed(self.recent_commands):
            if self._match_command(msg, cmd):
                self.seen_first_match = True
                break
        else:
            if not self.seen_first_match and self._setpoint_is_effectively_zero(msg):
                return
            self._fail("no recent PositionCommand matched PositionTarget")
            return

        if self.seen_nonzero_position and self.seen_nonzero_velocity and self.seen_hold:
            self._pass("position, velocity, and hold phases matched expected bridge output")

    def _tick(self, _event):
        if self.finished:
            return
        if (time.monotonic() - self.start_time) >= self.max_duration_sec:
            self._fail("timeout before observing all ego bridge phases")


if __name__ == "__main__":
    rospy.init_node("ego_bridge_regression_monitor")
    node = EgoBridgeRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
