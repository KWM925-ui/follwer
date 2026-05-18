#!/usr/bin/env python3
import glob
import math
import os
import sys
import time

import rospy

from human_follow_msgs.msg import FollowCommand, FollowState, Target2D, Target3D, TrackerStatus


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

from mavros_msgs.msg import PositionTarget  # noqa: E402


class LiveIdleSafetyMonitorNode:
    def __init__(self):
        self.detector_topic = rospy.get_param("~detector_topic", "/follow/detector/person_target")
        self.tracker_status_topic = rospy.get_param("~tracker_status_topic", "/follow/tracker/status")
        self.target_body_topic = rospy.get_param("~target_body_topic", "/follow/fusion/target_body")
        self.follow_state_topic = rospy.get_param("~follow_state_topic", "/follow/state")
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "/follow/offboard/setpoint")
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 8.0))
        self.startup_grace_sec = float(rospy.get_param("~startup_grace_sec", 0.8))
        self.min_detector_msgs = int(rospy.get_param("~min_detector_msgs", 3))
        self.min_tracker_msgs = int(rospy.get_param("~min_tracker_msgs", 3))
        self.min_follow_state_msgs = int(rospy.get_param("~min_follow_state_msgs", 3))
        self.min_command_msgs = int(rospy.get_param("~min_command_msgs", 3))
        self.min_setpoint_msgs = int(rospy.get_param("~min_setpoint_msgs", 5))
        self.output_frame_id = rospy.get_param("~output_frame_id", "base_link")
        self.value_tolerance = float(rospy.get_param("~value_tolerance", 1e-5))

        self.start_time = time.monotonic()
        self.detector_count = 0
        self.tracker_count = 0
        self.target_body_count = 0
        self.follow_state_count = 0
        self.command_count = 0
        self.setpoint_count = 0
        self.finished = False
        self.exit_code = 0

        self.detector_sub = rospy.Subscriber(self.detector_topic, Target2D, self._detector_callback, queue_size=10)
        self.tracker_sub = rospy.Subscriber(
            self.tracker_status_topic, TrackerStatus, self._tracker_status_callback, queue_size=10
        )
        self.target_body_sub = rospy.Subscriber(self.target_body_topic, Target3D, self._target_body_callback, queue_size=10)
        self.follow_state_sub = rospy.Subscriber(
            self.follow_state_topic, FollowState, self._follow_state_callback, queue_size=10
        )
        self.command_sub = rospy.Subscriber(self.command_topic, FollowCommand, self._command_callback, queue_size=10)
        self.setpoint_sub = rospy.Subscriber(self.setpoint_topic, PositionTarget, self._setpoint_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._tick)

        rospy.loginfo(
            "live_idle_safety_monitor ready: detector=%s tracker=%s state=%s setpoint=%s",
            self.detector_topic,
            self.tracker_status_topic,
            self.follow_state_topic,
            self.setpoint_topic,
        )

    def _fail(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 1
        rospy.logerr("live idle safety FAIL reason=%s", reason)
        rospy.signal_shutdown(reason)

    def _pass(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0
        rospy.loginfo(
            "live idle safety PASS detector=%d tracker=%d target_body=%d state=%d command=%d setpoint=%d",
            self.detector_count,
            self.tracker_count,
            self.target_body_count,
            self.follow_state_count,
            self.command_count,
            self.setpoint_count,
        )
        rospy.signal_shutdown(reason)

    def _close_zero(self, value):
        return math.isclose(float(value), 0.0, abs_tol=self.value_tolerance)

    def _detector_callback(self, msg):
        self.detector_count += 1
        if msg.valid:
            self._fail("detector emitted valid target without camera input")

    def _tracker_status_callback(self, msg):
        self.tracker_count += 1
        if msg.output_valid:
            self._fail("tracker output_valid true without camera input")
        if msg.tracking_state != "timeout":
            self._fail("tracker state not timeout: %s" % msg.tracking_state)

    def _target_body_callback(self, msg):
        self.target_body_count += 1
        if msg.valid:
            self._fail("target_body valid without camera input")

    def _follow_state_callback(self, msg):
        self.follow_state_count += 1
        if msg.state_name != "idle":
            self._fail("follow state not idle: %s" % msg.state_name)

    def _command_callback(self, msg):
        self.command_count += 1
        if msg.valid:
            self._fail("command valid without camera input")
        if msg.mode != "hold":
            self._fail("command mode not hold: %s" % msg.mode)
        if not self._close_zero(msg.forward_mps) or not self._close_zero(msg.lateral_mps) or not self._close_zero(msg.yaw_rate_rps):
            self._fail("command not zero")

    def _setpoint_callback(self, msg):
        self.setpoint_count += 1
        if msg.header.frame_id != self.output_frame_id:
            self._fail("setpoint frame_id mismatch: %s" % msg.header.frame_id)
        if not self._close_zero(msg.velocity.x):
            self._fail("setpoint velocity.x not zero")
        if not self._close_zero(msg.velocity.y):
            self._fail("setpoint velocity.y not zero")
        if not self._close_zero(msg.velocity.z):
            self._fail("setpoint velocity.z not zero")
        if not self._close_zero(msg.yaw_rate):
            self._fail("setpoint yaw_rate not zero")

    def _tick(self, _event):
        if self.finished:
            return

        elapsed_sec = time.monotonic() - self.start_time
        if elapsed_sec < self.startup_grace_sec:
            return

        if (
            self.detector_count >= self.min_detector_msgs
            and self.tracker_count >= self.min_tracker_msgs
            and self.follow_state_count >= self.min_follow_state_msgs
            and self.command_count >= self.min_command_msgs
            and self.setpoint_count >= self.min_setpoint_msgs
        ):
            self._pass("all live-idle safety requirements satisfied")
            return

        if elapsed_sec >= self.max_duration_sec:
            self._fail(
                "insufficient_msgs detector=%d tracker=%d state=%d command=%d setpoint=%d"
                % (
                    self.detector_count,
                    self.tracker_count,
                    self.follow_state_count,
                    self.command_count,
                    self.setpoint_count,
                )
            )


def main():
    rospy.init_node("live_idle_safety_monitor")
    node = LiveIdleSafetyMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
