#!/usr/bin/env python3
import glob
import os
import sys
import time

import rospy
from sensor_msgs.msg import Image

from human_follow_msgs.msg import FollowCommand, Target2D, Target3D, TrackerStatus


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


class OfflineVideoSmokeMonitorNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/follow/camera/image_raw")
        self.detector_topic = rospy.get_param("~detector_topic", "/follow/detector/person_target")
        self.tracker_status_topic = rospy.get_param("~tracker_status_topic", "/follow/tracker/status")
        self.target_body_topic = rospy.get_param("~target_body_topic", "/follow/fusion/target_body")
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "/follow/offboard/setpoint")
        self.overlay_topic = rospy.get_param("~overlay_topic", "/follow/debug/overlay_image")
        self.startup_grace_sec = float(rospy.get_param("~startup_grace_sec", 0.5))
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 18.0))
        self.min_image_count = int(rospy.get_param("~min_image_count", 8))
        self.min_overlay_count = int(rospy.get_param("~min_overlay_count", 3))
        self.min_detector_valid_count = int(rospy.get_param("~min_detector_valid_count", 1))
        self.min_tracker_valid_count = int(rospy.get_param("~min_tracker_valid_count", 1))
        self.min_target_body_valid_count = int(rospy.get_param("~min_target_body_valid_count", 1))
        self.min_command_valid_count = int(rospy.get_param("~min_command_valid_count", 1))
        self.min_setpoint_count = int(rospy.get_param("~min_setpoint_count", 3))

        self.start_time = time.monotonic()
        self.image_count = 0
        self.overlay_count = 0
        self.detector_msg_count = 0
        self.detector_valid_count = 0
        self.tracker_status_count = 0
        self.tracker_valid_count = 0
        self.target_body_count = 0
        self.target_body_valid_count = 0
        self.command_count = 0
        self.command_valid_count = 0
        self.setpoint_count = 0
        self.finished = False
        self.exit_code = 0

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=10)
        self.detector_sub = rospy.Subscriber(self.detector_topic, Target2D, self._detector_callback, queue_size=10)
        self.tracker_status_sub = rospy.Subscriber(
            self.tracker_status_topic, TrackerStatus, self._tracker_status_callback, queue_size=10
        )
        self.target_body_sub = rospy.Subscriber(self.target_body_topic, Target3D, self._target_body_callback, queue_size=10)
        self.command_sub = rospy.Subscriber(self.command_topic, FollowCommand, self._command_callback, queue_size=10)
        self.setpoint_sub = rospy.Subscriber(self.setpoint_topic, PositionTarget, self._setpoint_callback, queue_size=10)
        self.overlay_sub = rospy.Subscriber(self.overlay_topic, Image, self._overlay_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._tick)

        rospy.loginfo(
            "offline_video_smoke_monitor ready: image=%s detector=%s tracker=%s target_body=%s overlay=%s setpoint=%s",
            self.image_topic,
            self.detector_topic,
            self.tracker_status_topic,
            self.target_body_topic,
            self.overlay_topic,
            self.setpoint_topic,
        )

    def _image_callback(self, _msg):
        self.image_count += 1

    def _detector_callback(self, msg):
        self.detector_msg_count += 1
        if msg.valid:
            self.detector_valid_count += 1

    def _tracker_status_callback(self, msg):
        self.tracker_status_count += 1
        if msg.output_valid:
            self.tracker_valid_count += 1

    def _target_body_callback(self, msg):
        self.target_body_count += 1
        if msg.valid:
            self.target_body_valid_count += 1

    def _command_callback(self, msg):
        self.command_count += 1
        if msg.valid:
            self.command_valid_count += 1

    def _setpoint_callback(self, _msg):
        self.setpoint_count += 1

    def _overlay_callback(self, _msg):
        self.overlay_count += 1

    def _missing(self):
        missing = []
        if self.image_count < self.min_image_count:
            missing.append("image_count<%d" % self.min_image_count)
        if self.overlay_count < self.min_overlay_count:
            missing.append("overlay_count<%d" % self.min_overlay_count)
        if self.detector_msg_count == 0:
            missing.append("detector_msg_count=0")
        if self.detector_valid_count < self.min_detector_valid_count:
            missing.append("detector_valid_count<%d" % self.min_detector_valid_count)
        if self.tracker_status_count == 0:
            missing.append("tracker_status_count=0")
        if self.tracker_valid_count < self.min_tracker_valid_count:
            missing.append("tracker_valid_count<%d" % self.min_tracker_valid_count)
        if self.target_body_count == 0:
            missing.append("target_body_count=0")
        if self.target_body_valid_count < self.min_target_body_valid_count:
            missing.append("target_body_valid_count<%d" % self.min_target_body_valid_count)
        if self.command_count == 0:
            missing.append("command_count=0")
        if self.command_valid_count < self.min_command_valid_count:
            missing.append("command_valid_count<%d" % self.min_command_valid_count)
        if self.setpoint_count < self.min_setpoint_count:
            missing.append("setpoint_count<%d" % self.min_setpoint_count)
        return missing

    def _finish(self, ok, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0 if ok else 1
        if ok:
            rospy.loginfo(
                "offline video smoke PASS images=%d detector_valid=%d tracker_valid=%d target_body_valid=%d command_valid=%d setpoints=%d overlays=%d",
                self.image_count,
                self.detector_valid_count,
                self.tracker_valid_count,
                self.target_body_valid_count,
                self.command_valid_count,
                self.setpoint_count,
                self.overlay_count,
            )
        else:
            rospy.logerr("offline video smoke FAIL reason=%s", reason)
        rospy.signal_shutdown(reason)

    def _tick(self, _event):
        if self.finished:
            return

        elapsed_sec = time.monotonic() - self.start_time
        if elapsed_sec < self.startup_grace_sec:
            return

        missing = self._missing()
        if not missing:
            self._finish(True, "all smoke requirements satisfied")
            return

        if elapsed_sec >= self.max_duration_sec:
            self._finish(False, "missing=" + ",".join(missing))


def main():
    rospy.init_node("offline_video_smoke_monitor")
    node = OfflineVideoSmokeMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
