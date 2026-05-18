#!/usr/bin/env python3
import sys
import time

import numpy as np
import rospy

from human_follow_msgs.msg import Target3D


class FusionOutputRegressionMonitorNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/fusion/target_body")
        self.expected_center_xyz = np.array(rospy.get_param("~expected_center_xyz", [0.4, 0.0, 6.0]), dtype=np.float64)
        self.expected_frame_id = rospy.get_param("~expected_frame_id", "camera")
        self.max_center_error_m = float(rospy.get_param("~max_center_error_m", 0.15))
        self.min_support_points = int(rospy.get_param("~min_support_points", 6))
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 5.0))
        self.exit_code = 0
        self.start_time = time.monotonic()
        self.finished = False

        self.subscriber = rospy.Subscriber(self.input_topic, Target3D, self._callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

        rospy.loginfo("fusion_output_regression_monitor ready: input=%s", self.input_topic)

    def _finish(self, success, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0 if success else 1
        if success:
            rospy.loginfo("fusion output regression PASS %s", reason)
        else:
            rospy.logerr("fusion output regression FAIL %s", reason)
        rospy.signal_shutdown(reason)

    def _callback(self, msg):
        if self.finished or not msg.valid:
            return

        estimated = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=np.float64)
        center_error = float(np.linalg.norm(estimated - self.expected_center_xyz))
        if msg.frame_id != self.expected_frame_id:
            self._finish(False, "frame mismatch: %s != %s" % (msg.frame_id, self.expected_frame_id))
            return
        if msg.support_points < self.min_support_points:
            self._finish(False, "support points too low: %d" % msg.support_points)
            return
        if center_error > self.max_center_error_m:
            self._finish(False, "center error too large: %.3f" % center_error)
            return

        self._finish(
            True,
            "frame=%s support=%d center_error_m=%.3f" % (msg.frame_id, msg.support_points, center_error),
        )

    def _tick(self, _event):
        if self.finished:
            return
        if (time.monotonic() - self.start_time) >= self.max_duration_sec:
            self._finish(False, "timeout waiting for valid fusion output")


if __name__ == "__main__":
    rospy.init_node("fusion_output_regression_monitor")
    node = FusionOutputRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
