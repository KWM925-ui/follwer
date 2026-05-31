#!/usr/bin/env python3
import importlib.util
import math
import os
import sys

import rospy
from std_msgs.msg import String

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_BASE_MONITOR_PATH = os.path.join(_SCRIPT_DIR, "stage2_paper_line_regression_monitor_node.py")
_BASE_SPEC = importlib.util.spec_from_file_location("stage2_paper_line_regression_monitor_impl", _BASE_MONITOR_PATH)
_BASE_MODULE = importlib.util.module_from_spec(_BASE_SPEC)
_BASE_SPEC.loader.exec_module(_BASE_MODULE)
Stage2PaperLineRegressionMonitorNode = _BASE_MODULE.Stage2PaperLineRegressionMonitorNode


class Stage2PaperLineSearchRegressionMonitorNode(Stage2PaperLineRegressionMonitorNode):
    def __init__(self):
        self.phase_label_topic = rospy.get_param("~phase_label_topic", "/follow/test/phase_label")
        self.required_search_phase = rospy.get_param("~required_search_phase", "search_loss_right")
        self.min_search_goal_shift_m = float(rospy.get_param("~min_search_goal_shift_m", 0.25))
        self.min_search_cmd_shift_m = float(rospy.get_param("~min_search_cmd_shift_m", 0.15))

        self.current_phase = ""
        self.search_first_goal = None
        self.search_last_goal = None
        self.search_goal_count = 0
        self.search_first_cmd = None
        self.search_last_cmd = None
        self.search_cmd_count = 0

        super().__init__()
        rospy.Subscriber(self.phase_label_topic, String, self._phase_callback, queue_size=20)
        rospy.loginfo(
            "stage2_paper_line_search_regression_monitor ready phase=%s required_phase=%s",
            self.phase_label_topic,
            self.required_search_phase,
        )

    def _phase_callback(self, msg):
        phase = str(msg.data).strip()
        if phase == self.current_phase:
            return
        if phase == self.required_search_phase:
            self.search_first_goal = None
            self.search_last_goal = None
            self.search_goal_count = 0
            self.search_first_cmd = None
            self.search_last_cmd = None
            self.search_cmd_count = 0
        self.current_phase = phase

    def _pose_xyz(self, msg):
        point = msg.pose.position
        return float(point.x), float(point.y), float(point.z)

    def _distance(self, left, right):
        return math.sqrt(
            (left[0] - right[0]) ** 2
            + (left[1] - right[1]) ** 2
            + (left[2] - right[2]) ** 2
        )

    def _goal_callback(self, msg):
        super()._goal_callback(msg)
        if self.current_phase != self.required_search_phase:
            return
        xyz = self._pose_xyz(msg)
        if self.search_first_goal is None:
            self.search_first_goal = xyz
        self.search_last_goal = xyz
        self.search_goal_count += 1

    def _ego_cmd_callback(self, msg):
        super()._ego_cmd_callback(msg)
        if self.current_phase != self.required_search_phase:
            return
        xyz = (float(msg.position.x), float(msg.position.y), float(msg.position.z))
        if self.search_first_cmd is None:
            self.search_first_cmd = xyz
        self.search_last_cmd = xyz
        self.search_cmd_count += 1

    def _check_search_contracts(self):
        if self.current_phase != self.required_search_phase:
            return False, "waiting_search_phase"
        if self.search_first_goal is None or self.search_last_goal is None:
            return False, "waiting_search_goal_samples"
        if self.search_first_cmd is None or self.search_last_cmd is None:
            return False, "waiting_search_cmd_samples"

        goal_shift = self._distance(self.search_first_goal, self.search_last_goal)
        if self.search_goal_count < 2 or goal_shift < self.min_search_goal_shift_m:
            return False, "search_goal_shift_too_small"

        cmd_shift = self._distance(self.search_first_cmd, self.search_last_cmd)
        if self.search_cmd_count < 2 or cmd_shift < self.min_search_cmd_shift_m:
            return False, "search_cmd_shift_too_small"

        return True, "search_goal_count=%d search_cmd_count=%d goal_shift=%.3f cmd_shift=%.3f" % (
            self.search_goal_count,
            self.search_cmd_count,
            goal_shift,
            cmd_shift,
        )

    def _check_contracts(self):
        ok, reason = super()._check_contracts()
        if not ok:
            return ok, reason
        return self._check_search_contracts()


if __name__ == "__main__":
    rospy.init_node("stage2_paper_line_search_regression_monitor")
    node = Stage2PaperLineSearchRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
