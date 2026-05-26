#!/usr/bin/env python3
import importlib.util
import os
import sys

import rospy
from nav_msgs.msg import Path

from ego_planner.msg import Bspline
from human_follow_msgs.msg import FollowState

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_BASE_MONITOR_PATH = os.path.join(_SCRIPT_DIR, "stage2_real_ego_full_chain_regression_monitor_node.py")
_BASE_SPEC = importlib.util.spec_from_file_location("stage2_real_ego_full_chain_regression_monitor_impl", _BASE_MONITOR_PATH)
_BASE_MODULE = importlib.util.module_from_spec(_BASE_SPEC)
_BASE_SPEC.loader.exec_module(_BASE_MODULE)
Stage2RealEgoFullChainRegressionMonitorNode = _BASE_MODULE.Stage2RealEgoFullChainRegressionMonitorNode


class Stage2PaperLineRegressionMonitorNode(Stage2RealEgoFullChainRegressionMonitorNode):
    def __init__(self):
        self.bspline_topic = rospy.get_param("~bspline_topic", "/planning/bspline")
        self.state_topic = rospy.get_param("~state_topic", "/follow/stage2/state")
        self.debug_goal_path_topic = rospy.get_param("~debug_goal_path_topic", "/follow/stage2/debug_goal_path")
        self.require_paper_line_state_detail = bool(rospy.get_param("~require_paper_line_state_detail", True))

        self.last_bspline = None
        self.last_state = None
        self.last_debug_goal_path = None

        super().__init__()
        rospy.Subscriber(self.bspline_topic, Bspline, self._bspline_callback, queue_size=20)
        rospy.Subscriber(self.state_topic, FollowState, self._state_callback, queue_size=20)
        rospy.Subscriber(self.debug_goal_path_topic, Path, self._debug_goal_path_callback, queue_size=20)
        rospy.loginfo(
            "stage2_paper_line_regression_monitor ready bspline=%s state=%s debug_path=%s",
            self.bspline_topic,
            self.state_topic,
            self.debug_goal_path_topic,
        )

    def _bspline_callback(self, msg):
        self.last_bspline = msg

    def _state_callback(self, msg):
        self.last_state = msg

    def _debug_goal_path_callback(self, msg):
        self.last_debug_goal_path = msg

    def _check_contracts(self):
        ok, reason = super()._check_contracts()
        if not ok:
            return ok, reason
        if self.last_bspline is None:
            return False, "waiting_bspline"
        if not self.last_bspline.pos_pts:
            return False, "empty_bspline"
        if self.last_debug_goal_path is None:
            return False, "waiting_debug_goal_path"
        if not self.last_debug_goal_path.poses:
            return False, "empty_debug_goal_path"
        if self.last_state is None:
            return False, "waiting_stage2_state"
        if self.require_paper_line_state_detail and "candidate=" not in self.last_state.detail:
            return False, "waiting_paper_line_state_detail"
        return True, "paper_line_full_chain_ok"

    def _pass(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0
        rospy.loginfo("stage2 paper-line real ego regression PASS %s", reason)
        rospy.signal_shutdown(reason)

    def _fail(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 1
        rospy.logerr("stage2 paper-line real ego regression FAIL %s", reason)
        rospy.signal_shutdown(reason)


if __name__ == "__main__":
    rospy.init_node("stage2_paper_line_regression_monitor")
    node = Stage2PaperLineRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
