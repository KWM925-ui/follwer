#!/usr/bin/env python3
import importlib.util
import os
import re
import sys
import time

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import String

from ego_planner.msg import Bspline
from human_follow_msgs.msg import FollowState

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_BASE_MONITOR_PATH = os.path.join(_SCRIPT_DIR, "stage2_real_ego_full_chain_regression_monitor_node.py")
_BASE_SPEC = importlib.util.spec_from_file_location("stage2_real_ego_full_chain_regression_monitor_impl", _BASE_MONITOR_PATH)
_BASE_MODULE = importlib.util.module_from_spec(_BASE_SPEC)
_BASE_SPEC.loader.exec_module(_BASE_MODULE)
Stage2RealEgoFullChainRegressionMonitorNode = _BASE_MODULE.Stage2RealEgoFullChainRegressionMonitorNode


def _csv_or_list(value):
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value).split(",") if item.strip()]


class Stage2PaperLineRegressionMonitorNode(Stage2RealEgoFullChainRegressionMonitorNode):
    def __init__(self):
        self.bspline_topic = rospy.get_param("~bspline_topic", "/planning/bspline")
        self.state_topic = rospy.get_param("~state_topic", "/follow/stage2/state")
        self.debug_goal_path_topic = rospy.get_param("~debug_goal_path_topic", "/follow/stage2/debug_goal_path")
        self.phase_label_topic = rospy.get_param("~phase_label_topic", "/follow/test/phase_label")
        self.require_paper_line_state_detail = bool(rospy.get_param("~require_paper_line_state_detail", True))
        self.full_duration_evidence = bool(rospy.get_param("~full_duration_evidence", False))
        self.required_state_names = _csv_or_list(rospy.get_param("~required_state_names", ""))
        self.required_phase_labels = _csv_or_list(rospy.get_param("~required_phase_labels", ""))

        self.last_bspline = None
        self.last_state = None
        self.last_debug_goal_path = None
        self.last_contract_reason = "not_checked"
        self.contract_seen = False
        self.seen_state_names = set()
        self.seen_candidate_names = set()
        self.seen_phase_labels = set()

        super().__init__()
        rospy.Subscriber(self.bspline_topic, Bspline, self._bspline_callback, queue_size=20)
        rospy.Subscriber(self.state_topic, FollowState, self._state_callback, queue_size=20)
        rospy.Subscriber(self.debug_goal_path_topic, Path, self._debug_goal_path_callback, queue_size=20)
        rospy.Subscriber(self.phase_label_topic, String, self._phase_label_callback, queue_size=20)
        rospy.loginfo(
            "stage2_paper_line_regression_monitor ready bspline=%s state=%s debug_path=%s full_duration=%s",
            self.bspline_topic,
            self.state_topic,
            self.debug_goal_path_topic,
            self.full_duration_evidence,
        )

    def _bspline_callback(self, msg):
        self.last_bspline = msg

    def _state_callback(self, msg):
        self.last_state = msg
        if msg.state_name:
            self.seen_state_names.add(str(msg.state_name))
        match = re.search(r"candidate=([A-Za-z0-9_\\-]+)", msg.detail or "")
        if match:
            self.seen_candidate_names.add(match.group(1))

    def _debug_goal_path_callback(self, msg):
        self.last_debug_goal_path = msg

    def _phase_label_callback(self, msg):
        if msg.data:
            self.seen_phase_labels.add(str(msg.data))

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

    def _evidence_reason(self):
        states = ",".join(sorted(self.seen_state_names)) or "none"
        candidates = ",".join(sorted(self.seen_candidate_names)) or "none"
        phases = ",".join(sorted(self.seen_phase_labels)) or "none"
        return (
            "distinct_goals=%d distinct_cmds=%d request_count=%d states=%s candidates=%s phases=%s"
            % (
                self.distinct_goal_count,
                self.distinct_cmd_count,
                self.request_count,
                states,
                candidates,
                phases,
            )
        )

    def _missing_required(self, required, seen):
        return [item for item in required if item not in seen]

    def _tick(self, _event):
        if self.finished:
            return
        ok, reason = self._check_contracts()
        if ok:
            self.contract_seen = True
            self.last_contract_reason = reason
            if not self.full_duration_evidence:
                self._pass(self._evidence_reason())
                return
        else:
            self.last_contract_reason = reason

        elapsed = time.monotonic() - self.start_time
        if elapsed < self.max_duration_sec:
            return

        if not self.full_duration_evidence:
            self._fail(
                "timeout reason=%s distinct_goals=%d distinct_cmds=%d request_count=%d last_request=%s"
                % (reason, self.distinct_goal_count, self.distinct_cmd_count, self.request_count, self.last_request)
            )
            return

        missing_states = self._missing_required(self.required_state_names, self.seen_state_names)
        missing_phases = self._missing_required(self.required_phase_labels, self.seen_phase_labels)
        if not self.contract_seen:
            self._fail("full_duration_no_contract last_reason=%s %s" % (self.last_contract_reason, self._evidence_reason()))
            return
        if missing_states:
            self._fail("missing_states=%s %s" % (",".join(missing_states), self._evidence_reason()))
            return
        if missing_phases:
            self._fail("missing_phases=%s %s" % (",".join(missing_phases), self._evidence_reason()))
            return
        self._pass("full_duration %s" % self._evidence_reason())

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
