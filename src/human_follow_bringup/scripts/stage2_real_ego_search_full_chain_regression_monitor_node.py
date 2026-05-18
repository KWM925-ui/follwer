#!/usr/bin/env python3
import importlib.util
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_BASE_MONITOR_PATH = os.path.join(_SCRIPT_DIR, "stage2_real_ego_full_chain_regression_monitor_node.py")
_BASE_SPEC = importlib.util.spec_from_file_location("stage2_real_ego_full_chain_regression_monitor_impl", _BASE_MONITOR_PATH)
_BASE_MODULE = importlib.util.module_from_spec(_BASE_SPEC)
_BASE_SPEC.loader.exec_module(_BASE_MODULE)
Stage2RealEgoFullChainRegressionMonitorNode = _BASE_MODULE.Stage2RealEgoFullChainRegressionMonitorNode


class Stage2RealEgoSearchFullChainRegressionMonitorNode(Stage2RealEgoFullChainRegressionMonitorNode):
    def __init__(self):
        self.phase_label_topic = rospy.get_param("~phase_label_topic", "/follow/test/phase_label")
        self.required_search_phase = rospy.get_param("~required_search_phase", "search_loss_right")
        self.min_search_goal_shift_m = float(rospy.get_param("~min_search_goal_shift_m", 0.35))
        self.min_search_cmd_shift_m = float(rospy.get_param("~min_search_cmd_shift_m", 0.20))

        self.current_phase = ""
        self.pre_search_goal = None
        self.search_first_goal = None
        self.search_last_goal = None
        self.search_goal_count = 0
        self.search_first_waypoint = None
        self.search_last_waypoint = None
        self.search_waypoint_count = 0
        self.search_first_cmd = None
        self.search_last_cmd = None
        self.search_cmd_count = 0

        super().__init__()
        rospy.Subscriber(self.phase_label_topic, String, self._phase_callback, queue_size=20)
        rospy.loginfo(
            "stage2_real_ego_search_full_chain_regression_monitor ready phase=%s required_phase=%s",
            self.phase_label_topic,
            self.required_search_phase,
        )

    def _xyz_from_pose_msg(self, msg):
        return (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

    def _distance(self, a_xyz, b_xyz):
        dx = a_xyz[0] - b_xyz[0]
        dy = a_xyz[1] - b_xyz[1]
        dz = a_xyz[2] - b_xyz[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _phase_callback(self, msg):
        new_phase = msg.data.strip()
        if new_phase == self.current_phase:
            return
        if new_phase == self.required_search_phase:
            self.search_first_goal = None
            self.search_last_goal = None
            self.search_goal_count = 0
            self.search_first_waypoint = None
            self.search_last_waypoint = None
            self.search_waypoint_count = 0
            self.search_first_cmd = None
            self.search_last_cmd = None
            self.search_cmd_count = 0
        self.current_phase = new_phase

    def _goal_callback(self, msg):
        super()._goal_callback(msg)
        goal_xyz = self._xyz_from_pose_msg(msg)
        if self.current_phase != self.required_search_phase:
            self.pre_search_goal = goal_xyz
            return
        if self.search_first_goal is None:
            self.search_first_goal = goal_xyz
        self.search_last_goal = goal_xyz
        self.search_goal_count += 1

    def _ego_waypoint_callback(self, msg):
        super()._ego_waypoint_callback(msg)
        if self.current_phase != self.required_search_phase:
            return
        if not isinstance(msg, Path) or not msg.poses:
            return
        waypoint_xyz = self._xyz_from_pose_msg(msg.poses[0])
        if self.search_first_waypoint is None:
            self.search_first_waypoint = waypoint_xyz
        self.search_last_waypoint = waypoint_xyz
        self.search_waypoint_count += 1

    def _ego_cmd_callback(self, msg):
        super()._ego_cmd_callback(msg)
        if self.current_phase != self.required_search_phase:
            return
        cmd_xyz = (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        )
        if self.search_first_cmd is None:
            self.search_first_cmd = cmd_xyz
        self.search_last_cmd = cmd_xyz
        self.search_cmd_count += 1

    def _check_search_contracts(self):
        if self.current_phase != self.required_search_phase:
            return False, "waiting_search_phase"
        if self.pre_search_goal is None:
            return False, "waiting_pre_search_goal"
        if self.search_first_goal is None or self.search_last_goal is None:
            return False, "waiting_search_goal_samples"
        if self.search_first_waypoint is None or self.search_last_waypoint is None:
            return False, "waiting_search_waypoint_samples"
        if self.search_first_cmd is None or self.search_last_cmd is None:
            return False, "waiting_search_cmd_samples"

        search_goal_shift = self._distance(self.search_last_goal, self.search_first_goal)
        if self.search_goal_count < 2 or search_goal_shift < self.min_search_goal_shift_m:
            return False, "search_goal_shift_too_small"

        waypoint_shift = self._distance(self.search_last_waypoint, self.search_first_waypoint)
        if self.search_waypoint_count < 1 or waypoint_shift < self.min_search_goal_shift_m:
            return False, "search_waypoint_shift_too_small"

        cmd_shift = self._distance(self.search_last_cmd, self.search_first_cmd)
        if self.search_cmd_count < 2 or cmd_shift < self.min_search_cmd_shift_m:
            return False, "search_cmd_shift_too_small"

        return True, (
            "search_goal_count=%d search_waypoint_count=%d search_cmd_count=%d goal_shift=%.3f waypoint_shift=%.3f cmd_shift=%.3f"
            % (
                self.search_goal_count,
                self.search_waypoint_count,
                self.search_cmd_count,
                search_goal_shift,
                waypoint_shift,
                cmd_shift,
            )
        )

    def _tick(self, _event):
        if self.finished:
            return
        ok, reason = self._check_contracts()
        if ok:
            search_ok, search_reason = self._check_search_contracts()
            if search_ok:
                self._pass(
                    "distinct_goals=%d distinct_cmds=%d request_count=%d %s"
                    % (self.distinct_goal_count, self.distinct_cmd_count, self.request_count, search_reason)
                )
                return
            reason = search_reason
        if (time.monotonic() - self.start_time) >= self.max_duration_sec:
            self._fail(
                "timeout reason=%s distinct_goals=%d distinct_cmds=%d request_count=%d last_request=%s phase=%s search_goal_count=%d search_cmd_count=%d"
                % (
                    reason,
                    self.distinct_goal_count,
                    self.distinct_cmd_count,
                    self.request_count,
                    self.last_request,
                    self.current_phase or "<none>",
                    self.search_goal_count,
                    self.search_cmd_count,
                )
            )


if __name__ == "__main__":
    rospy.init_node("stage2_real_ego_search_full_chain_regression_monitor")
    node = Stage2RealEgoSearchFullChainRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
