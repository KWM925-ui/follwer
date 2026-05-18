#!/usr/bin/env python3
import math
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class Stage2SearchGoalRegressionMonitorNode:
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/follow/stage2/goal")
        self.phase_label_topic = rospy.get_param("~phase_label_topic", "/follow/test/phase_label")
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 10.0))
        self.min_search_goal_shift_m = float(rospy.get_param("~min_search_goal_shift_m", 0.35))
        self.required_search_phase = rospy.get_param("~required_search_phase", "search_loss_right")

        self.start_time = time.monotonic()
        self.finished = False
        self.exit_code = 0

        self.current_phase = ""
        self.phase_enter_time = None
        self.pre_search_goal = None
        self.search_first_goal = None
        self.search_last_goal = None
        self.search_goal_count = 0

        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback, queue_size=20)
        rospy.Subscriber(self.phase_label_topic, String, self._phase_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

        rospy.loginfo(
            "stage2_search_goal_regression_monitor ready goal=%s phase=%s required_phase=%s",
            self.goal_topic,
            self.phase_label_topic,
            self.required_search_phase,
        )

    def _goal_xyz(self, msg):
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
            self.phase_enter_time = time.monotonic()
            self.search_first_goal = None
            self.search_last_goal = None
            self.search_goal_count = 0
        self.current_phase = new_phase

    def _goal_callback(self, msg):
        goal_xyz = self._goal_xyz(msg)
        if self.current_phase != self.required_search_phase:
            self.pre_search_goal = goal_xyz
            return

        if self.search_first_goal is None:
            self.search_first_goal = goal_xyz
        self.search_last_goal = goal_xyz
        self.search_goal_count += 1

    def _fail(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 1
        rospy.logerr("stage2 search goal regression FAIL %s", reason)
        rospy.signal_shutdown(reason)

    def _pass(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0
        rospy.loginfo("stage2 search goal regression PASS %s", reason)
        rospy.signal_shutdown(reason)

    def _tick(self, _event):
        if self.finished:
            return

        if self.current_phase == self.required_search_phase:
            if self.pre_search_goal is None:
                return
            if self.search_first_goal is None or self.search_last_goal is None:
                return

            first_shift = self._distance(self.search_first_goal, self.pre_search_goal)
            search_shift = self._distance(self.search_last_goal, self.search_first_goal)
            if self.search_goal_count >= 2 and search_shift >= self.min_search_goal_shift_m:
                self._pass(
                    "search_goal_count=%d first_shift=%.3f search_shift=%.3f"
                    % (self.search_goal_count, first_shift, search_shift)
                )
                return

        if (time.monotonic() - self.start_time) >= self.max_duration_sec:
            self._fail(
                "timeout phase=%s search_goal_count=%d"
                % (self.current_phase or "<none>", self.search_goal_count)
            )


if __name__ == "__main__":
    rospy.init_node("stage2_search_goal_regression_monitor")
    node = Stage2SearchGoalRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
