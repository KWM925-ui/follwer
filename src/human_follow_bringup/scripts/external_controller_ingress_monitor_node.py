#!/usr/bin/env python3
import sys
import time

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import FollowCommand, FollowState
from human_follow_msgs.msg import Target3D


class ExternalControllerIngressMonitorNode:
    def __init__(self):
        self.follow_state_topic = rospy.get_param("~follow_state_topic", "/follow/state")
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.planning_path_topic = rospy.get_param("~planning_path_topic", "/follow/planning/debug_path")
        self.target_world_topic = rospy.get_param("~target_world_topic", "/follow/fusion/target_world")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/follow/lidar/points")
        self.controller_type = rospy.get_param("~controller_type", "")
        self.require_planning_path = bool(rospy.get_param("~require_planning_path", False))
        self.check_rate_hz = float(rospy.get_param("~check_rate_hz", 20.0))
        self.startup_grace_sec = float(rospy.get_param("~startup_grace_sec", 0.5))
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 24.0))
        self.min_nonzero_command_count = int(rospy.get_param("~min_nonzero_command_count", 3))
        self.min_planning_path_count = int(rospy.get_param("~min_planning_path_count", 2))
        self.min_target_world_valid_count = int(rospy.get_param("~min_target_world_valid_count", 3))
        self.min_odom_count = int(rospy.get_param("~min_odom_count", 5))
        self.min_nonempty_pointcloud_count = int(rospy.get_param("~min_nonempty_pointcloud_count", 2))

        self.start_time = time.monotonic()
        self.finished = False
        self.exit_code = 0

        self.seen_follow_states = set()
        self.nonzero_command_count = 0
        self.planning_path_count = 0
        self.target_world_valid_count = 0
        self.odom_count = 0
        self.nonempty_pointcloud_count = 0

        self.follow_state_sub = rospy.Subscriber(
            self.follow_state_topic, FollowState, self._follow_state_callback, queue_size=20
        )
        self.command_sub = rospy.Subscriber(
            self.command_topic, FollowCommand, self._command_callback, queue_size=20
        )
        self.path_sub = rospy.Subscriber(
            self.planning_path_topic, Path, self._planning_path_callback, queue_size=20
        )
        self.target_world_sub = rospy.Subscriber(
            self.target_world_topic, Target3D, self._target_world_callback, queue_size=20
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=20
        )
        self.pointcloud_sub = rospy.Subscriber(
            self.pointcloud_topic, PointCloud2, self._pointcloud_callback, queue_size=5
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.check_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "external_controller_ingress_monitor ready: state=%s cmd=%s path=%s target_world=%s odom=%s cloud=%s controller_type=%s require_path=%s",
            self.follow_state_topic,
            self.command_topic,
            self.planning_path_topic,
            self.target_world_topic,
            self.odom_topic,
            self.pointcloud_topic,
            self.controller_type or "unknown",
            "true" if self.require_planning_path else "false",
        )

    def _follow_state_callback(self, msg):
        if msg.state_name:
            self.seen_follow_states.add(msg.state_name)

    def _command_callback(self, msg):
        if not msg.valid:
            return
        if msg.mode == "body_velocity_yaw_rate":
            if abs(msg.forward_mps) > 1e-3 or abs(msg.lateral_mps) > 1e-3 or abs(msg.yaw_rate_rps) > 1e-3:
                self.nonzero_command_count += 1
        elif msg.mode == "search_yaw_only":
            if abs(msg.yaw_rate_rps) > 1e-3:
                self.nonzero_command_count += 1

    def _planning_path_callback(self, msg):
        if len(msg.poses) >= 2:
            self.planning_path_count += 1

    def _target_world_callback(self, msg):
        if msg.valid:
            self.target_world_valid_count += 1

    def _odom_callback(self, _msg):
        self.odom_count += 1

    def _pointcloud_callback(self, msg):
        if int(msg.width) > 0:
            self.nonempty_pointcloud_count += 1

    def _missing_requirements(self):
        missing = []
        for state_name in ("target_acquired", "follow", "search", "lost"):
            if state_name not in self.seen_follow_states:
                missing.append("state:%s" % state_name)
        if self.nonzero_command_count < self.min_nonzero_command_count:
            missing.append("nonzero_command_count<%d" % self.min_nonzero_command_count)
        if self.require_planning_path and self.planning_path_count < self.min_planning_path_count:
            missing.append("planning_path_count<%d" % self.min_planning_path_count)
        if self.require_planning_path and self.target_world_valid_count < self.min_target_world_valid_count:
            missing.append("target_world_valid_count<%d" % self.min_target_world_valid_count)
        if self.require_planning_path and self.odom_count < self.min_odom_count:
            missing.append("odom_count<%d" % self.min_odom_count)
        if self.require_planning_path and self.nonempty_pointcloud_count < self.min_nonempty_pointcloud_count:
            missing.append("nonempty_pointcloud_count<%d" % self.min_nonempty_pointcloud_count)
        return missing

    def _finish(self, success, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0 if success else 1
        if success:
            rospy.loginfo(
                "external ingress PASS controller_type=%s states=%s nonzero_commands=%d planning_paths=%d target_world_valid=%d odom=%d nonempty_pointcloud=%d",
                self.controller_type or "unknown",
                sorted(self.seen_follow_states),
                self.nonzero_command_count,
                self.planning_path_count,
                self.target_world_valid_count,
                self.odom_count,
                self.nonempty_pointcloud_count,
            )
        else:
            rospy.logerr(
                "external ingress FAIL controller_type=%s reason=%s",
                self.controller_type or "unknown",
                reason,
            )
        rospy.signal_shutdown("external ingress monitor complete")

    def _tick(self, _event):
        if self.finished:
            return
        elapsed_sec = time.monotonic() - self.start_time
        if elapsed_sec < self.startup_grace_sec:
            return

        missing = self._missing_requirements()
        if not missing:
            self._finish(True, "all requirements satisfied")
            return

        if elapsed_sec >= self.max_duration_sec:
            self._finish(False, "missing=" + ",".join(missing))


def main():
    rospy.init_node("external_controller_ingress_monitor")
    node = ExternalControllerIngressMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
