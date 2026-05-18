#!/usr/bin/env python3
import math
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import Target3D


class Stage2GoalAdapterRegressionMonitorNode:
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/follow/stage2/goal")
        self.debug_path_topic = rospy.get_param("~debug_path_topic", "/follow/stage2/debug_goal_path")
        self.ego_goal_topic = rospy.get_param("~ego_goal_topic", "/move_base_simple/goal")
        self.ego_odom_topic = rospy.get_param("~ego_odom_topic", "/odom_world")
        self.ego_grid_odom_topic = rospy.get_param("~ego_grid_odom_topic", "/grid_map/odom")
        self.ego_cloud_topic = rospy.get_param("~ego_cloud_topic", "/grid_map/cloud")
        self.target_world_topic = rospy.get_param("~target_world_topic", "/follow/fusion/target_world")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/lio/odom")
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 8.0))
        self.follow_distance_m = float(rospy.get_param("~follow_distance_m", 3.5))
        self.goal_tolerance_m = float(rospy.get_param("~goal_tolerance_m", 0.12))
        self.yaw_tolerance_rad = float(rospy.get_param("~yaw_tolerance_rad", 0.12))
        self.min_distinct_goals = int(rospy.get_param("~min_distinct_goals", 3))
        self.goal_change_threshold_m = float(rospy.get_param("~goal_change_threshold_m", 0.25))

        self.start_time = time.monotonic()
        self.last_goal = None
        self.last_goal_time = None
        self.last_debug_path = None
        self.last_ego_goal = None
        self.last_ego_odom = None
        self.last_ego_grid_odom = None
        self.last_ego_cloud = None
        self.last_target = None
        self.last_odom = None

        self.goal_seen = False
        self.debug_path_seen = False
        self.ego_goal_seen = False
        self.ego_odom_seen = False
        self.ego_grid_odom_seen = False
        self.ego_cloud_seen = False
        self.distinct_goal_count = 0
        self._last_counted_goal = None
        self.finished = False
        self.exit_code = 0

        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback, queue_size=20)
        rospy.Subscriber(self.debug_path_topic, Path, self._debug_path_callback, queue_size=20)
        rospy.Subscriber(self.ego_goal_topic, PoseStamped, self._ego_goal_callback, queue_size=20)
        rospy.Subscriber(self.ego_odom_topic, Odometry, self._ego_odom_callback, queue_size=20)
        rospy.Subscriber(self.ego_grid_odom_topic, Odometry, self._ego_grid_odom_callback, queue_size=20)
        rospy.Subscriber(self.ego_cloud_topic, PointCloud2, self._ego_cloud_callback, queue_size=20)
        rospy.Subscriber(self.target_world_topic, Target3D, self._target_passthrough_callback, queue_size=20)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

        rospy.loginfo(
            "stage2_goal_adapter_regression_monitor ready goal=%s ego_goal=%s ego_odom=%s cloud=%s",
            self.goal_topic,
            self.ego_goal_topic,
            self.ego_odom_topic,
            self.ego_cloud_topic,
        )

    def _target_passthrough_callback(self, msg):
        self.last_target = msg

    def _odom_callback(self, msg):
        self.last_odom = msg

    def _goal_callback(self, msg):
        self.last_goal = msg
        self.last_goal_time = time.monotonic()
        self.goal_seen = True
        goal_xyz = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        if self._last_counted_goal is None:
            self._last_counted_goal = goal_xyz
            self.distinct_goal_count = 1
            return

        dx = goal_xyz[0] - self._last_counted_goal[0]
        dy = goal_xyz[1] - self._last_counted_goal[1]
        dz = goal_xyz[2] - self._last_counted_goal[2]
        if math.sqrt(dx * dx + dy * dy + dz * dz) >= self.goal_change_threshold_m:
            self._last_counted_goal = goal_xyz
            self.distinct_goal_count += 1

    def _debug_path_callback(self, msg):
        self.last_debug_path = msg
        if len(msg.poses) >= 3:
            self.debug_path_seen = True

    def _ego_goal_callback(self, msg):
        self.last_ego_goal = msg
        self.ego_goal_seen = True

    def _ego_odom_callback(self, msg):
        self.last_ego_odom = msg
        self.ego_odom_seen = True

    def _ego_grid_odom_callback(self, msg):
        self.last_ego_grid_odom = msg
        self.ego_grid_odom_seen = True

    def _ego_cloud_callback(self, msg):
        self.last_ego_cloud = msg
        if int(msg.width) > 0 and int(msg.height) > 0:
            self.ego_cloud_seen = True

    def _fail(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 1
        rospy.logerr("stage2 goal/adapter regression FAIL %s", reason)
        rospy.signal_shutdown(reason)

    def _pass(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0
        rospy.loginfo("stage2 goal/adapter regression PASS %s", reason)
        rospy.signal_shutdown(reason)

    def _yaw_from_pose(self, pose_msg):
        z_value = float(pose_msg.pose.orientation.z)
        w_value = float(pose_msg.pose.orientation.w)
        return 2.0 * math.atan2(z_value, w_value)

    def _check_goal_geometry(self):
        if self.last_goal is None or self.last_target is None or self.last_odom is None:
            return False, "waiting_geometry_inputs"

        target_x = float(self.last_target.position.x)
        target_y = float(self.last_target.position.y)
        target_z = float(self.last_target.position.z)
        odom_x = float(self.last_odom.pose.pose.position.x)
        odom_y = float(self.last_odom.pose.pose.position.y)
        odom_z = float(self.last_odom.pose.pose.position.z)

        delta_x = odom_x - target_x
        delta_y = odom_y - target_y
        distance = math.hypot(delta_x, delta_y)
        if distance < 1e-6:
            return False, "degenerate_target_odom_distance"

        unit_x = delta_x / distance
        unit_y = delta_y / distance
        expected_x = target_x + unit_x * self.follow_distance_m
        expected_y = target_y + unit_y * self.follow_distance_m
        expected_z = max(odom_z, target_z)
        expected_yaw = math.atan2(target_y - expected_y, target_x - expected_x)

        goal_x = float(self.last_goal.pose.position.x)
        goal_y = float(self.last_goal.pose.position.y)
        goal_z = float(self.last_goal.pose.position.z)
        goal_yaw = self._yaw_from_pose(self.last_goal)

        dx = abs(goal_x - expected_x)
        dy = abs(goal_y - expected_y)
        dz = abs(goal_z - expected_z)
        yaw_error = abs(math.atan2(math.sin(goal_yaw - expected_yaw), math.cos(goal_yaw - expected_yaw)))
        if dx > self.goal_tolerance_m or dy > self.goal_tolerance_m or dz > self.goal_tolerance_m:
            return False, "goal_geometry_mismatch dx=%.3f dy=%.3f dz=%.3f" % (dx, dy, dz)
        if yaw_error > self.yaw_tolerance_rad:
            return False, "goal_yaw_mismatch err=%.3f" % yaw_error
        return True, "goal_geometry_ok"

    def _check_adapter_echo(self):
        if self.last_goal is None or self.last_ego_goal is None:
            return False, "waiting_goal_adapter"
        if self.last_ego_odom is None or self.last_ego_grid_odom is None:
            return False, "waiting_odom_adapter"
        if self.last_ego_cloud is None:
            return False, "waiting_cloud_adapter"

        goal = self.last_goal.pose.position
        ego_goal = self.last_ego_goal.pose.position
        if (
            abs(float(goal.x) - float(ego_goal.x)) > self.goal_tolerance_m
            or abs(float(goal.y) - float(ego_goal.y)) > self.goal_tolerance_m
            or abs(float(goal.z) - float(ego_goal.z)) > self.goal_tolerance_m
        ):
            return False, "ego_goal_echo_mismatch"

        odom = self.last_odom.pose.pose.position if self.last_odom is not None else None
        if odom is None:
            return False, "waiting_source_odom"
        ego_odom = self.last_ego_odom.pose.pose.position
        ego_grid = self.last_ego_grid_odom.pose.pose.position
        if (
            abs(float(odom.x) - float(ego_odom.x)) > self.goal_tolerance_m
            or abs(float(odom.y) - float(ego_odom.y)) > self.goal_tolerance_m
            or abs(float(odom.z) - float(ego_odom.z)) > self.goal_tolerance_m
        ):
            return False, "ego_odom_echo_mismatch"
        if (
            abs(float(odom.x) - float(ego_grid.x)) > self.goal_tolerance_m
            or abs(float(odom.y) - float(ego_grid.y)) > self.goal_tolerance_m
            or abs(float(odom.z) - float(ego_grid.z)) > self.goal_tolerance_m
        ):
            return False, "grid_odom_echo_mismatch"

        if int(self.last_ego_cloud.width) <= 0 or int(self.last_ego_cloud.height) <= 0:
            return False, "ego_cloud_empty"
        return True, "adapter_echo_ok"

    def _tick(self, _event):
        if self.finished:
            return

        geometry_ok, geometry_reason = self._check_goal_geometry()
        adapter_ok, adapter_reason = self._check_adapter_echo()
        if (
            self.goal_seen
            and self.debug_path_seen
            and self.ego_goal_seen
            and self.ego_odom_seen
            and self.ego_grid_odom_seen
            and self.ego_cloud_seen
            and self.distinct_goal_count >= self.min_distinct_goals
            and geometry_ok
            and adapter_ok
        ):
            self._pass(
                "goal generator and adapter contracts satisfied distinct_goals=%d"
                % self.distinct_goal_count
            )
            return

        if (time.monotonic() - self.start_time) >= self.max_duration_sec:
            self._fail(
                "timeout goal_seen=%s debug_path=%s ego_goal=%s ego_odom=%s grid_odom=%s cloud=%s distinct_goals=%d geometry=%s adapter=%s"
                % (
                    self.goal_seen,
                    self.debug_path_seen,
                    self.ego_goal_seen,
                    self.ego_odom_seen,
                    self.ego_grid_odom_seen,
                    self.ego_cloud_seen,
                    self.distinct_goal_count,
                    geometry_reason,
                    adapter_reason,
                )
            )


if __name__ == "__main__":
    rospy.init_node("stage2_goal_adapter_regression_monitor")
    node = Stage2GoalAdapterRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
