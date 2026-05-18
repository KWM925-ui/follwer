#!/usr/bin/env python3
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from quadrotor_msgs.msg import PositionCommand


def yaw_from_quaternion(z_value, w_value):
    return 2.0 * math.atan2(float(z_value), float(w_value))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class Stage2PlaceholderPlannerStubNode:
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom_world")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/grid_map/cloud")
        self.output_topic = rospy.get_param("~output_topic", "/follow/stage2/ego_position_cmd")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.goal_timeout_sec = float(rospy.get_param("~goal_timeout_sec", 0.6))
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.6))
        self.cloud_timeout_sec = float(rospy.get_param("~cloud_timeout_sec", 0.6))
        self.velocity_gain = float(rospy.get_param("~velocity_gain", 0.45))
        self.max_velocity_mps = float(rospy.get_param("~max_velocity_mps", 0.8))
        self.position_deadband_m = float(rospy.get_param("~position_deadband_m", 0.15))

        self.last_goal = None
        self.last_goal_time = None
        self.last_odom = None
        self.last_odom_time = None
        self.last_cloud = None
        self.last_cloud_time = None

        self.publisher = rospy.Publisher(self.output_topic, PositionCommand, queue_size=20)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback, queue_size=20)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self._cloud_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "stage2_placeholder_planner_stub ready goal=%s odom=%s cloud=%s output=%s",
            self.goal_topic,
            self.odom_topic,
            self.cloud_topic,
            self.output_topic,
        )

    def _goal_callback(self, msg):
        self.last_goal = msg
        self.last_goal_time = time.monotonic()

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_time = time.monotonic()

    def _cloud_callback(self, msg):
        self.last_cloud = msg
        self.last_cloud_time = time.monotonic()

    def _is_fresh(self, stamp, timeout_sec):
        return stamp is not None and (time.monotonic() - stamp) <= timeout_sec

    def _inputs_ready(self):
        if self.last_goal is None or not self._is_fresh(self.last_goal_time, self.goal_timeout_sec):
            return False
        if self.last_odom is None or not self._is_fresh(self.last_odom_time, self.odom_timeout_sec):
            return False
        if self.last_cloud is None or not self._is_fresh(self.last_cloud_time, self.cloud_timeout_sec):
            return False
        return int(self.last_cloud.width) > 0 and int(self.last_cloud.height) > 0

    def _build_command(self):
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.last_goal.header.frame_id or "map"

        goal_position = self.last_goal.pose.position
        odom_position = self.last_odom.pose.pose.position
        delta_x = float(goal_position.x) - float(odom_position.x)
        delta_y = float(goal_position.y) - float(odom_position.y)
        delta_z = float(goal_position.z) - float(odom_position.z)

        msg.position.x = float(goal_position.x)
        msg.position.y = float(goal_position.y)
        msg.position.z = float(goal_position.z)

        distance = math.sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z)
        if distance >= self.position_deadband_m:
            msg.velocity.x = clamp(delta_x * self.velocity_gain, -self.max_velocity_mps, self.max_velocity_mps)
            msg.velocity.y = clamp(delta_y * self.velocity_gain, -self.max_velocity_mps, self.max_velocity_mps)
            msg.velocity.z = clamp(delta_z * self.velocity_gain, -self.max_velocity_mps, self.max_velocity_mps)
        else:
            msg.velocity.x = 0.0
            msg.velocity.y = 0.0
            msg.velocity.z = 0.0

        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0
        msg.yaw = yaw_from_quaternion(self.last_goal.pose.orientation.z, self.last_goal.pose.orientation.w)
        msg.yaw_dot = 0.0
        msg.vel_norm = abs(msg.velocity.x) + abs(msg.velocity.y) + abs(msg.velocity.z)
        msg.acc_norm = 0.0
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return msg

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        if not self._inputs_ready():
            return
        try:
            self.publisher.publish(self._build_command())
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("stage2_placeholder_planner_stub")
    Stage2PlaceholderPlannerStubNode()
    rospy.spin()
