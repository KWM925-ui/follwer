#!/usr/bin/env python3
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


DEFAULT_SEQUENCE = [
    {
        "label": "right_bias_follow",
        "delay_sec": 1.0,
        "forward_m": 10.5,
        "lateral_m": 1.8,
        "repeat_count": 3,
        "repeat_interval_sec": 0.05,
        "settle_sec": 10.5,
    },
    {
        "label": "center_obstacle",
        "delay_sec": 0.8,
        "forward_m": 10.5,
        "lateral_m": 0.0,
        "repeat_count": 3,
        "repeat_interval_sec": 0.05,
        "settle_sec": 12.5,
    },
]


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


class Stage2VisualDemoGoalReplayNode:
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.sequence = rospy.get_param("~sequence", DEFAULT_SEQUENCE)
        self.startup_odom_timeout_sec = float(rospy.get_param("~startup_odom_timeout_sec", 8.0))
        self.loop = bool(rospy.get_param("~loop", True))

        self.last_odom = None
        self.last_odom_stamp = None

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=20)

        self._wait_for_odom_or_die()
        self._run_sequence()

    def _odom_cb(self, msg):
        self.last_odom = msg
        self.last_odom_stamp = rospy.Time.now()

    def _wait_for_odom_or_die(self):
        deadline = time.monotonic() + max(self.startup_odom_timeout_sec, 0.5)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.last_odom is not None:
                return
            rate.sleep()
        raise RuntimeError("stage2_visual_demo_goal_replay timed out waiting for odom")

    def _current_goal_pose(self, forward_m, lateral_m):
        odom = self.last_odom
        yaw_rad = yaw_from_quaternion(odom.pose.pose.orientation)
        start_x = float(odom.pose.pose.position.x)
        start_y = float(odom.pose.pose.position.y)
        goal_x = start_x + math.cos(yaw_rad) * float(forward_m) - math.sin(yaw_rad) * float(lateral_m)
        goal_y = start_y + math.sin(yaw_rad) * float(forward_m) + math.cos(yaw_rad) * float(lateral_m)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame_id
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        return msg

    def _sleep_with_shutdown(self, duration_sec):
        end_time = time.monotonic() + max(float(duration_sec), 0.0)
        rate = rospy.Rate(40.0)
        while not rospy.is_shutdown() and time.monotonic() < end_time:
            rate.sleep()

    def _publish_goal_burst(self, goal_msg, repeat_count, repeat_interval_sec):
        for _ in range(max(int(repeat_count), 1)):
            goal_msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(goal_msg)
            self._sleep_with_shutdown(repeat_interval_sec)

    def _run_sequence_once(self):
        for step in self.sequence:
            if rospy.is_shutdown():
                return
            label = str(step.get("label", "step")).strip() or "step"
            self._sleep_with_shutdown(float(step.get("delay_sec", 0.0)))
            goal_msg = self._current_goal_pose(
                step.get("forward_m", 0.0),
                step.get("lateral_m", 0.0),
            )
            rospy.loginfo(
                "stage2_visual_demo_goal_replay step=%s goal=(%.2f, %.2f)",
                label,
                goal_msg.pose.position.x,
                goal_msg.pose.position.y,
            )
            self._publish_goal_burst(
                goal_msg,
                step.get("repeat_count", 3),
                float(step.get("repeat_interval_sec", 0.05)),
            )
            self._sleep_with_shutdown(float(step.get("settle_sec", 8.0)))

    def _run_sequence(self):
        rospy.loginfo(
            "stage2_visual_demo_goal_replay ready: steps=%d loop=%s",
            len(self.sequence),
            self.loop,
        )
        while not rospy.is_shutdown():
            self._run_sequence_once()
            if not self.loop:
                rospy.loginfo("stage2_visual_demo_goal_replay finished one-shot sequence")
                return


if __name__ == "__main__":
    rospy.init_node("stage2_visual_demo_goal_replay")
    Stage2VisualDemoGoalReplayNode()
