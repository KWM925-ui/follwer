#!/usr/bin/env python3
import math

import rospy
from nav_msgs.msg import Odometry

from human_follow_msgs.msg import Target3D


def yaw_from_quaternion(x_value, y_value, z_value, w_value):
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


class BodyTruthWorldProjectorNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/sim/truth_target_body")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.output_topic = rospy.get_param("~output_topic", "/follow/sim/human_truth_world")
        self.output_frame_id = rospy.get_param("~output_frame_id", "map")
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.6))

        self.last_odom = None
        self.last_odom_stamp = None
        self.publisher = rospy.Publisher(self.output_topic, Target3D, queue_size=10)
        self.truth_sub = rospy.Subscriber(self.input_topic, Target3D, self._truth_callback, queue_size=20)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)

        rospy.loginfo(
            "body_target_world_projector ready: input=%s odom=%s output=%s",
            self.input_topic,
            self.odom_topic,
            self.output_topic,
        )

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_stamp = rospy.Time.now()

    def _odom_is_fresh(self):
        if self.last_odom is None or self.last_odom_stamp is None:
            return False
        return (rospy.Time.now() - self.last_odom_stamp).to_sec() <= self.odom_timeout_sec

    def _truth_callback(self, msg):
        if rospy.is_shutdown():
            return
        out = Target3D()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.output_frame_id
        out.track_id = msg.track_id
        out.frame_id = self.output_frame_id
        out.confidence = msg.confidence
        out.support_points = msg.support_points

        if not msg.valid or not self._odom_is_fresh():
            out.valid = False
            out.position.x = 0.0
            out.position.y = 0.0
            out.position.z = 0.0
            try:
                self.publisher.publish(out)
            except rospy.ROSException:
                return
            return

        odom = self.last_odom
        yaw_rad = yaw_from_quaternion(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)

        relative_x = float(msg.position.x)
        relative_y = float(msg.position.y)
        relative_z = float(msg.position.z)

        out.valid = True
        out.position.x = float(odom.pose.pose.position.x) + cos_yaw * relative_x - sin_yaw * relative_y
        out.position.y = float(odom.pose.pose.position.y) + sin_yaw * relative_x + cos_yaw * relative_y
        out.position.z = float(odom.pose.pose.position.z) + relative_z
        try:
            self.publisher.publish(out)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("body_truth_world_projector")
    BodyTruthWorldProjectorNode()
    rospy.spin()
