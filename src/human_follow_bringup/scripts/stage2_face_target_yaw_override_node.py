#!/usr/bin/env python3
import math

import rospy
from nav_msgs.msg import Odometry

from human_follow_msgs.msg import Target3D
from quadrotor_msgs.msg import PositionCommand


def wrap_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


class Stage2FaceTargetYawOverrideNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/stage2/ego_position_cmd")
        self.output_topic = rospy.get_param("~output_topic", "/follow/stage2/ego_position_cmd_face_target")
        self.target_world_topic = rospy.get_param("~target_world_topic", "/follow/fusion/target_world")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.target_timeout_sec = float(rospy.get_param("~target_timeout_sec", 0.8))
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.8))
        self.min_face_distance_m = float(rospy.get_param("~min_face_distance_m", 0.2))

        self.last_target = None
        self.last_target_stamp = None
        self.last_odom = None
        self.last_odom_stamp = None

        self.publisher = rospy.Publisher(self.output_topic, PositionCommand, queue_size=20)
        rospy.Subscriber(self.input_topic, PositionCommand, self._cmd_callback, queue_size=20)
        rospy.Subscriber(self.target_world_topic, Target3D, self._target_callback, queue_size=20)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)

        rospy.loginfo(
            "stage2_face_target_yaw_override ready: cmd=%s -> %s target=%s odom=%s",
            self.input_topic,
            self.output_topic,
            self.target_world_topic,
            self.odom_topic,
        )

    def _target_callback(self, msg):
        if not msg.valid:
            return
        self.last_target = msg
        self.last_target_stamp = rospy.Time.now()

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_stamp = rospy.Time.now()

    def _target_is_fresh(self):
        return (
            self.last_target is not None
            and self.last_target_stamp is not None
            and (rospy.Time.now() - self.last_target_stamp).to_sec() <= self.target_timeout_sec
        )

    def _odom_is_fresh(self):
        return (
            self.last_odom is not None
            and self.last_odom_stamp is not None
            and (rospy.Time.now() - self.last_odom_stamp).to_sec() <= self.odom_timeout_sec
        )

    def _cmd_callback(self, msg):
        out = PositionCommand()
        out.header = msg.header
        out.position = msg.position
        out.velocity = msg.velocity
        out.acceleration = msg.acceleration
        out.jerk = msg.jerk
        out.angular_velocity = msg.angular_velocity
        out.attitude = msg.attitude
        out.thrust = msg.thrust
        out.yaw = msg.yaw
        out.yaw_dot = msg.yaw_dot
        out.vel_norm = msg.vel_norm
        out.acc_norm = msg.acc_norm
        out.kx = msg.kx
        out.kv = msg.kv
        out.trajectory_id = msg.trajectory_id
        out.trajectory_flag = msg.trajectory_flag

        if self._target_is_fresh() and self._odom_is_fresh():
            cmd_x = float(self.last_odom.pose.pose.position.x)
            cmd_y = float(self.last_odom.pose.pose.position.y)
            target_x = float(self.last_target.position.x)
            target_y = float(self.last_target.position.y)
            delta_x = target_x - cmd_x
            delta_y = target_y - cmd_y
            if math.hypot(delta_x, delta_y) >= self.min_face_distance_m:
                out.yaw = wrap_pi(math.atan2(delta_y, delta_x))
                out.yaw_dot = 0.0

        try:
            self.publisher.publish(out)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("stage2_face_target_yaw_override")
    Stage2FaceTargetYawOverrideNode()
    rospy.spin()
