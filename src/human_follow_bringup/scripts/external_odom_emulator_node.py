#!/usr/bin/env python3

import math

import rospy
from nav_msgs.msg import Odometry


def _diag_to_covariance(diag_values):
    values = list(diag_values)
    if len(values) != 6:
        raise ValueError("expected 6 covariance diagonal values")
    covariance = [0.0] * 36
    for index, value in enumerate(values):
        covariance[index * 6 + index] = float(value)
    return covariance


def _yaw_to_quaternion(yaw_rad):
    half_yaw = 0.5 * yaw_rad
    return math.sin(half_yaw), math.cos(half_yaw)


def main():
    rospy.init_node("external_odom_emulator")

    output_topic = rospy.get_param("~output_topic", "/mavros/odometry/out")
    frame_id = rospy.get_param("~frame_id", "odom")
    child_frame_id = rospy.get_param("~child_frame_id", "base_link")
    rate_hz = float(rospy.get_param("~rate_hz", 30.0))

    position_x = float(rospy.get_param("~position_x", 0.0))
    position_y = float(rospy.get_param("~position_y", 0.0))
    position_z = float(rospy.get_param("~position_z", 0.0))
    yaw_rad = float(rospy.get_param("~yaw_rad", 0.0))

    velocity_x = float(rospy.get_param("~velocity_x", 0.0))
    velocity_y = float(rospy.get_param("~velocity_y", 0.0))
    velocity_z = float(rospy.get_param("~velocity_z", 0.0))

    pose_cov_diag = rospy.get_param("~pose_cov_diag", [1.0e-4] * 6)
    twist_cov_diag = rospy.get_param("~twist_cov_diag", [1.0e-4] * 6)

    quat_z, quat_w = _yaw_to_quaternion(yaw_rad)

    message = Odometry()
    message.header.frame_id = frame_id
    message.child_frame_id = child_frame_id
    message.pose.pose.position.x = position_x
    message.pose.pose.position.y = position_y
    message.pose.pose.position.z = position_z
    message.pose.pose.orientation.z = quat_z
    message.pose.pose.orientation.w = quat_w
    message.twist.twist.linear.x = velocity_x
    message.twist.twist.linear.y = velocity_y
    message.twist.twist.linear.z = velocity_z
    message.pose.covariance = _diag_to_covariance(pose_cov_diag)
    message.twist.covariance = _diag_to_covariance(twist_cov_diag)

    publisher = rospy.Publisher(output_topic, Odometry, queue_size=10)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        message.header.stamp = rospy.Time.now()
        try:
            publisher.publish(message)
        except rospy.ROSException:
            if rospy.is_shutdown():
                break
        rate.sleep()


if __name__ == "__main__":
    main()
