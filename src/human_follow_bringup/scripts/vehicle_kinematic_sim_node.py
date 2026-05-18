#!/usr/bin/env python3
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from human_follow_msgs.msg import FollowCommand
from quadrotor_msgs.msg import PositionCommand


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def quaternion_from_yaw(yaw_rad):
    half = 0.5 * float(yaw_rad)
    return 0.0, 0.0, math.sin(half), math.cos(half)


def wrap_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


class VehicleKinematicSimNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/control/cmd_body")
        self.input_mode = str(rospy.get_param("~input_mode", "follow_command")).strip() or "follow_command"
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.6))
        self.linear_velocity_scale = float(rospy.get_param("~linear_velocity_scale", 0.55))
        self.max_forward_mps = float(rospy.get_param("~max_forward_mps", 1.2))
        self.max_lateral_mps = float(rospy.get_param("~max_lateral_mps", 1.0))
        self.max_yaw_rate_rps = float(rospy.get_param("~max_yaw_rate_rps", 0.8))
        self.velocity_tau_sec = float(rospy.get_param("~velocity_tau_sec", 0.6))
        self.yaw_rate_tau_sec = float(rospy.get_param("~yaw_rate_tau_sec", 0.4))
        self.altitude_hold_m = float(rospy.get_param("~altitude_hold_m", 1.6))
        self.altitude_tau_sec = float(rospy.get_param("~altitude_tau_sec", 1.0))
        self.position_kp_xy = float(rospy.get_param("~position_kp_xy", 0.8))
        self.position_kp_z = float(rospy.get_param("~position_kp_z", 1.0))
        self.yaw_kp = float(rospy.get_param("~yaw_kp", 1.2))
        self.max_world_speed_mps = float(rospy.get_param("~max_world_speed_mps", 1.5))
        self.max_vertical_mps = float(rospy.get_param("~max_vertical_mps", 0.8))
        self.initial_x_m = float(rospy.get_param("~initial_x_m", 0.0))
        self.initial_y_m = float(rospy.get_param("~initial_y_m", 0.0))
        self.initial_z_m = float(rospy.get_param("~initial_z_m", self.altitude_hold_m))
        self.initial_yaw_deg = float(rospy.get_param("~initial_yaw_deg", 0.0))

        self.position_x = self.initial_x_m
        self.position_y = self.initial_y_m
        self.position_z = self.initial_z_m
        self.yaw_rad = math.radians(self.initial_yaw_deg)
        self.body_vx = 0.0
        self.body_vy = 0.0
        self.world_vx = 0.0
        self.world_vy = 0.0
        self.world_vz = 0.0
        self.yaw_rate_rps = 0.0
        self.last_command = None
        self.last_command_stamp = None
        self.last_update_stamp = rospy.Time.now()
        self.last_logged_mode = None

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        if self.input_mode == "follow_command":
            subscriber_type = FollowCommand
            callback = self._follow_command_callback
        elif self.input_mode == "position_command":
            subscriber_type = PositionCommand
            callback = self._position_command_callback
        else:
            raise RuntimeError("unsupported vehicle_kinematic_sim input_mode '%s'" % self.input_mode)
        self.command_sub = rospy.Subscriber(self.input_topic, subscriber_type, callback, queue_size=20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "vehicle_kinematic_sim ready: mode=%s input=%s odom=%s frame=%s->%s",
            self.input_mode,
            self.input_topic,
            self.odom_topic,
            self.world_frame_id,
            self.child_frame_id,
        )

    def _follow_command_callback(self, msg):
        self.last_command = msg
        self.last_command_stamp = rospy.Time.now()

    def _position_command_callback(self, msg):
        self.last_command = msg
        self.last_command_stamp = rospy.Time.now()

    def _command_is_fresh(self, now):
        if self.last_command is None or self.last_command_stamp is None:
            return False
        return (now - self.last_command_stamp).to_sec() <= self.command_timeout_sec

    def _first_order_update(self, current_value, target_value, dt_sec, tau_sec):
        tau = max(float(tau_sec), 1e-3)
        alpha = 1.0 - math.exp(-max(float(dt_sec), 0.0) / tau)
        return float(current_value) + alpha * (float(target_value) - float(current_value))

    def _target_motion_from_command(self, now):
        if self.input_mode == "position_command":
            return self._target_motion_from_position_command(now)
        if not self._command_is_fresh(now):
            return 0.0, 0.0, 0.0, "hold_timeout"

        msg = self.last_command
        if msg.valid and msg.mode == "body_velocity_yaw_rate":
            target_vx = clamp(msg.forward_mps * self.linear_velocity_scale, -self.max_forward_mps, self.max_forward_mps)
            target_vy = clamp(msg.lateral_mps * self.linear_velocity_scale, -self.max_lateral_mps, self.max_lateral_mps)
            target_yaw_rate = clamp(msg.yaw_rate_rps, -self.max_yaw_rate_rps, self.max_yaw_rate_rps)
            return target_vx, target_vy, target_yaw_rate, msg.mode

        if msg.valid and msg.mode == "search_yaw_only":
            target_yaw_rate = clamp(msg.yaw_rate_rps, -self.max_yaw_rate_rps, self.max_yaw_rate_rps)
            return 0.0, 0.0, target_yaw_rate, msg.mode

        return 0.0, 0.0, 0.0, msg.mode or "hold"

    def _target_motion_from_position_command(self, now):
        target_z = self.altitude_hold_m
        if not self._command_is_fresh(now):
            return 0.0, 0.0, 0.0, target_z, "hold_timeout"

        msg = self.last_command
        position_error_x = float(msg.position.x) - self.position_x
        position_error_y = float(msg.position.y) - self.position_y
        position_error_z = float(msg.position.z) - self.position_z

        target_world_vx = float(msg.velocity.x) + self.position_kp_xy * position_error_x
        target_world_vy = float(msg.velocity.y) + self.position_kp_xy * position_error_y
        target_world_vx = clamp(target_world_vx, -self.max_world_speed_mps, self.max_world_speed_mps)
        target_world_vy = clamp(target_world_vy, -self.max_world_speed_mps, self.max_world_speed_mps)

        desired_yaw = float(msg.yaw)
        target_yaw_rate = float(msg.yaw_dot) + self.yaw_kp * wrap_pi(desired_yaw - self.yaw_rad)
        target_yaw_rate = clamp(target_yaw_rate, -self.max_yaw_rate_rps, self.max_yaw_rate_rps)

        self.world_vz = clamp(
            float(msg.velocity.z) + self.position_kp_z * position_error_z,
            -self.max_vertical_mps,
            self.max_vertical_mps,
        )
        target_z = float(msg.position.z)
        return target_world_vx, target_world_vy, target_yaw_rate, target_z, "position_command"

    def _publish_tf_and_odom(self, stamp):
        if rospy.is_shutdown():
            return
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw_rad)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame_id
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = float(self.position_x)
        transform.transform.translation.y = float(self.position_y)
        transform.transform.translation.z = float(self.position_z)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        try:
            self.tf_broadcaster.sendTransform(transform)
        except rospy.ROSException:
            return

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(self.position_x)
        odom.pose.pose.position.y = float(self.position_y)
        odom.pose.pose.position.z = float(self.position_z)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self.body_vx)
        odom.twist.twist.linear.y = float(self.body_vy)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = float(self.yaw_rate_rps)
        try:
            self.odom_pub.publish(odom)
        except rospy.ROSException:
            return

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        dt_sec = max(0.0, (now - self.last_update_stamp).to_sec())
        self.last_update_stamp = now
        if dt_sec <= 0.0:
            self._publish_tf_and_odom(now)
            return

        target_z = self.altitude_hold_m
        if self.input_mode == "position_command":
            target_world_vx, target_world_vy, target_yaw_rate, target_z, mode = self._target_motion_from_command(now)
        else:
            target_vx, target_vy, target_yaw_rate, mode = self._target_motion_from_command(now)
        if mode != self.last_logged_mode:
            if self.input_mode == "position_command":
                rospy.loginfo(
                    "vehicle_kinematic_sim mode=%s target_world_vx=%.3f target_world_vy=%.3f target_yaw_rate=%.3f",
                    mode,
                    target_world_vx,
                    target_world_vy,
                    target_yaw_rate,
                )
            else:
                rospy.loginfo(
                    "vehicle_kinematic_sim mode=%s target_vx=%.3f target_vy=%.3f target_yaw_rate=%.3f",
                    mode,
                    target_vx,
                    target_vy,
                    target_yaw_rate,
                )
            self.last_logged_mode = mode

        self.yaw_rate_rps = self._first_order_update(self.yaw_rate_rps, target_yaw_rate, dt_sec, self.yaw_rate_tau_sec)

        if self.input_mode == "position_command":
            # PositionCommand is a world-frame tracking interface. Keep x/y tracking
            # in world coordinates so a yaw-only override does not bend translation.
            self.world_vx = self._first_order_update(self.world_vx, target_world_vx, dt_sec, self.velocity_tau_sec)
            self.world_vy = self._first_order_update(self.world_vy, target_world_vy, dt_sec, self.velocity_tau_sec)
        else:
            self.body_vx = self._first_order_update(self.body_vx, target_vx, dt_sec, self.velocity_tau_sec)
            self.body_vy = self._first_order_update(self.body_vy, target_vy, dt_sec, self.velocity_tau_sec)
            cos_yaw = math.cos(self.yaw_rad)
            sin_yaw = math.sin(self.yaw_rad)
            self.world_vx = cos_yaw * self.body_vx - sin_yaw * self.body_vy
            self.world_vy = sin_yaw * self.body_vx + cos_yaw * self.body_vy

        self.position_x += self.world_vx * dt_sec
        self.position_y += self.world_vy * dt_sec
        if self.input_mode == "position_command":
            self.position_z += self.world_vz * dt_sec
            self.position_z = self._first_order_update(self.position_z, target_z, dt_sec, self.altitude_tau_sec)
        else:
            self.position_z = self._first_order_update(self.position_z, self.altitude_hold_m, dt_sec, self.altitude_tau_sec)
        self.yaw_rad += self.yaw_rate_rps * dt_sec
        cos_yaw = math.cos(self.yaw_rad)
        sin_yaw = math.sin(self.yaw_rad)
        self.body_vx = cos_yaw * self.world_vx + sin_yaw * self.world_vy
        self.body_vy = -sin_yaw * self.world_vx + cos_yaw * self.world_vy
        self._publish_tf_and_odom(now)


if __name__ == "__main__":
    rospy.init_node("vehicle_kinematic_sim")
    VehicleKinematicSimNode()
    rospy.spin()
