#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import FollowCommand, FollowState, Target3D


STATE_NAME = {
    FollowState.IDLE: "idle",
    FollowState.TARGET_ACQUIRED: "target_acquired",
    FollowState.FOLLOW: "follow",
    FollowState.SEARCH: "search",
    FollowState.LOST: "lost",
}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def apply_deadband(value, deadband):
    if deadband <= 0.0:
        return float(value)
    if abs(value) <= deadband:
        return 0.0
    return float(value) - math.copysign(float(deadband), float(value))


def yaw_from_quaternion(x_value, y_value, z_value, w_value):
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


class PlanningAlgorithmTemplateNode:
    def __init__(self):
        self.target_world_topic = rospy.get_param("~target_world_topic", "/follow/fusion/target_world")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/follow/lidar/points")
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.state_topic = rospy.get_param("~state_topic", "/follow/state")
        self.planning_path_topic = rospy.get_param("~planning_path_topic", "/follow/planning/debug_path")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.target_fresh_timeout_sec = float(rospy.get_param("~target_fresh_timeout_sec", 0.25))
        self.acquire_state_hold_sec = float(rospy.get_param("~acquire_state_hold_sec", 0.5))
        self.search_timeout_sec = float(rospy.get_param("~search_timeout_sec", 2.0))
        self.lost_timeout_sec = float(rospy.get_param("~lost_timeout_sec", 5.0))
        self.search_yaw_rate_rps = float(rospy.get_param("~search_yaw_rate_rps", 0.25))
        self.search_sweep_span_deg = float(rospy.get_param("~search_sweep_span_deg", 0.0))
        self.search_sweep_pass_count = max(int(rospy.get_param("~search_sweep_pass_count", 0)), 0)
        self.search_return_to_center = bool(rospy.get_param("~search_return_to_center", False))
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.6))
        self.pointcloud_timeout_sec = float(rospy.get_param("~pointcloud_timeout_sec", 0.6))
        self.require_pointcloud = bool(rospy.get_param("~require_pointcloud", False))
        self.desired_distance_m = float(rospy.get_param("~desired_distance_m", 4.0))
        self.kp_forward = float(rospy.get_param("~kp_forward", 0.70))
        self.kp_lateral = float(rospy.get_param("~kp_lateral", 0.70))
        self.kp_yaw = float(rospy.get_param("~kp_yaw", 0.55))
        self.kd_forward = float(rospy.get_param("~kd_forward", 0.08))
        self.kd_lateral = float(rospy.get_param("~kd_lateral", 0.05))
        self.kd_yaw = float(rospy.get_param("~kd_yaw", 0.05))
        self.max_forward_mps = float(rospy.get_param("~max_forward_mps", 1.0))
        self.max_lateral_mps = float(rospy.get_param("~max_lateral_mps", 0.8))
        self.max_yaw_rate_rps = float(rospy.get_param("~max_yaw_rate_rps", 0.5))
        self.distance_deadband_m = float(rospy.get_param("~distance_deadband_m", 0.03))
        self.lateral_deadband_m = float(rospy.get_param("~lateral_deadband_m", 0.03))
        self.yaw_deadband_rad = float(rospy.get_param("~yaw_deadband_rad", 0.02))

        self.last_target_world = None
        self.last_target_stamp = None
        self.last_odom = None
        self.last_odom_stamp = None
        self.last_pointcloud_stamp = None
        self.last_seen_lateral_sign = 1.0
        self.acquire_stamp = None
        self.last_control_stamp = None
        self.last_logged_state = None
        self.last_forward_error = 0.0
        self.last_lateral_error = 0.0
        self.last_yaw_error = 0.0

        self.command_pub = rospy.Publisher(self.command_topic, FollowCommand, queue_size=10)
        self.state_pub = rospy.Publisher(self.state_topic, FollowState, queue_size=10)
        self.path_pub = rospy.Publisher(self.planning_path_topic, Path, queue_size=10)

        self.target_sub = rospy.Subscriber(self.target_world_topic, Target3D, self._target_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=10)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self._pointcloud_callback, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "planning_algorithm_template ready: target_world=%s odom=%s cloud=%s cmd=%s path=%s require_cloud=%s",
            self.target_world_topic,
            self.odom_topic,
            self.pointcloud_topic,
            self.command_topic,
            self.planning_path_topic,
            "true" if self.require_pointcloud else "false",
        )

    def _target_callback(self, msg):
        if not msg.valid:
            return
        now = rospy.Time.now()
        if self.last_target_stamp is None or (now - self.last_target_stamp).to_sec() > self.target_fresh_timeout_sec:
            self.acquire_stamp = now
        self.last_target_world = msg
        self.last_target_stamp = now
        relative_body = self._world_target_to_body(msg)
        if relative_body is not None and abs(relative_body[1]) > 1e-3:
            self.last_seen_lateral_sign = 1.0 if relative_body[1] >= 0.0 else -1.0

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_stamp = rospy.Time.now()

    def _pointcloud_callback(self, _msg):
        self.last_pointcloud_stamp = rospy.Time.now()

    def _odom_is_fresh(self, now):
        if self.last_odom is None or self.last_odom_stamp is None:
            return False
        return (now - self.last_odom_stamp).to_sec() <= self.odom_timeout_sec

    def _pointcloud_is_fresh(self, now):
        if not self.require_pointcloud:
            return True
        if self.last_pointcloud_stamp is None:
            return False
        return (now - self.last_pointcloud_stamp).to_sec() <= self.pointcloud_timeout_sec

    def _current_state(self, now):
        if not self._odom_is_fresh(now):
            return FollowState.IDLE, "odom unavailable"
        if not self._pointcloud_is_fresh(now):
            return FollowState.IDLE, "pointcloud unavailable"
        if self.last_target_world is None or self.last_target_stamp is None:
            return FollowState.IDLE, "no active target"

        age_sec = (now - self.last_target_stamp).to_sec()
        if age_sec <= self.target_fresh_timeout_sec:
            if self.acquire_stamp and (now - self.acquire_stamp).to_sec() <= self.acquire_state_hold_sec:
                return FollowState.TARGET_ACQUIRED, "fresh target_world"
            return FollowState.FOLLOW, "tracking target_world"

        lost_age_sec = max(0.0, age_sec - self.target_fresh_timeout_sec)
        if lost_age_sec <= self.search_timeout_sec:
            return FollowState.SEARCH, "target_world temporarily lost"
        if lost_age_sec <= self.search_timeout_sec + self.lost_timeout_sec:
            return FollowState.LOST, "target_world lost, waiting for reacquire"
        return FollowState.IDLE, "target_world stale"

    def _search_sweep_enabled(self):
        return (
            abs(self.search_yaw_rate_rps) > 1e-6
            and self.search_sweep_span_deg > 1e-6
            and self.search_sweep_pass_count > 0
        )

    def _search_segment_durations_sec(self):
        if not self._search_sweep_enabled():
            return []
        yaw_rate = abs(self.search_yaw_rate_rps)
        span_rad = math.radians(self.search_sweep_span_deg)
        half_sweep_sec = 0.5 * span_rad / yaw_rate
        full_sweep_sec = span_rad / yaw_rate
        durations = [half_sweep_sec]
        durations.extend(full_sweep_sec for _ in range(self.search_sweep_pass_count))
        if self.search_return_to_center:
            durations.append(half_sweep_sec)
        return durations

    def _search_yaw_rate_command(self, now):
        if self.last_target_stamp is None:
            return 0.0
        if not self._search_sweep_enabled():
            return self.search_yaw_rate_rps * self.last_seen_lateral_sign
        elapsed_sec = max(0.0, (now - self.last_target_stamp).to_sec() - self.target_fresh_timeout_sec)
        direction = 1.0 if self.last_seen_lateral_sign >= 0.0 else -1.0
        base_rate = abs(self.search_yaw_rate_rps)
        for duration_sec in self._search_segment_durations_sec():
            if elapsed_sec <= duration_sec:
                return base_rate * direction
            elapsed_sec -= duration_sec
            direction *= -1.0
        return 0.0

    def _current_pose_xyyaw(self):
        if self.last_odom is None:
            return None
        odom = self.last_odom
        return (
            float(odom.pose.pose.position.x),
            float(odom.pose.pose.position.y),
            yaw_from_quaternion(
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            ),
        )

    def _world_target_to_body(self, target_msg):
        pose = self._current_pose_xyyaw()
        if pose is None:
            return None
        vehicle_x, vehicle_y, yaw_rad = pose
        delta_x = float(target_msg.position.x) - vehicle_x
        delta_y = float(target_msg.position.y) - vehicle_y
        body_x = math.cos(yaw_rad) * delta_x + math.sin(yaw_rad) * delta_y
        body_y = -math.sin(yaw_rad) * delta_x + math.cos(yaw_rad) * delta_y
        body_z = float(target_msg.position.z) - float(self.last_odom.pose.pose.position.z)
        return body_x, body_y, body_z

    def _make_standoff_world(self, target_msg):
        pose = self._current_pose_xyyaw()
        if pose is None:
            return None
        vehicle_x, vehicle_y, _yaw_rad = pose
        target_x = float(target_msg.position.x)
        target_y = float(target_msg.position.y)
        delta_x = vehicle_x - target_x
        delta_y = vehicle_y - target_y
        distance = math.hypot(delta_x, delta_y)
        if distance < 1e-6:
            return vehicle_x, vehicle_y
        unit_x = delta_x / distance
        unit_y = delta_y / distance
        return (
            target_x + unit_x * self.desired_distance_m,
            target_y + unit_y * self.desired_distance_m,
        )

    def _world_point_to_body(self, world_x, world_y):
        pose = self._current_pose_xyyaw()
        if pose is None:
            return None
        vehicle_x, vehicle_y, yaw_rad = pose
        delta_x = float(world_x) - vehicle_x
        delta_y = float(world_y) - vehicle_y
        body_x = math.cos(yaw_rad) * delta_x + math.sin(yaw_rad) * delta_y
        body_y = -math.sin(yaw_rad) * delta_x + math.cos(yaw_rad) * delta_y
        return body_x, body_y

    def _pd_axis(self, error_value, last_error_value, dt_sec, kp, kd, limit_value):
        derivative_value = 0.0
        if dt_sec > 1e-4:
            derivative_value = (float(error_value) - float(last_error_value)) / float(dt_sec)
        raw_output = float(kp) * float(error_value) + float(kd) * derivative_value
        return clamp(raw_output, -float(limit_value), float(limit_value))

    def _make_hold_command(self, stamp):
        command = FollowCommand()
        command.header.stamp = stamp
        command.mode = "hold"
        command.valid = False
        return command

    def _publish_path(self, stamp, standoff_xy, target_msg):
        if self.last_odom is None or standoff_xy is None or target_msg is None:
            return
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.world_frame_id

        vehicle_pose = self.last_odom.pose.pose.position
        path.poses = []
        for point_xyz in (
            (float(vehicle_pose.x), float(vehicle_pose.y), float(vehicle_pose.z)),
            (float(standoff_xy[0]), float(standoff_xy[1]), float(vehicle_pose.z)),
            (float(target_msg.position.x), float(target_msg.position.y), float(target_msg.position.z)),
        ):
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.world_frame_id
            pose.pose.position.x = point_xyz[0]
            pose.pose.position.y = point_xyz[1]
            pose.pose.position.z = point_xyz[2]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        try:
            self.path_pub.publish(path)
        except rospy.ROSException:
            return

    def _tick(self, _event):
        if rospy.is_shutdown():
            return

        now = rospy.Time.now()
        dt_sec = 0.0
        if self.last_control_stamp is not None:
            dt_sec = max(0.0, (now - self.last_control_stamp).to_sec())
        self.last_control_stamp = now

        state, detail = self._current_state(now)
        if state != self.last_logged_state:
            rospy.loginfo("planning_algorithm_template state=%s detail=%s", STATE_NAME[state], detail)
            self.last_logged_state = state

        command = self._make_hold_command(now)

        if state in (FollowState.TARGET_ACQUIRED, FollowState.FOLLOW) and self.last_target_world is not None:
            relative_body = self._world_target_to_body(self.last_target_world)
            standoff_xy = self._make_standoff_world(self.last_target_world)
            standoff_body = None if standoff_xy is None else self._world_point_to_body(standoff_xy[0], standoff_xy[1])

            if relative_body is not None and standoff_body is not None:
                desired_yaw = math.atan2(relative_body[1], max(relative_body[0], 1e-6))
                forward_error = apply_deadband(standoff_body[0], self.distance_deadband_m)
                lateral_error = apply_deadband(standoff_body[1], self.lateral_deadband_m)
                yaw_error = apply_deadband(wrap_pi(desired_yaw), self.yaw_deadband_rad)

                command.forward_mps = self._pd_axis(
                    forward_error,
                    self.last_forward_error,
                    dt_sec,
                    self.kp_forward,
                    self.kd_forward,
                    self.max_forward_mps,
                )
                command.lateral_mps = self._pd_axis(
                    lateral_error,
                    self.last_lateral_error,
                    dt_sec,
                    self.kp_lateral,
                    self.kd_lateral,
                    self.max_lateral_mps,
                )
                command.yaw_rate_rps = self._pd_axis(
                    yaw_error,
                    self.last_yaw_error,
                    dt_sec,
                    self.kp_yaw,
                    self.kd_yaw,
                    self.max_yaw_rate_rps,
                )
                command.mode = "body_velocity_yaw_rate"
                command.valid = True
                self.last_forward_error = float(forward_error)
                self.last_lateral_error = float(lateral_error)
                self.last_yaw_error = float(yaw_error)
                self._publish_path(now, standoff_xy, self.last_target_world)
        elif state == FollowState.SEARCH:
            self.last_forward_error = 0.0
            self.last_lateral_error = 0.0
            self.last_yaw_error = 0.0
            command.forward_mps = 0.0
            command.lateral_mps = 0.0
            command.yaw_rate_rps = self._search_yaw_rate_command(now)
            command.mode = "search_yaw_only"
            command.valid = True
        else:
            self.last_forward_error = 0.0
            self.last_lateral_error = 0.0
            self.last_yaw_error = 0.0

        follow_state = FollowState()
        follow_state.header.stamp = now
        follow_state.state = state
        follow_state.state_name = STATE_NAME[state]
        follow_state.detail = detail

        try:
            self.command_pub.publish(command)
            self.state_pub.publish(follow_state)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("human_follow_controller")
    PlanningAlgorithmTemplateNode()
    rospy.spin()
