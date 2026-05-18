#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path as FsPath

import numpy as np
import rospy
from ego_planner.msg import Bspline
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import FollowState


ROOT = FsPath("/home/coco/follwer_ws")
FUSION_SCRIPT_DIR = ROOT / "src" / "human_follow_fusion" / "scripts"
if str(FUSION_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(FUSION_SCRIPT_DIR))

from projection_math import (  # noqa: E402
    load_camera_lidar_extrinsics_yaml,
    load_rigid_transform_yaml,
    transform_points,
)


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


def clamp_nonnegative(value):
    return max(float(value), 0.0)


class CmdOdomTrackingProbe:
    def __init__(self, extra_clearance_override_m=None):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.last_odom = None
        self.last_odom_recv_time = None
        self.last_cmd = None
        self.last_cmd_recv_time = None
        self.last_bspline = None
        self.last_bspline_recv_time = None
        self.last_stage2_goal = None
        self.last_stage2_goal_recv_time = None
        self.last_ego_goal = None
        self.last_ego_goal_recv_time = None
        self.last_waypoints = None
        self.last_waypoints_recv_time = None
        self.last_state = None
        self.last_occupancy_stamp = None
        self.last_obstacle_cloud_stamp = None
        self.last_foreground_cloud_stamp = None
        self.occupied_cells = set()
        self.static_obstacle_cells = set()
        self.foreground_cells = set()

        self.occupancy_resolution_m = float(rospy.get_param("/ego_planner_node/grid_map/resolution", 0.1))
        self.map_inflation_m = clamp_nonnegative(
            rospy.get_param("/ego_planner_node/grid_map/obstacles_inflation", 0.1)
        )
        self.goalgen_occupancy_timeout_sec = float(
            rospy.get_param("/stage2_follow_goal_generator/occupancy_timeout_sec", 0.6)
        )
        if extra_clearance_override_m is None:
            self.extra_clearance_m = 0.0
        else:
            self.extra_clearance_m = clamp_nonnegative(extra_clearance_override_m)

        self.cloud_extrinsics_yaml = str(rospy.get_param("/stage2_ego_topic_adapter/cloud_extrinsics_yaml", ""))
        self.cloud_body_yaml = str(rospy.get_param("/stage2_ego_topic_adapter/cloud_body_yaml", ""))
        self.world_frame_id = str(rospy.get_param("/stage2_ego_topic_adapter/world_frame_id", "map"))
        self._camera_lidar_extrinsics = None
        self._body_from_camera = None
        if self.cloud_extrinsics_yaml and self.cloud_body_yaml:
            try:
                self._camera_lidar_extrinsics = load_camera_lidar_extrinsics_yaml(self.cloud_extrinsics_yaml)
                self._body_from_camera = load_rigid_transform_yaml(self.cloud_body_yaml)["target_T_source"]
            except Exception as exc:
                rospy.logwarn("failed to load cloud transforms for obstacle-only diagnostics: %s", exc)
                self._camera_lidar_extrinsics = None
                self._body_from_camera = None

        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/ego_position_cmd", PositionCommand, self._cmd_cb, queue_size=50)
        rospy.Subscriber("/planning/bspline", Bspline, self._bspline_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/goal", PoseStamped, self._stage2_goal_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/ego_goal", PoseStamped, self._ego_goal_cb, queue_size=50)
        rospy.Subscriber("/waypoint_generator/waypoints", RosPath, self._waypoints_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/state", FollowState, self._state_cb, queue_size=50)
        rospy.Subscriber("/grid_map/occupancy_inflate", PointCloud2, self._occupancy_cb, queue_size=5)
        rospy.Subscriber("/follow/lidar/obstacle_points", PointCloud2, self._obstacle_cloud_cb, queue_size=5)
        rospy.Subscriber("/follow/lidar/foreground_points", PointCloud2, self._foreground_cloud_cb, queue_size=5)

    def _odom_cb(self, msg):
        self.last_odom = msg
        self.last_odom_recv_time = time.time()

    def _cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_recv_time = time.time()

    def _bspline_cb(self, msg):
        self.last_bspline = msg
        self.last_bspline_recv_time = time.time()

    def _stage2_goal_cb(self, msg):
        self.last_stage2_goal = msg
        self.last_stage2_goal_recv_time = time.time()

    def _ego_goal_cb(self, msg):
        self.last_ego_goal = msg
        self.last_ego_goal_recv_time = time.time()

    def _waypoints_cb(self, msg):
        self.last_waypoints = msg
        self.last_waypoints_recv_time = time.time()

    def _state_cb(self, msg):
        self.last_state = msg

    def _occupancy_cb(self, msg):
        occupied = set()
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            occupied.add(self._cell_key(point[0], point[1], point[2]))
        self.occupied_cells = occupied
        self.last_occupancy_stamp = time.time()

    def _cloud_to_world_points(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return np.zeros((0, 3), dtype=np.float64)

        raw_points = np.asarray(points, dtype=np.float64)
        frame_id = str(getattr(msg.header, "frame_id", ""))
        if frame_id in ("map", "world", self.world_frame_id):
            return raw_points

        if self.last_odom is None or self._camera_lidar_extrinsics is None or self._body_from_camera is None:
            return np.zeros((0, 3), dtype=np.float64)

        camera_points = transform_points(raw_points, self._camera_lidar_extrinsics["camera_T_lidar"])
        body_points = transform_points(camera_points, self._body_from_camera)

        pose = self.last_odom.pose.pose
        yaw_rad = yaw_from_quaternion(pose.orientation)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        world_points = np.zeros_like(body_points)
        world_points[:, 0] = float(pose.position.x) + cos_yaw * body_points[:, 0] - sin_yaw * body_points[:, 1]
        world_points[:, 1] = float(pose.position.y) + sin_yaw * body_points[:, 0] + cos_yaw * body_points[:, 1]
        world_points[:, 2] = float(pose.position.z) + body_points[:, 2]
        return world_points

    def _cells_from_cloud(self, msg):
        points_world = self._cloud_to_world_points(msg)
        return {
            self._cell_key(point[0], point[1], point[2])
            for point in points_world
            if np.all(np.isfinite(point))
        }

    def _obstacle_cloud_cb(self, msg):
        self.static_obstacle_cells = self._cells_from_cloud(msg)
        self.last_obstacle_cloud_stamp = time.time()

    def _foreground_cloud_cb(self, msg):
        self.foreground_cells = self._cells_from_cloud(msg)
        self.last_foreground_cloud_stamp = time.time()

    def _cell_key(self, x_value, y_value, z_value):
        resolution = max(self.occupancy_resolution_m, 0.02)
        return (
            int(round(float(x_value) / resolution)),
            int(round(float(y_value) / resolution)),
            int(round(float(z_value) / resolution)),
        )

    def _point_dict(self, x_value, y_value, z_value):
        return {
            "x": round(float(x_value), 3),
            "y": round(float(y_value), 3),
            "z": round(float(z_value), 3),
        }

    def _state_snapshot(self):
        if self.last_state is None:
            return None
        return {
            "state_name": str(self.last_state.state_name),
            "detail": str(self.last_state.detail),
        }

    def _occupancy_age_sec(self):
        if self.last_occupancy_stamp is None:
            return None
        return max(0.0, time.time() - float(self.last_occupancy_stamp))

    def _message_age_sec(self, recv_time):
        if recv_time is None:
            return None
        return max(0.0, time.time() - float(recv_time))

    def _decorate_event(self, payload, recv_time=None):
        event = dict(payload)
        occupancy_age_sec = self._occupancy_age_sec()
        message_age_sec = self._message_age_sec(recv_time)
        event["occupancy_age_sec"] = None if occupancy_age_sec is None else round(occupancy_age_sec, 3)
        event["occupancy_fresh_for_goalgen"] = (
            occupancy_age_sec is not None and occupancy_age_sec <= self.goalgen_occupancy_timeout_sec
        )
        event["message_age_sec"] = None if message_age_sec is None else round(message_age_sec, 3)
        detect_t_sec = event.get("t_sec")
        if message_age_sec is not None and isinstance(detect_t_sec, (int, float)):
            event["approx_publish_t_sec"] = round(max(0.0, float(detect_t_sec) - message_age_sec), 3)
        else:
            event["approx_publish_t_sec"] = None
        return event

    def wait_odom(self, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_odom is not None:
                return True
            time.sleep(0.05)
        return False

    def wait_occupancy_after(self, earliest_time, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_occupancy_stamp is not None and self.last_occupancy_stamp >= earliest_time and self.occupied_cells:
                return True
            time.sleep(0.05)
        return False

    def send_goal(self, x_value, y_value, z_value=0.0):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = x_value
        msg.pose.position.y = y_value
        msg.pose.position.z = z_value
        msg.pose.orientation.w = 1.0
        for _ in range(3):
            self.goal_pub.publish(msg)
            time.sleep(0.05)

    def _point_is_in_cells(self, cells, x_value, y_value, z_value, clearance_m=None):
        if not cells:
            return False
        if clearance_m is None:
            clearance_m = self.extra_clearance_m
        radius_cells = max(
            int(math.ceil(float(clearance_m) / max(self.occupancy_resolution_m, 0.02))),
            0,
        )
        center = self._cell_key(x_value, y_value, z_value)
        if radius_cells == 0:
            return center in cells
        for dx_value in range(-radius_cells, radius_cells + 1):
            for dy_value in range(-radius_cells, radius_cells + 1):
                for dz_value in range(-radius_cells, radius_cells + 1):
                    if dx_value * dx_value + dy_value * dy_value + dz_value * dz_value > radius_cells * radius_cells:
                        continue
                    if (
                        center[0] + dx_value,
                        center[1] + dy_value,
                        center[2] + dz_value,
                    ) in cells:
                        return True
        return False

    def point_is_in_inflated_obstacle(self, x_value, y_value, z_value):
        return self._point_is_in_cells(
            self.occupied_cells,
            x_value,
            y_value,
            z_value,
            clearance_m=self.extra_clearance_m,
        )

    def point_is_in_static_obstacle(self, x_value, y_value, z_value):
        return self._point_is_in_cells(
            self.static_obstacle_cells,
            x_value,
            y_value,
            z_value,
            clearance_m=self.map_inflation_m + self.extra_clearance_m,
        )

    def point_is_in_foreground(self, x_value, y_value, z_value):
        return self._point_is_in_cells(
            self.foreground_cells,
            x_value,
            y_value,
            z_value,
            clearance_m=self.map_inflation_m + self.extra_clearance_m,
        )

    def _cloud_age_sec(self, stamp_value):
        if stamp_value is None:
            return None
        return max(0.0, time.time() - float(stamp_value))

    def _nearest_cell_context(self, cells, x_value, y_value, z_value):
        if not cells:
            return None
        resolution = max(self.occupancy_resolution_m, 0.02)
        query = (float(x_value), float(y_value), float(z_value))
        best_key = None
        best_dist = None
        for key in cells:
            point = (key[0] * resolution, key[1] * resolution, key[2] * resolution)
            dist = math.sqrt(
                (point[0] - query[0]) ** 2
                + (point[1] - query[1]) ** 2
                + (point[2] - query[2]) ** 2
            )
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_key = key
        if best_key is None:
            return None
        nearest = (best_key[0] * resolution, best_key[1] * resolution, best_key[2] * resolution)
        return {
            "point": self._point_dict(nearest[0], nearest[1], nearest[2]),
            "distance_m": round(float(best_dist), 3),
        }

    def _hit_context(self, x_value, y_value, z_value):
        return {
            "all_ego_occupancy_hit": self.point_is_in_inflated_obstacle(x_value, y_value, z_value),
            "static_obstacle_hit": self.point_is_in_static_obstacle(x_value, y_value, z_value),
            "foreground_hit": self.point_is_in_foreground(x_value, y_value, z_value),
            "occupancy_age_sec": None if self._occupancy_age_sec() is None else round(self._occupancy_age_sec(), 3),
            "static_obstacle_cloud_age_sec": (
                None
                if self._cloud_age_sec(self.last_obstacle_cloud_stamp) is None
                else round(self._cloud_age_sec(self.last_obstacle_cloud_stamp), 3)
            ),
            "foreground_cloud_age_sec": (
                None
                if self._cloud_age_sec(self.last_foreground_cloud_stamp) is None
                else round(self._cloud_age_sec(self.last_foreground_cloud_stamp), 3)
            ),
            "static_obstacle_effective_clearance_m": round(float(self.map_inflation_m + self.extra_clearance_m), 3),
            "foreground_effective_clearance_m": round(float(self.map_inflation_m + self.extra_clearance_m), 3),
            "nearest_all_occupancy": self._nearest_cell_context(self.occupied_cells, x_value, y_value, z_value),
            "nearest_static_obstacle": self._nearest_cell_context(self.static_obstacle_cells, x_value, y_value, z_value),
            "nearest_foreground": self._nearest_cell_context(self.foreground_cells, x_value, y_value, z_value),
        }

    def _tracking_metrics(self):
        if self.last_cmd is None or self.last_odom is None:
            return None

        cmd_x = float(self.last_cmd.position.x)
        cmd_y = float(self.last_cmd.position.y)
        cmd_z = float(self.last_cmd.position.z)
        odom_x = float(self.last_odom.pose.pose.position.x)
        odom_y = float(self.last_odom.pose.pose.position.y)
        odom_z = float(self.last_odom.pose.pose.position.z)

        dx_value = odom_x - cmd_x
        dy_value = odom_y - cmd_y
        dz_value = odom_z - cmd_z

        dir_x = float(self.last_cmd.velocity.x)
        dir_y = float(self.last_cmd.velocity.y)
        dir_norm = math.hypot(dir_x, dir_y)
        if dir_norm < 1e-3:
            dir_x = cmd_x - odom_x
            dir_y = cmd_y - odom_y
            dir_norm = math.hypot(dir_x, dir_y)
        if dir_norm < 1e-3:
            cmd_yaw = float(self.last_cmd.yaw)
            dir_x = math.cos(cmd_yaw)
            dir_y = math.sin(cmd_yaw)
            dir_norm = 1.0
        dir_x /= dir_norm
        dir_y /= dir_norm
        left_x = -dir_y
        left_y = dir_x

        return {
            "cmd_point": self._point_dict(cmd_x, cmd_y, cmd_z),
            "odom_point": self._point_dict(odom_x, odom_y, odom_z),
            "cmd_speed_xy_mps": round(math.hypot(float(self.last_cmd.velocity.x), float(self.last_cmd.velocity.y)), 3),
            "cmd_speed_z_mps": round(float(self.last_cmd.velocity.z), 3),
            "tracking_error_xy_m": round(math.hypot(dx_value, dy_value), 3),
            "tracking_error_z_m": round(abs(dz_value), 3),
            "tracking_error_along_cmd_m": round(dx_value * dir_x + dy_value * dir_y, 3),
            "tracking_error_lateral_cmd_m": round(dx_value * left_x + dy_value * left_y, 3),
            "state": self._state_snapshot(),
            "trajectory_id": int(getattr(self.last_cmd, "trajectory_id", 0)),
        }

    def _evaluate_bspline_t(self, bspline_msg, t_sec):
        order = int(bspline_msg.order)
        knots = [float(value) for value in bspline_msg.knots]
        control_points = [
            [float(point.x), float(point.y), float(point.z)]
            for point in bspline_msg.pos_pts
        ]
        if not control_points or len(knots) < (len(control_points) + order + 1):
            return None

        m_value = len(knots) - 1
        lower_u = knots[order]
        upper_u = knots[m_value - order]
        u_value = min(max(lower_u, float(t_sec) + lower_u), upper_u)

        k_index = order
        while k_index + 1 < len(knots) and knots[k_index + 1] < u_value:
            k_index += 1

        work = [control_points[k_index - order + i][:] for i in range(order + 1)]
        for r_value in range(1, order + 1):
            for i_value in range(order, r_value - 1, -1):
                left = knots[i_value + k_index - order]
                right = knots[i_value + 1 + k_index - r_value]
                denom = right - left
                alpha = 0.0 if abs(denom) <= 1e-9 else (u_value - left) / denom
                prev_point = work[i_value - 1]
                curr_point = work[i_value]
                work[i_value] = [
                    (1.0 - alpha) * prev_point[axis] + alpha * curr_point[axis]
                    for axis in range(3)
                ]
        return work[order]

    def _bspline_curve_occupancy_summary(self, bspline_msg):
        if bspline_msg is None or not bspline_msg.pos_pts or not bspline_msg.knots:
            return None
        order = int(bspline_msg.order)
        knots = [float(value) for value in bspline_msg.knots]
        if len(knots) <= order * 2:
            return None
        duration_sec = float(knots[-order - 1] - knots[order])
        if duration_sec <= 0.0:
            return None

        planner_check_limit_sec = max(duration_sec * (2.0 / 3.0), 0.0)
        start_point = self._evaluate_bspline_t(bspline_msg, 0.0)
        end_point = self._evaluate_bspline_t(bspline_msg, duration_sec)
        if start_point is None or end_point is None:
            return None
        end_to_end_distance_m = math.sqrt(
            (float(end_point[0]) - float(start_point[0])) ** 2
            + (float(end_point[1]) - float(start_point[1])) ** 2
            + (float(end_point[2]) - float(start_point[2])) ** 2
        )
        resolution_m = max(self.occupancy_resolution_m, 0.02)
        planner_like_step_sec = duration_sec
        if end_to_end_distance_m > 1e-6:
            planner_like_step_sec = duration_sec / max(end_to_end_distance_m / resolution_m, 1.0)
        planner_like_step_sec = max(float(planner_like_step_sec), 1e-3)

        sample_step_sec = 0.02
        sample_count = max(int(math.ceil(duration_sec / sample_step_sec)), 1)
        first_curve_hit = None
        for sample_index in range(sample_count + 1):
            t_sec = min(float(sample_index) * sample_step_sec, duration_sec)
            point_xyz = self._evaluate_bspline_t(bspline_msg, t_sec)
            if point_xyz is None:
                continue
            if self.point_is_in_inflated_obstacle(point_xyz[0], point_xyz[1], point_xyz[2]):
                first_curve_hit = {
                    "point": self._point_dict(point_xyz[0], point_xyz[1], point_xyz[2]),
                    "curve_local_t_sec": round(float(t_sec), 3),
                    "beyond_planner_two_thirds": bool(float(t_sec) > planner_check_limit_sec + 1e-9),
                }
                break

        planner_like_hit = None
        planner_t_sec = 0.0
        while planner_t_sec < planner_check_limit_sec:
            point_xyz = self._evaluate_bspline_t(bspline_msg, planner_t_sec)
            if point_xyz is not None and self.point_is_in_inflated_obstacle(point_xyz[0], point_xyz[1], point_xyz[2]):
                planner_like_hit = {
                    "point": self._point_dict(point_xyz[0], point_xyz[1], point_xyz[2]),
                    "curve_local_t_sec": round(float(planner_t_sec), 3),
                }
                break
            planner_t_sec += planner_like_step_sec

        return {
            "duration_sec": round(float(duration_sec), 3),
            "planner_like_step_sec": round(float(planner_like_step_sec), 3),
            "planner_like_check_limit_sec": round(float(planner_check_limit_sec), 3),
            "first_curve_hit": first_curve_hit,
            "planner_like_hit": planner_like_hit,
        }

    def _first_point_in_occ_from_pose(self, pose_msg):
        if pose_msg is None:
            return None
        point = pose_msg.pose.position
        if self.point_is_in_inflated_obstacle(point.x, point.y, point.z):
            return self._point_dict(point.x, point.y, point.z)
        return None

    def _first_point_in_occ_from_path(self, path_msg):
        if path_msg is None:
            return None
        for pose_stamped in path_msg.poses:
            point = pose_stamped.pose.position
            if self.point_is_in_inflated_obstacle(point.x, point.y, point.z):
                return self._point_dict(point.x, point.y, point.z)
        return None

    def run(self, goal_forward_m=10.5, dwell_sec=14.0, prewait_occupancy_sec=0.0):
        if not self.wait_odom():
            return {"status": "failed", "reason": "odom_timeout"}

        odom = self.last_odom
        yaw_rad = yaw_from_quaternion(odom.pose.pose.orientation)
        start_x = float(odom.pose.pose.position.x)
        start_y = float(odom.pose.pose.position.y)
        start_z = float(odom.pose.pose.position.z)
        goal_xy = (
            start_x + math.cos(yaw_rad) * float(goal_forward_m),
            start_y + math.sin(yaw_rad) * float(goal_forward_m),
        )

        had_occupancy_before_goal = self.last_occupancy_stamp is not None and bool(self.occupied_cells)
        if prewait_occupancy_sec > 0.0:
            if not self.wait_occupancy_after(0.0):
                return {
                    "status": "failed",
                    "reason": "occupancy_timeout_before_goal",
                    "had_occupancy_before_goal": bool(had_occupancy_before_goal),
                }
            time.sleep(float(prewait_occupancy_sec))
            had_occupancy_before_goal = self.last_occupancy_stamp is not None and bool(self.occupied_cells)

        goal_send_time = time.time()
        self.send_goal(*goal_xy)
        if not self.wait_occupancy_after(goal_send_time):
            return {
                "status": "failed",
                "reason": "occupancy_timeout_after_goal",
                "had_occupancy_before_goal": bool(had_occupancy_before_goal),
            }
        occupancy_ready_stamp = float(self.last_occupancy_stamp)

        max_tracking_error_xy_m = 0.0
        max_abs_tracking_error_along_m = 0.0
        max_abs_tracking_error_lateral_m = 0.0
        max_cmd_speed_xy_mps = 0.0
        first_stage2_goal_in_occ = None
        first_ego_goal_in_occ = None
        first_waypoint_in_occ = None
        first_bspline_curve_in_occ = None
        first_cmd_in_occ = None
        first_odom_in_occ = None
        worst_tracking_sample = None

        deadline = time.time() + float(dwell_sec)
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)
            if self.last_odom is None or self.last_cmd is None:
                continue

            now_rel_sec = time.time() - goal_send_time
            metrics = self._tracking_metrics()
            if metrics is None:
                continue

            max_tracking_error_xy_m = max(max_tracking_error_xy_m, float(metrics["tracking_error_xy_m"]))
            max_abs_tracking_error_along_m = max(
                max_abs_tracking_error_along_m,
                abs(float(metrics["tracking_error_along_cmd_m"])),
            )
            max_abs_tracking_error_lateral_m = max(
                max_abs_tracking_error_lateral_m,
                abs(float(metrics["tracking_error_lateral_cmd_m"])),
            )
            max_cmd_speed_xy_mps = max(max_cmd_speed_xy_mps, float(metrics["cmd_speed_xy_mps"]))
            if worst_tracking_sample is None or float(metrics["tracking_error_xy_m"]) >= float(worst_tracking_sample["tracking_error_xy_m"]):
                worst_tracking_sample = dict(metrics)
                worst_tracking_sample["t_sec"] = round(now_rel_sec, 3)

            if (
                first_stage2_goal_in_occ is None
                and self.last_stage2_goal is not None
                and self.last_stage2_goal_recv_time is not None
                and self.last_stage2_goal_recv_time >= goal_send_time
            ):
                point_dict = self._first_point_in_occ_from_pose(self.last_stage2_goal)
                if point_dict is not None:
                    first_stage2_goal_in_occ = self._decorate_event(
                        {
                            "t_sec": round(now_rel_sec, 3),
                            "point": point_dict,
                            "state": self._state_snapshot(),
                        },
                        recv_time=self.last_stage2_goal_recv_time,
                    )

            if (
                first_ego_goal_in_occ is None
                and self.last_ego_goal is not None
                and self.last_ego_goal_recv_time is not None
                and self.last_ego_goal_recv_time >= goal_send_time
            ):
                point_dict = self._first_point_in_occ_from_pose(self.last_ego_goal)
                if point_dict is not None:
                    first_ego_goal_in_occ = self._decorate_event(
                        {
                            "t_sec": round(now_rel_sec, 3),
                            "point": point_dict,
                            "state": self._state_snapshot(),
                        },
                        recv_time=self.last_ego_goal_recv_time,
                    )

            if (
                first_waypoint_in_occ is None
                and self.last_waypoints is not None
                and self.last_waypoints_recv_time is not None
                and self.last_waypoints_recv_time >= goal_send_time
            ):
                point_dict = self._first_point_in_occ_from_path(self.last_waypoints)
                if point_dict is not None:
                    first_waypoint_in_occ = self._decorate_event(
                        {
                            "t_sec": round(now_rel_sec, 3),
                            "point": point_dict,
                            "state": self._state_snapshot(),
                        },
                        recv_time=self.last_waypoints_recv_time,
                    )

            if (
                first_bspline_curve_in_occ is None
                and self.last_bspline is not None
                and self.last_bspline_recv_time is not None
                and self.last_bspline_recv_time >= goal_send_time
            ):
                bspline_summary = self._bspline_curve_occupancy_summary(self.last_bspline)
                if bspline_summary is not None and bspline_summary["first_curve_hit"] is not None:
                    first_bspline_curve_in_occ = self._decorate_event(
                        {
                            "t_sec": round(now_rel_sec, 3),
                            "point": bspline_summary["first_curve_hit"]["point"],
                            "state": self._state_snapshot(),
                            "bspline_traj_id": int(getattr(self.last_bspline, "traj_id", 0)),
                            "cmd_trajectory_id_at_detect": (
                                None if self.last_cmd is None else int(getattr(self.last_cmd, "trajectory_id", 0))
                            ),
                            "curve_local_t_sec": bspline_summary["first_curve_hit"]["curve_local_t_sec"],
                            "beyond_planner_two_thirds": bspline_summary["first_curve_hit"]["beyond_planner_two_thirds"],
                            "planner_like_step_sec": bspline_summary["planner_like_step_sec"],
                            "planner_like_check_limit_sec": bspline_summary["planner_like_check_limit_sec"],
                            "planner_like_hit_curve_local_t_sec": (
                                None
                                if bspline_summary["planner_like_hit"] is None
                                else bspline_summary["planner_like_hit"]["curve_local_t_sec"]
                            ),
                            "planner_like_would_detect": bool(bspline_summary["planner_like_hit"] is not None),
                        },
                        recv_time=self.last_bspline_recv_time,
                    )

            cmd_in_occ = self.point_is_in_inflated_obstacle(
                self.last_cmd.position.x,
                self.last_cmd.position.y,
                self.last_cmd.position.z,
            )
            odom_in_occ = self.point_is_in_inflated_obstacle(
                self.last_odom.pose.pose.position.x,
                self.last_odom.pose.pose.position.y,
                self.last_odom.pose.pose.position.z,
            )

            if cmd_in_occ and first_cmd_in_occ is None:
                cmd_payload = dict(metrics)
                cmd_payload["t_sec"] = round(now_rel_sec, 3)
                cmd_payload["hit_context"] = self._hit_context(
                    self.last_cmd.position.x,
                    self.last_cmd.position.y,
                    self.last_cmd.position.z,
                )
                first_cmd_in_occ = self._decorate_event(cmd_payload, recv_time=self.last_cmd_recv_time)
            if odom_in_occ and first_odom_in_occ is None:
                odom_payload = dict(metrics)
                odom_payload["t_sec"] = round(now_rel_sec, 3)
                odom_payload["hit_context"] = self._hit_context(
                    self.last_odom.pose.pose.position.x,
                    self.last_odom.pose.pose.position.y,
                    self.last_odom.pose.pose.position.z,
                )
                first_odom_in_occ = self._decorate_event(odom_payload, recv_time=self.last_odom_recv_time)

        first_occurrences = {
            "stage2_goal": first_stage2_goal_in_occ,
            "ego_goal": first_ego_goal_in_occ,
            "waypoint": first_waypoint_in_occ,
            "bspline_curve": first_bspline_curve_in_occ,
            "position_cmd": first_cmd_in_occ,
            "odom": first_odom_in_occ,
        }
        earliest_layer = None
        earliest_time = None
        for layer_name, payload in first_occurrences.items():
            if payload is None:
                continue
            ordering_time = payload.get("approx_publish_t_sec")
            if ordering_time is None:
                ordering_time = payload["t_sec"]
            if earliest_time is None or ordering_time < earliest_time:
                earliest_time = ordering_time
                earliest_layer = layer_name

        odom_first_while_cmd_clear = (
            first_odom_in_occ is not None
            and (first_cmd_in_occ is None or float(first_odom_in_occ["t_sec"]) < float(first_cmd_in_occ["t_sec"]))
        )

        return {
            "status": "ok",
            "goal_xy": [round(goal_xy[0], 3), round(goal_xy[1], 3)],
            "start_xyz": [round(start_x, 3), round(start_y, 3), round(start_z, 3)],
            "had_occupancy_before_goal": bool(had_occupancy_before_goal),
            "occupancy_ready_delay_sec": round(max(0.0, occupancy_ready_stamp - goal_send_time), 3),
            "map_inflation_m": round(float(self.map_inflation_m), 3),
            "extra_clearance_m": round(float(self.extra_clearance_m), 3),
            "effective_total_clearance_m": round(float(self.map_inflation_m + self.extra_clearance_m), 3),
            "static_obstacle_cell_count": len(self.static_obstacle_cells),
            "foreground_cell_count": len(self.foreground_cells),
            "all_occupancy_cell_count": len(self.occupied_cells),
            "static_obstacle_cloud_age_sec": (
                None
                if self._cloud_age_sec(self.last_obstacle_cloud_stamp) is None
                else round(self._cloud_age_sec(self.last_obstacle_cloud_stamp), 3)
            ),
            "foreground_cloud_age_sec": (
                None
                if self._cloud_age_sec(self.last_foreground_cloud_stamp) is None
                else round(self._cloud_age_sec(self.last_foreground_cloud_stamp), 3)
            ),
            "max_tracking_error_xy_m": round(max_tracking_error_xy_m, 3),
            "max_abs_tracking_error_along_m": round(max_abs_tracking_error_along_m, 3),
            "max_abs_tracking_error_lateral_m": round(max_abs_tracking_error_lateral_m, 3),
            "max_cmd_speed_xy_mps": round(max_cmd_speed_xy_mps, 3),
            "earliest_failing_layer": earliest_layer,
            "first_stage2_goal_in_occ": first_stage2_goal_in_occ,
            "first_ego_goal_in_occ": first_ego_goal_in_occ,
            "first_waypoint_in_occ": first_waypoint_in_occ,
            "first_bspline_curve_in_occ": first_bspline_curve_in_occ,
            "first_cmd_in_occ": first_cmd_in_occ,
            "first_odom_in_occ": first_odom_in_occ,
            "odom_first_while_cmd_clear": bool(odom_first_while_cmd_clear),
            "worst_tracking_sample": worst_tracking_sample,
            "final_state": self._state_snapshot(),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-forward-m", type=float, default=10.5)
    parser.add_argument("--dwell-sec", type=float, default=14.0)
    parser.add_argument("--prewait-occupancy-sec", type=float, default=0.0)
    parser.add_argument("--extra-clearance-m", type=float, default=0.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    rospy.init_node("stage2_cmd_odom_tracking_probe", anonymous=True)
    probe = CmdOdomTrackingProbe(extra_clearance_override_m=args.extra_clearance_m)
    result = probe.run(
        goal_forward_m=args.goal_forward_m,
        dwell_sec=args.dwell_sec,
        prewait_occupancy_sec=args.prewait_occupancy_sec,
    )
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(result)


if __name__ == "__main__":
    main()
