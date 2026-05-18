#!/usr/bin/env python3
import argparse
import json
import math
import threading
import time
from collections import deque

import rospy
from ego_planner.msg import Bspline
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import FollowState


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


class CenterObstacleTraceProbe:
    def __init__(self, clearance_override_m=None):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.goal_send_time = None
        self.armed_external_goal_after_time = None
        self.external_goal_point = None
        self.last_odom = None
        self.last_odom_recv_time = None
        self.last_state = None
        self.last_state_recv_time = None
        self.last_stage2_goal = None
        self.last_stage2_goal_recv_time = None
        self.last_ego_goal = None
        self.last_ego_goal_recv_time = None
        self.last_waypoints = None
        self.last_waypoints_recv_time = None
        self.last_bspline = None
        self.last_bspline_recv_time = None
        self.last_position_cmd = None
        self.last_position_cmd_recv_time = None
        self.last_grid_cloud = None
        self.last_grid_cloud_recv_time = None
        self.last_occupancy_stamp = None
        self.occupied_cells = set()
        self.occupancy_history = deque()
        self.occupancy_history_sec = 3.0
        self.occupancy_history_max_frames = 80
        self._occupancy_lock = threading.Lock()

        self.occupancy_resolution_m = float(rospy.get_param("/ego_planner_node/grid_map/resolution", 0.1))
        self.map_inflation_m = max(
            float(rospy.get_param("/ego_planner_node/grid_map/obstacles_inflation", 0.1)),
            0.0,
        )
        default_extra_clearance_m = 0.05
        if clearance_override_m is None:
            self.extra_clearance_m = default_extra_clearance_m
        else:
            self.extra_clearance_m = max(float(clearance_override_m), 0.0)

        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/state", FollowState, self._state_cb, queue_size=50)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._nav_goal_cb, queue_size=20)
        rospy.Subscriber("/follow/stage2/goal", PoseStamped, self._stage2_goal_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/ego_goal", PoseStamped, self._ego_goal_cb, queue_size=50)
        rospy.Subscriber("/waypoint_generator/waypoints", Path, self._waypoints_cb, queue_size=50)
        rospy.Subscriber("/planning/bspline", Bspline, self._bspline_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/ego_position_cmd", PositionCommand, self._position_cmd_cb, queue_size=50)
        rospy.Subscriber("/grid_map/cloud", PointCloud2, self._grid_cloud_cb, queue_size=5)
        rospy.Subscriber("/grid_map/occupancy_inflate", PointCloud2, self._occupancy_cb, queue_size=5)

    def _odom_cb(self, msg):
        self.last_odom = msg
        self.last_odom_recv_time = time.time()

    def _state_cb(self, msg):
        self.last_state = msg
        self.last_state_recv_time = time.time()

    def _nav_goal_cb(self, msg):
        recv_time = time.time()
        if self.armed_external_goal_after_time is None:
            return
        if recv_time < self.armed_external_goal_after_time:
            return
        if self.goal_send_time is not None:
            return
        self.goal_send_time = recv_time
        self.external_goal_point = {
            "frame_id": str(msg.header.frame_id),
            "point": self._point_dict(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
        }

    def _stage2_goal_cb(self, msg):
        self.last_stage2_goal = msg
        self.last_stage2_goal_recv_time = time.time()

    def _ego_goal_cb(self, msg):
        self.last_ego_goal = msg
        self.last_ego_goal_recv_time = time.time()

    def _waypoints_cb(self, msg):
        self.last_waypoints = msg
        self.last_waypoints_recv_time = time.time()

    def _bspline_cb(self, msg):
        self.last_bspline = msg
        self.last_bspline_recv_time = time.time()

    def _position_cmd_cb(self, msg):
        self.last_position_cmd = msg
        self.last_position_cmd_recv_time = time.time()

    def _grid_cloud_cb(self, msg):
        self.last_grid_cloud = msg
        self.last_grid_cloud_recv_time = time.time()

    def _occupancy_cb(self, msg):
        recv_time = time.time()
        occupied = set()
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            occupied.add(self._cell_key(point[0], point[1], point[2]))
        with self._occupancy_lock:
            self.occupied_cells = occupied
            self.last_occupancy_stamp = recv_time
            self.occupancy_history.append(
                {
                    "recv_time": recv_time,
                    "occupied_cells": occupied,
                }
            )
            self._prune_occupancy_history(recv_time)

    def _prune_occupancy_history(self, now_wall):
        cutoff = float(now_wall) - float(self.occupancy_history_sec)
        while self.occupancy_history and (
            len(self.occupancy_history) > self.occupancy_history_max_frames
            or self.occupancy_history[0]["recv_time"] < cutoff
        ):
            self.occupancy_history.popleft()

    def _occupancy_history_snapshot(self):
        with self._occupancy_lock:
            return list(self.occupancy_history)

    def _occupied_cells_snapshot(self):
        with self._occupancy_lock:
            return set(self.occupied_cells)

    def _occupancy_stamp_and_nonempty(self):
        with self._occupancy_lock:
            return self.last_occupancy_stamp, bool(self.occupied_cells)

    def _latest_occupancy_cells_before(self, wall_time_sec):
        target_time = float(wall_time_sec)
        for entry in reversed(self._occupancy_history_snapshot()):
            if float(entry["recv_time"]) <= target_time + 1e-9:
                return entry
        return None

    def _first_occupancy_cells_at_or_after(self, wall_time_sec):
        target_time = float(wall_time_sec)
        for entry in self._occupancy_history_snapshot():
            if float(entry["recv_time"]) >= target_time - 1e-9:
                return entry
        return None

    def _first_detecting_occupancy_entry_for_point(self, x_value, y_value, z_value, recv_time):
        target_time = float(recv_time)
        for entry in self._occupancy_history_snapshot():
            if float(entry["recv_time"]) < target_time - 1e-9:
                continue
            if self._point_is_in_inflated_obstacle_from_cells(
                x_value,
                y_value,
                z_value,
                entry["occupied_cells"],
            ):
                return entry
        return None

    def _point_timeline_against_occupancy(self, x_value, y_value, z_value, recv_time):
        before_entry = self._latest_occupancy_cells_before(recv_time)
        after_entry = self._first_occupancy_cells_at_or_after(recv_time)
        first_detecting_entry = self._first_detecting_occupancy_entry_for_point(
            x_value,
            y_value,
            z_value,
            recv_time,
        )
        return {
            "occupancy_before_msg_recv_age_sec": (
                None
                if before_entry is None
                else round(max(0.0, float(recv_time) - float(before_entry["recv_time"])), 3)
            ),
            "occupancy_before_msg_recv_would_detect": bool(
                before_entry is not None
                and self._point_is_in_inflated_obstacle_from_cells(
                    x_value,
                    y_value,
                    z_value,
                    before_entry["occupied_cells"],
                )
            ),
            "occupancy_at_or_after_msg_recv_age_sec": (
                None
                if after_entry is None
                else round(max(0.0, float(after_entry["recv_time"]) - float(recv_time)), 3)
            ),
            "occupancy_at_or_after_msg_recv_would_detect": bool(
                after_entry is not None
                and self._point_is_in_inflated_obstacle_from_cells(
                    x_value,
                    y_value,
                    z_value,
                    after_entry["occupied_cells"],
                )
            ),
            "occupancy_first_detect_after_msg_recv_age_sec": (
                None
                if first_detecting_entry is None
                else round(max(0.0, float(first_detecting_entry["recv_time"]) - float(recv_time)), 3)
            ),
        }

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

    def _nearest_grid_cloud_summary(self, point_xyz):
        if self.last_grid_cloud is None:
            return None
        nearest_distance = None
        nearest_point = None
        within_015 = 0
        within_030 = 0
        within_050 = 0
        for x_value, y_value, z_value in pc2.read_points(
            self.last_grid_cloud,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            dx_value = float(x_value) - float(point_xyz[0])
            dy_value = float(y_value) - float(point_xyz[1])
            dz_value = float(z_value) - float(point_xyz[2])
            dist_value = math.sqrt(dx_value * dx_value + dy_value * dy_value + dz_value * dz_value)
            if nearest_distance is None or dist_value < nearest_distance:
                nearest_distance = dist_value
                nearest_point = {
                    "x": round(float(x_value), 3),
                    "y": round(float(y_value), 3),
                    "z": round(float(z_value), 3),
                }
            if dist_value <= 0.15:
                within_015 += 1
            if dist_value <= 0.30:
                within_030 += 1
            if dist_value <= 0.50:
                within_050 += 1
        if nearest_distance is None:
            return None
        return {
            "nearest_distance_m": round(float(nearest_distance), 3),
            "nearest_point": nearest_point,
            "within_0_15m": int(within_015),
            "within_0_30m": int(within_030),
            "within_0_50m": int(within_050),
        }

    def _recv_after_goal(self, recv_time, goal_send_time):
        return recv_time is not None and recv_time >= goal_send_time

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
            occupancy_stamp, has_occupied = self._occupancy_stamp_and_nonempty()
            if occupancy_stamp is not None and occupancy_stamp >= earliest_time and has_occupied:
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

    def wait_external_goal(self, timeout_sec=8.0):
        self.goal_send_time = None
        self.external_goal_point = None
        self.armed_external_goal_after_time = time.time()
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.goal_send_time is not None:
                self.armed_external_goal_after_time = None
                return True
            time.sleep(0.05)
        self.armed_external_goal_after_time = None
        return False

    def _point_is_in_inflated_obstacle_from_cells(self, x_value, y_value, z_value, occupied_cells):
        if not occupied_cells:
            return False
        radius_cells = max(
            int(math.ceil(self.extra_clearance_m / max(self.occupancy_resolution_m, 0.02))),
            0,
        )
        center = self._cell_key(x_value, y_value, z_value)
        if radius_cells == 0:
            return center in occupied_cells
        for dx_value in range(-radius_cells, radius_cells + 1):
            for dy_value in range(-radius_cells, radius_cells + 1):
                for dz_value in range(-radius_cells, radius_cells + 1):
                    if dx_value * dx_value + dy_value * dy_value + dz_value * dz_value > radius_cells * radius_cells:
                        continue
                    if (
                        center[0] + dx_value,
                        center[1] + dy_value,
                        center[2] + dz_value,
                    ) in occupied_cells:
                        return True
        return False

    def point_is_in_inflated_obstacle(self, x_value, y_value, z_value):
        return self._point_is_in_inflated_obstacle_from_cells(
            x_value,
            y_value,
            z_value,
            self.occupied_cells,
        )

    def _first_point_in_occ_from_path(self, path_msg):
        if path_msg is None:
            return None
        for pose_stamped in path_msg.poses:
            point = pose_stamped.pose.position
            if self.point_is_in_inflated_obstacle(point.x, point.y, point.z):
                return self._point_dict(point.x, point.y, point.z)
        return None

    def _first_point_in_occ_from_bspline(self, bspline_msg):
        if bspline_msg is None:
            return None
        for point in bspline_msg.pos_pts:
            if self.point_is_in_inflated_obstacle(point.x, point.y, point.z):
                return self._point_dict(point.x, point.y, point.z)
        return None

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

    def _compute_checked_curve_length(self, bspline_msg, checked_duration_sec):
        if checked_duration_sec <= 0.0:
            return 0.0
        sample_dt = max(min(float(checked_duration_sec) / 200.0, 0.01), 1e-3)
        prev_point = self._evaluate_bspline_t(bspline_msg, 0.0)
        if prev_point is None:
            return 0.0
        length_m = 0.0
        t_sec = sample_dt
        while t_sec <= checked_duration_sec + 1e-6:
            current_point = self._evaluate_bspline_t(bspline_msg, min(t_sec, checked_duration_sec))
            if current_point is None:
                break
            length_m += math.sqrt(
                (float(current_point[0]) - float(prev_point[0])) ** 2
                + (float(current_point[1]) - float(prev_point[1])) ** 2
                + (float(current_point[2]) - float(prev_point[2])) ** 2
            )
            prev_point = current_point
            t_sec += sample_dt
        return length_m

    def _sample_bspline_hit_with_step(self, bspline_msg, checked_duration_sec, step_sec, occupied_cells=None):
        occupied = self._occupied_cells_snapshot() if occupied_cells is None else occupied_cells
        t_sec = 0.0
        last_point = None
        max_gap_m = 0.0
        hit_payload = None
        while t_sec <= checked_duration_sec + 1e-6:
            point_xyz = self._evaluate_bspline_t(bspline_msg, min(t_sec, checked_duration_sec))
            if point_xyz is None:
                break
            if last_point is not None:
                gap_m = math.sqrt(
                    (float(point_xyz[0]) - float(last_point[0])) ** 2
                    + (float(point_xyz[1]) - float(last_point[1])) ** 2
                    + (float(point_xyz[2]) - float(last_point[2])) ** 2
                )
                max_gap_m = max(max_gap_m, gap_m)
            if hit_payload is None and self._point_is_in_inflated_obstacle_from_cells(
                point_xyz[0],
                point_xyz[1],
                point_xyz[2],
                occupied,
            ):
                hit_payload = {
                    "point": self._point_dict(point_xyz[0], point_xyz[1], point_xyz[2]),
                    "curve_local_t_sec": round(float(min(t_sec, checked_duration_sec)), 3),
                }
            last_point = point_xyz
            t_sec += max(float(step_sec), 1e-3)
        return hit_payload, round(float(max_gap_m), 3)

    def _bspline_curve_occupancy_summary(self, bspline_msg, occupied_cells=None):
        if bspline_msg is None or not bspline_msg.pos_pts or not bspline_msg.knots:
            return None
        occupied = self._occupied_cells_snapshot() if occupied_cells is None else occupied_cells
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
        checked_curve_length_m = self._compute_checked_curve_length(bspline_msg, planner_check_limit_sec)
        planner_code_step_sec = planner_check_limit_sec
        if checked_curve_length_m > 1e-6:
            planner_code_step_sec = planner_check_limit_sec / max(
                1.0,
                math.ceil(checked_curve_length_m / max(resolution_m * 0.5, 1e-3)),
            )
        planner_code_step_sec = max(float(planner_code_step_sec), 1e-3)

        sample_step_sec = 0.02
        sample_count = max(int(math.ceil(duration_sec / sample_step_sec)), 1)
        first_curve_hit = None
        for sample_index in range(sample_count + 1):
            t_sec = min(float(sample_index) * sample_step_sec, duration_sec)
            point_xyz = self._evaluate_bspline_t(bspline_msg, t_sec)
            if point_xyz is None:
                continue
            if self._point_is_in_inflated_obstacle_from_cells(point_xyz[0], point_xyz[1], point_xyz[2], occupied):
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
            if point_xyz is not None and self._point_is_in_inflated_obstacle_from_cells(
                point_xyz[0],
                point_xyz[1],
                point_xyz[2],
                occupied,
            ):
                planner_like_hit = {
                    "point": self._point_dict(point_xyz[0], point_xyz[1], point_xyz[2]),
                    "curve_local_t_sec": round(float(planner_t_sec), 3),
                }
                break
            planner_t_sec += planner_like_step_sec

        planner_code_hit, planner_code_max_gap_m = self._sample_bspline_hit_with_step(
            bspline_msg,
            planner_check_limit_sec,
            planner_code_step_sec,
            occupied_cells=occupied,
        )

        return {
            "duration_sec": round(float(duration_sec), 3),
            "planner_like_step_sec": round(float(planner_like_step_sec), 3),
            "planner_like_check_limit_sec": round(float(planner_check_limit_sec), 3),
            "planner_code_checked_curve_length_m": round(float(checked_curve_length_m), 3),
            "planner_code_step_sec": round(float(planner_code_step_sec), 3),
            "planner_code_max_gap_m": round(float(planner_code_max_gap_m), 3),
            "first_curve_hit": first_curve_hit,
            "planner_like_hit": planner_like_hit,
            "planner_code_hit": planner_code_hit,
        }

    def _capture_first_occurrence(self, recv_time, goal_send_time, current_value, extractor, now_rel_sec, now_wall):
        if current_value is None or not self._recv_after_goal(recv_time, goal_send_time):
            return None
        extract_result = extractor(current_value)
        if extract_result is None:
            return None
        if isinstance(extract_result, dict) and "point" in extract_result:
            event = dict(extract_result)
        else:
            event = {
                "point": extract_result,
            }
        message_age_sec = max(0.0, float(now_wall - recv_time))
        event.update({
            "t_sec": round(now_rel_sec, 3),
            "state": self._state_snapshot(),
            "message_age_sec": round(message_age_sec, 3),
            "approx_publish_t_sec": round(max(0.0, float(now_rel_sec) - message_age_sec), 3),
        })
        return event

    def _first_stage2_goal_event(self, msg):
        x_value = float(msg.pose.position.x)
        y_value = float(msg.pose.position.y)
        z_value = float(msg.pose.position.z)
        if not self.point_is_in_inflated_obstacle(x_value, y_value, z_value):
            return None
        event = {"point": self._point_dict(x_value, y_value, z_value)}
        event.update(self._point_timeline_against_occupancy(x_value, y_value, z_value, self.last_stage2_goal_recv_time))
        return event

    def _first_ego_goal_event(self, msg):
        x_value = float(msg.pose.position.x)
        y_value = float(msg.pose.position.y)
        z_value = float(msg.pose.position.z)
        if not self.point_is_in_inflated_obstacle(x_value, y_value, z_value):
            return None
        event = {"point": self._point_dict(x_value, y_value, z_value)}
        event.update(self._point_timeline_against_occupancy(x_value, y_value, z_value, self.last_ego_goal_recv_time))
        return event

    def _first_waypoint_event(self, path_msg):
        if path_msg is None or not path_msg.poses:
            return None
        point = path_msg.poses[0].pose.position
        x_value = float(point.x)
        y_value = float(point.y)
        z_value = float(point.z)
        if not self.point_is_in_inflated_obstacle(x_value, y_value, z_value):
            return None
        event = {"point": self._point_dict(x_value, y_value, z_value)}
        event.update(self._point_timeline_against_occupancy(x_value, y_value, z_value, self.last_waypoints_recv_time))
        return event

    def _first_bspline_curve_event(self, bspline_msg, bspline_recv_time):
        summary = self._bspline_curve_occupancy_summary(bspline_msg)
        if summary is None or summary["first_curve_hit"] is None:
            return None
        occupancy_before_bspline = self._latest_occupancy_cells_before(bspline_recv_time)
        occupancy_after_bspline = self._first_occupancy_cells_at_or_after(bspline_recv_time)
        first_detecting_after_bspline = None
        for entry in self._occupancy_history_snapshot():
            if float(entry["recv_time"]) < float(bspline_recv_time) - 1e-9:
                continue
            candidate_summary = self._bspline_curve_occupancy_summary(
                bspline_msg,
                occupied_cells=entry["occupied_cells"],
            )
            if candidate_summary is not None and candidate_summary["first_curve_hit"] is not None:
                first_detecting_after_bspline = (entry, candidate_summary)
                break
        prior_summary = None
        if occupancy_before_bspline is not None:
            prior_summary = self._bspline_curve_occupancy_summary(
                bspline_msg,
                occupied_cells=occupancy_before_bspline["occupied_cells"],
            )
        next_summary = None
        if occupancy_after_bspline is not None:
            next_summary = self._bspline_curve_occupancy_summary(
                bspline_msg,
                occupied_cells=occupancy_after_bspline["occupied_cells"],
            )
        return {
            "point": summary["first_curve_hit"]["point"],
            "bspline_traj_id": int(getattr(bspline_msg, "traj_id", 0)),
            "cmd_trajectory_id_at_detect": (
                None if self.last_position_cmd is None else int(getattr(self.last_position_cmd, "trajectory_id", 0))
            ),
            "curve_local_t_sec": summary["first_curve_hit"]["curve_local_t_sec"],
            "beyond_planner_two_thirds": summary["first_curve_hit"]["beyond_planner_two_thirds"],
            "planner_like_step_sec": summary["planner_like_step_sec"],
            "planner_like_check_limit_sec": summary["planner_like_check_limit_sec"],
            "planner_like_hit_curve_local_t_sec": (
                None
                if summary["planner_like_hit"] is None
                else summary["planner_like_hit"]["curve_local_t_sec"]
            ),
            "planner_like_would_detect": bool(summary["planner_like_hit"] is not None),
            "planner_code_checked_curve_length_m": summary["planner_code_checked_curve_length_m"],
            "planner_code_step_sec": summary["planner_code_step_sec"],
            "planner_code_max_gap_m": summary["planner_code_max_gap_m"],
            "planner_code_hit_curve_local_t_sec": (
                None
                if summary["planner_code_hit"] is None
                else summary["planner_code_hit"]["curve_local_t_sec"]
            ),
            "planner_code_would_detect": bool(summary["planner_code_hit"] is not None),
            "occupancy_before_bspline_recv_age_sec": (
                None
                if occupancy_before_bspline is None
                else round(max(0.0, float(bspline_recv_time) - float(occupancy_before_bspline["recv_time"])), 3)
            ),
            "occupancy_before_bspline_recv_curve_hit_t_sec": (
                None
                if prior_summary is None or prior_summary["first_curve_hit"] is None
                else prior_summary["first_curve_hit"]["curve_local_t_sec"]
            ),
            "occupancy_before_bspline_recv_planner_like_hit_curve_local_t_sec": (
                None
                if prior_summary is None or prior_summary["planner_like_hit"] is None
                else prior_summary["planner_like_hit"]["curve_local_t_sec"]
            ),
            "occupancy_before_bspline_recv_planner_like_would_detect": bool(
                prior_summary is not None and prior_summary["planner_like_hit"] is not None
            ),
            "occupancy_before_bspline_recv_planner_code_hit_curve_local_t_sec": (
                None
                if prior_summary is None or prior_summary["planner_code_hit"] is None
                else prior_summary["planner_code_hit"]["curve_local_t_sec"]
            ),
            "occupancy_before_bspline_recv_planner_code_would_detect": bool(
                prior_summary is not None and prior_summary["planner_code_hit"] is not None
            ),
            "occupancy_at_or_after_bspline_recv_age_sec": (
                None
                if occupancy_after_bspline is None
                else round(max(0.0, float(occupancy_after_bspline["recv_time"]) - float(bspline_recv_time)), 3)
            ),
            "occupancy_at_or_after_bspline_recv_curve_hit_t_sec": (
                None
                if next_summary is None or next_summary["first_curve_hit"] is None
                else next_summary["first_curve_hit"]["curve_local_t_sec"]
            ),
            "occupancy_at_or_after_bspline_recv_planner_like_hit_curve_local_t_sec": (
                None
                if next_summary is None or next_summary["planner_like_hit"] is None
                else next_summary["planner_like_hit"]["curve_local_t_sec"]
            ),
            "occupancy_at_or_after_bspline_recv_planner_like_would_detect": bool(
                next_summary is not None and next_summary["planner_like_hit"] is not None
            ),
            "occupancy_at_or_after_bspline_recv_planner_code_hit_curve_local_t_sec": (
                None
                if next_summary is None or next_summary["planner_code_hit"] is None
                else next_summary["planner_code_hit"]["curve_local_t_sec"]
            ),
            "occupancy_at_or_after_bspline_recv_planner_code_would_detect": bool(
                next_summary is not None and next_summary["planner_code_hit"] is not None
            ),
            "occupancy_first_detect_after_bspline_recv_age_sec": (
                None
                if first_detecting_after_bspline is None
                else round(
                    max(
                        0.0,
                        float(first_detecting_after_bspline[0]["recv_time"]) - float(bspline_recv_time),
                    ),
                    3,
                )
            ),
            "occupancy_first_detect_after_bspline_recv_curve_hit_t_sec": (
                None
                if first_detecting_after_bspline is None
                else first_detecting_after_bspline[1]["first_curve_hit"]["curve_local_t_sec"]
            ),
            "occupancy_first_detect_after_bspline_recv_planner_like_would_detect": bool(
                first_detecting_after_bspline is not None
                and first_detecting_after_bspline[1]["planner_like_hit"] is not None
            ),
            "occupancy_first_detect_after_bspline_recv_planner_code_would_detect": bool(
                first_detecting_after_bspline is not None
                and first_detecting_after_bspline[1]["planner_code_hit"] is not None
            ),
        }

    def run(self, dwell_sec=14.0, prewait_occupancy_sec=0.0, wait_external_goal=False):
        if not self.wait_odom():
            return {"status": "failed", "reason": "odom_timeout"}

        odom = self.last_odom
        yaw_rad = yaw_from_quaternion(odom.pose.pose.orientation)
        start_x = odom.pose.pose.position.x
        start_y = odom.pose.pose.position.y
        start_z = odom.pose.pose.position.z
        goal_xy = (
            start_x + math.cos(yaw_rad) * 10.5,
            start_y + math.sin(yaw_rad) * 10.5,
        )

        occupancy_stamp, has_occupied = self._occupancy_stamp_and_nonempty()
        had_occupancy_before_goal = occupancy_stamp is not None and has_occupied
        if prewait_occupancy_sec > 0.0:
            if not self.wait_occupancy_after(0.0):
                return {
                    "status": "failed",
                    "reason": "occupancy_timeout_before_goal",
                    "had_occupancy_before_goal": bool(had_occupancy_before_goal),
                }
            time.sleep(float(prewait_occupancy_sec))
            occupancy_stamp, has_occupied = self._occupancy_stamp_and_nonempty()
            had_occupancy_before_goal = occupancy_stamp is not None and has_occupied
        if wait_external_goal:
            if not self.wait_external_goal(timeout_sec=8.0):
                return {"status": "failed", "reason": "external_goal_timeout"}
            goal_send_time = float(self.goal_send_time)
            if self.external_goal_point is not None:
                point = self.external_goal_point["point"]
                goal_xy = (float(point["x"]), float(point["y"]))
        else:
            goal_send_time = time.time()
            self.goal_send_time = goal_send_time
            self.external_goal_point = {
                "frame_id": "map",
                "point": self._point_dict(goal_xy[0], goal_xy[1], 0.0),
            }
            self.send_goal(*goal_xy)
        if not self.wait_occupancy_after(goal_send_time):
            return {
                "status": "failed",
                "reason": "occupancy_timeout_after_goal",
                "had_occupancy_before_goal": bool(had_occupancy_before_goal),
            }

        occupancy_stamp, _ = self._occupancy_stamp_and_nonempty()
        occupancy_ready_delay_sec = max(0.0, float(occupancy_stamp - goal_send_time))

        max_travel = 0.0
        max_abs_lateral_offset = 0.0
        entered_obstacle = False
        last_stage2 = None
        first_stage2_goal_in_occ = None
        first_ego_goal_in_occ = None
        first_waypoint_in_occ = None
        first_bspline_in_occ = None
        first_bspline_curve_in_occ = None
        first_position_cmd_in_occ = None
        first_odom_in_occ = None

        deadline = time.time() + dwell_sec
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)
            now_wall = time.time()
            now_rel_sec = now_wall - goal_send_time

            if self.last_odom is not None:
                x_value = self.last_odom.pose.pose.position.x
                y_value = self.last_odom.pose.pose.position.y
                z_value = self.last_odom.pose.pose.position.z
                travel = math.hypot(x_value - start_x, y_value - start_y)
                max_travel = max(max_travel, travel)
                max_abs_lateral_offset = max(max_abs_lateral_offset, abs(y_value - start_y))
                if self.point_is_in_inflated_obstacle(x_value, y_value, z_value):
                    entered_obstacle = True
                if (
                    first_odom_in_occ is None
                    and self._recv_after_goal(self.last_odom_recv_time, goal_send_time)
                    and self.point_is_in_inflated_obstacle(x_value, y_value, z_value)
                ):
                    message_age_sec = max(0.0, float(now_wall - self.last_odom_recv_time))
                    first_odom_in_occ = {
                        "t_sec": round(now_rel_sec, 3),
                        "point": self._point_dict(x_value, y_value, z_value),
                        "state": self._state_snapshot(),
                        "message_age_sec": round(message_age_sec, 3),
                        "approx_publish_t_sec": round(max(0.0, float(now_rel_sec) - message_age_sec), 3),
                    }

            if self.last_state is not None and self._recv_after_goal(self.last_state_recv_time, goal_send_time):
                last_stage2 = (self.last_state.state_name, self.last_state.detail)

            if first_stage2_goal_in_occ is None:
                first_stage2_goal_in_occ = self._capture_first_occurrence(
                    self.last_stage2_goal_recv_time,
                    goal_send_time,
                    self.last_stage2_goal,
                    self._first_stage2_goal_event,
                    now_rel_sec,
                    now_wall,
                )

            if first_ego_goal_in_occ is None:
                first_ego_goal_in_occ = self._capture_first_occurrence(
                    self.last_ego_goal_recv_time,
                    goal_send_time,
                    self.last_ego_goal,
                    self._first_ego_goal_event,
                    now_rel_sec,
                    now_wall,
                )

            if first_waypoint_in_occ is None:
                first_waypoint_in_occ = self._capture_first_occurrence(
                    self.last_waypoints_recv_time,
                    goal_send_time,
                    self.last_waypoints,
                    self._first_waypoint_event,
                    now_rel_sec,
                    now_wall,
                )

            if first_bspline_in_occ is None:
                first_bspline_in_occ = self._capture_first_occurrence(
                    self.last_bspline_recv_time,
                    goal_send_time,
                    self.last_bspline,
                    self._first_point_in_occ_from_bspline,
                    now_rel_sec,
                    now_wall,
                )

            if first_bspline_curve_in_occ is None:
                first_bspline_curve_in_occ = self._capture_first_occurrence(
                    self.last_bspline_recv_time,
                    goal_send_time,
                    self.last_bspline,
                    lambda msg: self._first_bspline_curve_event(msg, self.last_bspline_recv_time),
                    now_rel_sec,
                    now_wall,
                )

            if first_position_cmd_in_occ is None:
                first_position_cmd_in_occ = self._capture_first_occurrence(
                    self.last_position_cmd_recv_time,
                    goal_send_time,
                    self.last_position_cmd,
                    lambda msg: (
                        self._point_dict(msg.position.x, msg.position.y, msg.position.z)
                        if self.point_is_in_inflated_obstacle(
                            msg.position.x,
                            msg.position.y,
                            msg.position.z,
                        )
                        else None
                    ),
                    now_rel_sec,
                    now_wall,
                )

        first_occurrences = {
            "stage2_goal": first_stage2_goal_in_occ,
            "ego_goal": first_ego_goal_in_occ,
            "waypoint": first_waypoint_in_occ,
            "bspline_curve": first_bspline_curve_in_occ,
            "position_cmd": first_position_cmd_in_occ,
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

        return {
            "status": "ok",
            "goal_xy": [round(goal_xy[0], 3), round(goal_xy[1], 3)],
            "start_xyz": [round(start_x, 3), round(start_y, 3), round(start_z, 3)],
            "wait_external_goal": bool(wait_external_goal),
            "external_goal_point": self.external_goal_point,
            "max_travel_xy_m": round(max_travel, 3),
            "max_abs_lateral_offset_m": round(max_abs_lateral_offset, 3),
            "entered_obstacle_box": entered_obstacle,
            "last_stage2": last_stage2,
            "had_occupancy_before_goal": bool(had_occupancy_before_goal),
            "occupancy_ready_delay_sec": round(occupancy_ready_delay_sec, 3),
            "map_inflation_m": round(float(self.map_inflation_m), 3),
            "obstacle_clearance_m": round(float(self.extra_clearance_m), 3),
            "effective_total_clearance_m": round(float(self.map_inflation_m + self.extra_clearance_m), 3),
            "occupancy_resolution_m": round(float(self.occupancy_resolution_m), 3),
            "prewait_occupancy_sec": round(float(prewait_occupancy_sec), 3),
            "earliest_failing_layer": earliest_layer,
            "first_stage2_goal_in_occ": first_stage2_goal_in_occ,
            "first_ego_goal_in_occ": first_ego_goal_in_occ,
            "first_waypoint_in_occ": first_waypoint_in_occ,
            "first_bspline_in_occ": first_bspline_in_occ,
            "first_bspline_curve_in_occ": first_bspline_curve_in_occ,
            "first_position_cmd_in_occ": first_position_cmd_in_occ,
            "first_odom_in_occ": first_odom_in_occ,
            "first_bspline_curve_grid_cloud_nearest": (
                self._nearest_grid_cloud_summary(
                    (
                        first_bspline_curve_in_occ["point"]["x"],
                        first_bspline_curve_in_occ["point"]["y"],
                        first_bspline_curve_in_occ["point"]["z"],
                    )
                )
                if first_bspline_curve_in_occ is not None
                else None
            ),
            "first_stage2_goal_grid_cloud_nearest": (
                self._nearest_grid_cloud_summary(
                    (
                        first_stage2_goal_in_occ["point"]["x"],
                        first_stage2_goal_in_occ["point"]["y"],
                        first_stage2_goal_in_occ["point"]["z"],
                    )
                )
                if first_stage2_goal_in_occ is not None
                else None
            ),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--clearance-m", type=float, default=None)
    parser.add_argument("--dwell-sec", type=float, default=14.0)
    parser.add_argument("--prewait-occupancy-sec", type=float, default=0.0)
    parser.add_argument("--wait-external-goal", action="store_true")
    args = parser.parse_args()

    rospy.init_node("stage2_center_obstacle_trace_probe", anonymous=True)
    probe = CenterObstacleTraceProbe(clearance_override_m=args.clearance_m)
    result = probe.run(
        dwell_sec=float(args.dwell_sec),
        prewait_occupancy_sec=float(args.prewait_occupancy_sec),
        wait_external_goal=bool(args.wait_external_goal),
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result.get("status") != "ok":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
