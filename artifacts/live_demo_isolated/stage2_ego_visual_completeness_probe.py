#!/usr/bin/env python3
import argparse
import json
import math
import sys
import subprocess
import time
from pathlib import Path

import numpy as np
import rospy
from ego_planner.msg import Bspline
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

from human_follow_msgs.msg import FollowState


ROOT = Path("/home/coco/follwer_ws")
RVIZ_CONFIG = ROOT / "src/human_follow_bringup/rviz/stage2_real_ego_integrated.rviz"
FUSION_SCRIPT_DIR = ROOT / "src" / "human_follow_fusion" / "scripts"
if str(FUSION_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(FUSION_SCRIPT_DIR))

from projection_math import (  # noqa: E402
    load_camera_lidar_extrinsics_yaml,
    load_rigid_transform_yaml,
    transform_points,
)


def _yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


def _distance_xy(a_x, a_y, b_x, b_y):
    return math.hypot(float(a_x) - float(b_x), float(a_y) - float(b_y))


def _point_dict(point):
    return {
        "x": round(float(point[0]), 3),
        "y": round(float(point[1]), 3),
        "z": round(float(point[2]), 3),
    }


def _evaluate_bspline_t(bspline_msg, t_sec):
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
    if upper_u <= lower_u:
        return None
    u_value = min(max(lower_u, float(t_sec) + lower_u), upper_u)

    k_index = order
    while k_index + 1 < len(knots) and knots[k_index + 1] < u_value:
        k_index += 1

    left_index = k_index - order
    if left_index < 0 or left_index + order >= len(control_points):
        return None
    work = [control_points[left_index + i][:] for i in range(order + 1)]
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


class _TopicWatch:
    def __init__(self, topic, msg_type, queue_size=20):
        self.topic = topic
        self.msg_type = msg_type
        self.count = 0
        self.last_msg = None
        self.first_recv_time = None
        self.last_recv_time = None
        self.max_marker_point_count = 0
        self.max_path_pose_count = 0
        self.sub = rospy.Subscriber(topic, msg_type, self._callback, queue_size=queue_size)

    def _callback(self, msg):
        now = time.time()
        self.count += 1
        self.last_msg = msg
        if self.first_recv_time is None:
            self.first_recv_time = now
        self.last_recv_time = now
        if hasattr(msg, "points"):
            self.max_marker_point_count = max(self.max_marker_point_count, len(getattr(msg, "points", [])))
        if hasattr(msg, "poses"):
            self.max_path_pose_count = max(self.max_path_pose_count, len(getattr(msg, "poses", [])))

    def age_sec(self):
        if self.last_recv_time is None:
            return None
        return max(0.0, time.time() - float(self.last_recv_time))


class Stage2EgoVisualCompletenessProbe:
    def __init__(self, extra_clearance_m=0.05):
        self.extra_clearance_m = max(float(extra_clearance_m), 0.0)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.occupancy_resolution_m = float(rospy.get_param("/ego_planner_node/grid_map/resolution", 0.1))
        self.map_inflation_m = max(float(rospy.get_param("/ego_planner_node/grid_map/obstacles_inflation", 0.1)), 0.0)
        self.occupied_cells = set()
        self.static_obstacle_cells = set()
        self.foreground_cells = set()
        self.last_occupancy_recv_time = None
        self.last_static_obstacle_recv_time = None
        self.last_foreground_recv_time = None
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
                rospy.logwarn("failed to load cloud transforms for visual completeness diagnostics: %s", exc)

        self.watchers = {
            "odom": _TopicWatch("/follow/sim/vehicle_odom", Odometry),
            "grid_odom": _TopicWatch("/grid_map/odom", Odometry),
            "odom_world": _TopicWatch("/odom_world", Odometry),
            "stage2_goal": _TopicWatch("/follow/stage2/goal", PoseStamped),
            "stage2_debug_goal_path": _TopicWatch("/follow/stage2/debug_goal_path", RosPath),
            "ego_goal": _TopicWatch("/follow/stage2/ego_goal", PoseStamped),
            "waypoints": _TopicWatch("/waypoint_generator/waypoints", RosPath),
            "bspline": _TopicWatch("/planning/bspline", Bspline),
            "bspline_path": _TopicWatch("/follow/stage2/ego_bspline_path", RosPath),
            "ego_cmd": _TopicWatch("/follow/stage2/ego_position_cmd", PositionCommand),
            "ego_cmd_face_target": _TopicWatch("/follow/stage2/ego_position_cmd_face_target", PositionCommand),
            "stage2_state": _TopicWatch("/follow/stage2/state", FollowState),
            "grid_cloud": _TopicWatch("/grid_map/cloud", PointCloud2),
            "occupancy": _TopicWatch("/grid_map/occupancy_inflate", PointCloud2),
            "static_obstacle_cloud": _TopicWatch("/follow/lidar/obstacle_points", PointCloud2),
            "foreground_cloud": _TopicWatch("/follow/lidar/foreground_points", PointCloud2),
            "ego_goal_marker": _TopicWatch("/ego_planner_node/goal_point", Marker),
            "ego_global_marker": _TopicWatch("/ego_planner_node/global_list", Marker),
            "ego_optimal_marker": _TopicWatch("/ego_planner_node/optimal_list", Marker),
            "ego_astar_marker": _TopicWatch("/ego_planner_node/a_star_list", Marker),
            "ego_init_marker": _TopicWatch("/ego_planner_node/init_list", Marker),
        }
        rospy.Subscriber("/grid_map/occupancy_inflate", PointCloud2, self._occupancy_cb, queue_size=5)
        rospy.Subscriber("/follow/lidar/obstacle_points", PointCloud2, self._static_obstacle_cb, queue_size=5)
        rospy.Subscriber("/follow/lidar/foreground_points", PointCloud2, self._foreground_cb, queue_size=5)

    def _cell_key(self, x_value, y_value, z_value):
        resolution = max(self.occupancy_resolution_m, 0.02)
        return (
            int(round(float(x_value) / resolution)),
            int(round(float(y_value) / resolution)),
            int(round(float(z_value) / resolution)),
        )

    def _occupancy_cb(self, msg):
        occupied = set()
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            occupied.add(self._cell_key(point[0], point[1], point[2]))
        self.occupied_cells = occupied
        self.last_occupancy_recv_time = time.time()

    def _cloud_to_world_points(self, msg):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            return np.zeros((0, 3), dtype=np.float64)

        raw_points = np.asarray(points, dtype=np.float64)
        frame_id = str(getattr(msg.header, "frame_id", ""))
        if frame_id in ("map", "world", self.world_frame_id):
            return raw_points

        odom = self.watchers["odom"].last_msg
        if odom is None or self._camera_lidar_extrinsics is None or self._body_from_camera is None:
            return np.zeros((0, 3), dtype=np.float64)

        camera_points = transform_points(raw_points, self._camera_lidar_extrinsics["camera_T_lidar"])
        body_points = transform_points(camera_points, self._body_from_camera)
        pose = odom.pose.pose
        yaw_rad = _yaw_from_quaternion(pose.orientation)
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

    def _static_obstacle_cb(self, msg):
        self.static_obstacle_cells = self._cells_from_cloud(msg)
        self.last_static_obstacle_recv_time = time.time()

    def _foreground_cb(self, msg):
        self.foreground_cells = self._cells_from_cloud(msg)
        self.last_foreground_recv_time = time.time()

    def _point_is_in_cells(self, cells, x_value, y_value, z_value, clearance_m):
        if not cells:
            return False
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

    def _point_is_in_inflated_obstacle(self, x_value, y_value, z_value):
        return self._point_is_in_cells(
            self.occupied_cells,
            x_value,
            y_value,
            z_value,
            self.extra_clearance_m,
        )

    def _point_is_in_static_obstacle(self, x_value, y_value, z_value):
        return self._point_is_in_cells(
            self.static_obstacle_cells,
            x_value,
            y_value,
            z_value,
            self.map_inflation_m + self.extra_clearance_m,
        )

    def _point_is_in_foreground(self, x_value, y_value, z_value):
        return self._point_is_in_cells(
            self.foreground_cells,
            x_value,
            y_value,
            z_value,
            self.map_inflation_m + self.extra_clearance_m,
        )

    def _age_sec(self, stamp):
        if stamp is None:
            return None
        return max(0.0, time.time() - float(stamp))

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
            "point": _point_dict(nearest),
            "distance_m": round(float(best_dist), 3),
        }

    def _hit_context(self, x_value, y_value, z_value):
        return {
            "all_ego_occupancy_hit": self._point_is_in_inflated_obstacle(x_value, y_value, z_value),
            "static_obstacle_hit": self._point_is_in_static_obstacle(x_value, y_value, z_value),
            "foreground_hit": self._point_is_in_foreground(x_value, y_value, z_value),
            "occupancy_age_sec": None if self._age_sec(self.last_occupancy_recv_time) is None else round(self._age_sec(self.last_occupancy_recv_time), 3),
            "static_obstacle_cloud_age_sec": None if self._age_sec(self.last_static_obstacle_recv_time) is None else round(self._age_sec(self.last_static_obstacle_recv_time), 3),
            "foreground_cloud_age_sec": None if self._age_sec(self.last_foreground_recv_time) is None else round(self._age_sec(self.last_foreground_recv_time), 3),
            "nearest_all_occupancy": self._nearest_cell_context(self.occupied_cells, x_value, y_value, z_value),
            "nearest_static_obstacle": self._nearest_cell_context(self.static_obstacle_cells, x_value, y_value, z_value),
            "nearest_foreground": self._nearest_cell_context(self.foreground_cells, x_value, y_value, z_value),
        }

    def _wait_for(self, predicate, timeout_sec):
        deadline = time.time() + float(timeout_sec)
        while time.time() < deadline and not rospy.is_shutdown():
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def _wait_for_initial_chain(self, timeout_sec=12.0):
        required = ("odom", "grid_cloud")
        return self._wait_for(
            lambda: all(self.watchers[name].last_msg is not None for name in required),
            timeout_sec,
        )

    def _send_forward_goal(self, goal_forward_m):
        odom = self.watchers["odom"].last_msg
        yaw_rad = _yaw_from_quaternion(odom.pose.pose.orientation)
        start_x = float(odom.pose.pose.position.x)
        start_y = float(odom.pose.pose.position.y)
        goal_x = start_x + math.cos(yaw_rad) * float(goal_forward_m)
        goal_y = start_y + math.sin(yaw_rad) * float(goal_forward_m)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        for _ in range(4):
            self.goal_pub.publish(msg)
            time.sleep(0.05)
        return {
            "start": [round(start_x, 3), round(start_y, 3), round(float(odom.pose.pose.position.z), 3)],
            "goal": [round(goal_x, 3), round(goal_y, 3), 0.0],
        }

    def _rosnode_list(self):
        try:
            out = subprocess.check_output(["rosnode", "list"], text=True, timeout=5)
        except Exception:
            return []
        return [line.strip() for line in out.splitlines() if line.strip()]

    def _rviz_topics(self):
        text = RVIZ_CONFIG.read_text(errors="replace")
        topics = []
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("Topic: ") or stripped.startswith("Marker Topic: "):
                topics.append(stripped.split(":", 1)[1].strip())
        return sorted(set(topics))

    def _rviz_display_names(self):
        names = []
        for line in RVIZ_CONFIG.read_text(errors="replace").splitlines():
            stripped = line.strip()
            if stripped.startswith("Name: "):
                names.append(stripped.split(":", 1)[1].strip())
        return names

    def _path_summary(self, msg):
        if msg is None:
            return {"present": False}
        poses = list(getattr(msg, "poses", []))
        length = 0.0
        first = None
        last = None
        prev = None
        first_occ = None
        for pose_stamped in poses:
            point = pose_stamped.pose.position
            curr = (float(point.x), float(point.y), float(point.z))
            if first is None:
                first = curr
            last = curr
            if prev is not None:
                length += math.sqrt(
                    (curr[0] - prev[0]) ** 2
                    + (curr[1] - prev[1]) ** 2
                    + (curr[2] - prev[2]) ** 2
                )
            if first_occ is None and self._point_is_in_inflated_obstacle(curr[0], curr[1], curr[2]):
                first_occ = _point_dict(curr)
            prev = curr
        return {
            "present": True,
            "frame_id": str(msg.header.frame_id),
            "pose_count": len(poses),
            "length_m": round(length, 3),
            "first": None if first is None else _point_dict(first),
            "last": None if last is None else _point_dict(last),
            "first_point_in_inflated_obstacle": first_occ,
        }

    def _marker_summary(self, msg):
        if msg is None:
            return {"present": False}
        return {
            "present": True,
            "frame_id": str(msg.header.frame_id),
            "type": int(msg.type),
            "point_count": len(getattr(msg, "points", [])),
            "ns": str(getattr(msg, "ns", "")),
            "id": int(getattr(msg, "id", 0)),
        }

    def _cloud_summary(self, msg, max_points=20000):
        if msg is None:
            return {"present": False}
        count = 0
        min_xyz = [float("inf"), float("inf"), float("inf")]
        max_xyz = [float("-inf"), float("-inf"), float("-inf")]
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            count += 1
            if count > max_points:
                break
            for idx in range(3):
                value = float(point[idx])
                min_xyz[idx] = min(min_xyz[idx], value)
                max_xyz[idx] = max(max_xyz[idx], value)
        return {
            "present": True,
            "frame_id": str(msg.header.frame_id),
            "sampled_points": min(count, max_points),
            "truncated": count > max_points,
            "min": None if count == 0 else [round(v, 3) for v in min_xyz],
            "max": None if count == 0 else [round(v, 3) for v in max_xyz],
        }

    def _bspline_summary(self, msg):
        if msg is None:
            return {"present": False}
        order = int(msg.order)
        pos_pts = list(msg.pos_pts)
        knots = [float(v) for v in msg.knots]
        duration = None
        if len(knots) > order * 2:
            duration = knots[-order - 1] - knots[order]
        first_occ = None
        for point in pos_pts:
            if self._point_is_in_inflated_obstacle(point.x, point.y, point.z):
                first_occ = _point_dict((point.x, point.y, point.z))
                break
        return {
            "present": True,
            "traj_id": int(getattr(msg, "traj_id", 0)),
            "order": order,
            "control_point_count": len(pos_pts),
            "knot_count": len(knots),
            "duration_sec": None if duration is None else round(float(duration), 3),
            "first_control_point_in_inflated_obstacle": first_occ,
        }

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

        planner_prefix_limit_sec = max(duration_sec * (2.0 / 3.0), 0.0)
        first_full_hit = None
        first_prefix_hit = None
        sample_step_sec = 0.02
        sample_count = max(int(math.ceil(duration_sec / sample_step_sec)), 1)
        for sample_index in range(sample_count + 1):
            local_t_sec = min(float(sample_index) * sample_step_sec, duration_sec)
            point = _evaluate_bspline_t(bspline_msg, local_t_sec)
            if point is None:
                continue
            if self._point_is_in_inflated_obstacle(point[0], point[1], point[2]):
                hit = {
                    "traj_id": int(getattr(bspline_msg, "traj_id", 0)),
                    "point": _point_dict(point),
                    "curve_local_t_sec": round(local_t_sec, 3),
                    "duration_sec": round(duration_sec, 3),
                    "fraction": round(local_t_sec / max(duration_sec, 1e-6), 3),
                    "beyond_planner_two_thirds": bool(local_t_sec > planner_prefix_limit_sec + 1e-9),
                }
                if first_full_hit is None:
                    first_full_hit = dict(hit)
                if first_prefix_hit is None and not hit["beyond_planner_two_thirds"]:
                    first_prefix_hit = dict(hit)
                if first_full_hit is not None and first_prefix_hit is not None:
                    break

        return {
            "duration_sec": round(duration_sec, 3),
            "planner_prefix_limit_sec": round(planner_prefix_limit_sec, 3),
            "first_full_hit": first_full_hit,
            "first_prefix_hit": first_prefix_hit,
        }

    def _cmd_odom_summary(self):
        cmd = self.watchers["ego_cmd"].last_msg
        cmd_face = self.watchers["ego_cmd_face_target"].last_msg
        odom = self.watchers["odom"].last_msg
        if cmd is None or odom is None:
            return {"present": False}
        active_cmd = cmd_face if cmd_face is not None else cmd
        cmd_xy = (float(active_cmd.position.x), float(active_cmd.position.y))
        odom_xy = (float(odom.pose.pose.position.x), float(odom.pose.pose.position.y))
        return {
            "present": True,
            "active_cmd_topic": "/follow/stage2/ego_position_cmd_face_target" if cmd_face is not None else "/follow/stage2/ego_position_cmd",
            "tracking_error_xy_m": round(_distance_xy(cmd_xy[0], cmd_xy[1], odom_xy[0], odom_xy[1]), 3),
            "cmd_speed_xy_mps": round(math.hypot(float(active_cmd.velocity.x), float(active_cmd.velocity.y)), 3),
            "cmd_traj_id": int(getattr(active_cmd, "trajectory_id", 0)),
            "odom_frame_id": str(odom.header.frame_id),
            "odom_child_frame_id": str(odom.child_frame_id),
        }

    def _odom_alignment_summary(self):
        odom = self.watchers["odom"].last_msg
        odom_world = self.watchers["odom_world"].last_msg
        grid_odom = self.watchers["grid_odom"].last_msg
        if odom is None or odom_world is None or grid_odom is None:
            return {"present": False}

        def xy_delta(a_msg, b_msg):
            return _distance_xy(
                a_msg.pose.pose.position.x,
                a_msg.pose.pose.position.y,
                b_msg.pose.pose.position.x,
                b_msg.pose.pose.position.y,
            )

        return {
            "present": True,
            "odom_frame_id": str(odom.header.frame_id),
            "odom_world_frame_id": str(odom_world.header.frame_id),
            "grid_odom_frame_id": str(grid_odom.header.frame_id),
            "odom_to_odom_world_xy_m": round(xy_delta(odom, odom_world), 4),
            "odom_to_grid_odom_xy_m": round(xy_delta(odom, grid_odom), 4),
        }

    def _watcher_counts(self):
        return {
            name: {
                "topic": watcher.topic,
                "count": watcher.count,
                "age_sec": None if watcher.age_sec() is None else round(watcher.age_sec(), 3),
                "max_marker_point_count": watcher.max_marker_point_count,
                "max_path_pose_count": watcher.max_path_pose_count,
            }
            for name, watcher in sorted(self.watchers.items())
        }

    def run(self, goal_forward_m=10.5, dwell_sec=14.0):
        initial_ok = self._wait_for_initial_chain()
        if not initial_ok:
            return {
                "status": "failed",
                "reason": "initial_chain_timeout",
                "watchers": self._watcher_counts(),
            }

        goal_info = self._send_forward_goal(goal_forward_m)
        goal_send_time = time.time()

        required_after_goal = (
            "stage2_goal",
            "ego_goal",
            "waypoints",
            "bspline",
            "bspline_path",
            "ego_cmd",
            "ego_cmd_face_target",
            "ego_optimal_marker",
        )
        after_goal_ok = self._wait_for(
            lambda: (
                all(
                    self.watchers[name].last_recv_time is not None
                    and self.watchers[name].last_recv_time >= goal_send_time
                    for name in required_after_goal
                )
                and bool(self.occupied_cells)
            ),
            timeout_sec=10.0,
        )

        start_odom = self.watchers["odom"].last_msg
        start_x = float(start_odom.pose.pose.position.x)
        start_y = float(start_odom.pose.pose.position.y)
        max_travel = 0.0
        max_tracking_error = 0.0
        odom_entered_obstacle = False
        odom_entered_static_obstacle = False
        cmd_entered_obstacle = False
        cmd_entered_static_obstacle = False
        first_odom_hit = None
        first_cmd_hit = None
        seen_bspline_traj_ids = set()
        max_bspline_control_points = 0
        max_bspline_duration_sec = 0.0
        max_bspline_path_pose_count = 0
        max_bspline_path_length_m = 0.0
        bspline_path_entered_obstacle = False
        first_bspline_curve_full_hit = None
        first_bspline_curve_prefix_hit = None
        states = []

        deadline = time.time() + float(dwell_sec)
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.1)
            odom = self.watchers["odom"].last_msg
            cmd = self.watchers["ego_cmd"].last_msg
            if odom is not None:
                ox = float(odom.pose.pose.position.x)
                oy = float(odom.pose.pose.position.y)
                oz = float(odom.pose.pose.position.z)
                max_travel = max(max_travel, _distance_xy(start_x, start_y, ox, oy))
                odom_hit = self._point_is_in_inflated_obstacle(ox, oy, oz)
                odom_static_hit = self._point_is_in_static_obstacle(ox, oy, oz)
                if odom_hit and first_odom_hit is None:
                    first_odom_hit = {
                        "t_sec": round(time.time() - goal_send_time, 3),
                        "point": _point_dict((ox, oy, oz)),
                        "hit_context": self._hit_context(ox, oy, oz),
                    }
                odom_entered_obstacle = odom_entered_obstacle or odom_hit
                odom_entered_static_obstacle = odom_entered_static_obstacle or odom_static_hit
            if cmd is not None:
                cx = float(cmd.position.x)
                cy = float(cmd.position.y)
                cz = float(cmd.position.z)
                cmd_hit = self._point_is_in_inflated_obstacle(cx, cy, cz)
                cmd_static_hit = self._point_is_in_static_obstacle(cx, cy, cz)
                if cmd_hit and first_cmd_hit is None:
                    first_cmd_hit = {
                        "t_sec": round(time.time() - goal_send_time, 3),
                        "point": _point_dict((cx, cy, cz)),
                        "trajectory_id": int(getattr(cmd, "trajectory_id", 0)),
                        "hit_context": self._hit_context(cx, cy, cz),
                    }
                cmd_entered_obstacle = cmd_entered_obstacle or cmd_hit
                cmd_entered_static_obstacle = cmd_entered_static_obstacle or cmd_static_hit
                if odom is not None:
                    max_tracking_error = max(
                        max_tracking_error,
                        _distance_xy(cx, cy, odom.pose.pose.position.x, odom.pose.pose.position.y),
                    )
            bspline = self.watchers["bspline"].last_msg
            if bspline is not None:
                traj_id = int(getattr(bspline, "traj_id", 0))
                is_new_traj = traj_id not in seen_bspline_traj_ids
                seen_bspline_traj_ids.add(traj_id)
                bspline_summary = self._bspline_summary(bspline)
                max_bspline_control_points = max(
                    max_bspline_control_points,
                    int(bspline_summary.get("control_point_count") or 0),
                )
                max_bspline_duration_sec = max(
                    max_bspline_duration_sec,
                    float(bspline_summary.get("duration_sec") or 0.0),
                )
                if is_new_traj:
                    curve_occ = self._bspline_curve_occupancy_summary(bspline)
                    if curve_occ is not None:
                        if first_bspline_curve_full_hit is None and curve_occ.get("first_full_hit") is not None:
                            first_bspline_curve_full_hit = dict(curve_occ["first_full_hit"])
                            first_bspline_curve_full_hit["cmd_trajectory_id_at_detect"] = (
                                None
                                if self.watchers["ego_cmd"].last_msg is None
                                else int(getattr(self.watchers["ego_cmd"].last_msg, "trajectory_id", 0))
                            )
                        if first_bspline_curve_prefix_hit is None and curve_occ.get("first_prefix_hit") is not None:
                            first_bspline_curve_prefix_hit = dict(curve_occ["first_prefix_hit"])
                            first_bspline_curve_prefix_hit["cmd_trajectory_id_at_detect"] = (
                                None
                                if self.watchers["ego_cmd"].last_msg is None
                                else int(getattr(self.watchers["ego_cmd"].last_msg, "trajectory_id", 0))
                            )
            bspline_path = self.watchers["bspline_path"].last_msg
            if bspline_path is not None:
                path_summary = self._path_summary(bspline_path)
                max_bspline_path_pose_count = max(
                    max_bspline_path_pose_count,
                    int(path_summary.get("pose_count") or 0),
                )
                max_bspline_path_length_m = max(
                    max_bspline_path_length_m,
                    float(path_summary.get("length_m") or 0.0),
                )
                bspline_path_entered_obstacle = (
                    bspline_path_entered_obstacle
                    or path_summary.get("first_point_in_inflated_obstacle") is not None
                )
            state = self.watchers["stage2_state"].last_msg
            if state is not None:
                snapshot = (str(state.state_name), str(state.detail))
                if not states or states[-1] != snapshot:
                    states.append(snapshot)

        nodes = self._rosnode_list()
        rviz_topics = self._rviz_topics()

        path_summaries = {
            "stage2_goal_pose": self._path_summary(_TopicAsPath.from_pose(self.watchers["stage2_goal"].last_msg, "map")),
            "stage2_debug_goal_path": self._path_summary(self.watchers["stage2_debug_goal_path"].last_msg),
            "ego_input_waypoints": self._path_summary(self.watchers["waypoints"].last_msg),
            "ego_bspline_path": self._path_summary(self.watchers["bspline_path"].last_msg),
        }
        marker_summaries = {
            "ego_goal_point": self._marker_summary(self.watchers["ego_goal_marker"].last_msg),
            "ego_global_list": self._marker_summary(self.watchers["ego_global_marker"].last_msg),
            "ego_optimal_list": self._marker_summary(self.watchers["ego_optimal_marker"].last_msg),
            "ego_astar_list": self._marker_summary(self.watchers["ego_astar_marker"].last_msg),
            "ego_init_list": self._marker_summary(self.watchers["ego_init_marker"].last_msg),
        }
        cloud_summaries = {
            "grid_cloud": self._cloud_summary(self.watchers["grid_cloud"].last_msg),
            "occupancy_inflate": self._cloud_summary(self.watchers["occupancy"].last_msg),
        }

        required_nodes = {
            "/ego_planner_node": "/ego_planner_node" in nodes,
            "/traj_server": "/traj_server" in nodes,
            "/stage2_follow_goal_generator": "/stage2_follow_goal_generator" in nodes,
            "/stage2_ego_topic_adapter": "/stage2_ego_topic_adapter" in nodes,
            "/stage2_move_base_goal_to_waypoint_path": "/stage2_move_base_goal_to_waypoint_path" in nodes,
            "/stage2_bspline_path_visualizer": "/stage2_bspline_path_visualizer" in nodes,
            "/vehicle_kinematic_sim": "/vehicle_kinematic_sim" in nodes,
        }
        required_rviz_topics = {
            "/grid_map/cloud": "/grid_map/cloud" in rviz_topics,
            "/grid_map/occupancy_inflate": "/grid_map/occupancy_inflate" in rviz_topics,
            "/follow/stage2/debug_goal_path": "/follow/stage2/debug_goal_path" in rviz_topics,
            "/waypoint_generator/waypoints": "/waypoint_generator/waypoints" in rviz_topics,
            "/follow/stage2/ego_bspline_path": "/follow/stage2/ego_bspline_path" in rviz_topics,
            "/ego_planner_node/goal_point": "/ego_planner_node/goal_point" in rviz_topics,
            "/ego_planner_node/optimal_list": "/ego_planner_node/optimal_list" in rviz_topics,
            "/ego_planner_node/a_star_list": "/ego_planner_node/a_star_list" in rviz_topics,
            "/ego_planner_node/init_list": "/ego_planner_node/init_list" in rviz_topics,
            "/follow/viz/vehicle_path": "/follow/viz/vehicle_path" in rviz_topics,
        }

        checks = {
            "required_nodes_alive": all(required_nodes.values()),
            "required_rviz_topics_configured": all(required_rviz_topics.values()),
            "after_goal_chain_observed": bool(after_goal_ok),
            "ego_bspline_nonempty": self.watchers["bspline"].last_msg is not None and len(self.watchers["bspline"].last_msg.pos_pts) >= 4,
            "ego_bspline_path_nonempty": path_summaries["ego_bspline_path"].get("pose_count", 0) >= 4,
            "ego_bspline_path_length_observed": max_bspline_path_length_m >= 0.2,
            "ego_replanned_multiple_times": len(seen_bspline_traj_ids) >= 2,
            "stage2_debug_goal_path_nonempty": path_summaries["stage2_debug_goal_path"].get("pose_count", 0) >= 1,
            "ego_global_marker_present": self.watchers["ego_global_marker"].max_marker_point_count >= 2,
            "ego_optimal_marker_present": self.watchers["ego_optimal_marker"].max_marker_point_count >= 2,
            "cloud_frame_map": cloud_summaries["grid_cloud"].get("frame_id") == "map",
            "occupancy_frame_world_or_map": cloud_summaries["occupancy_inflate"].get("frame_id") in ("map", "world"),
            "odom_frame_map": self._cmd_odom_summary().get("odom_frame_id") == "map",
            "odom_adapter_alignment_ok": (
                self._odom_alignment_summary().get("present")
                and self._odom_alignment_summary().get("odom_to_odom_world_xy_m", 999.0) <= 0.02
                and self._odom_alignment_summary().get("odom_to_grid_odom_xy_m", 999.0) <= 0.02
            ),
            "motion_ge_3m": max_travel >= 3.0,
            "tracking_error_bounded": max_tracking_error <= 0.35,
            "odom_not_in_obstacle": not odom_entered_obstacle,
            "odom_not_in_static_obstacle": not odom_entered_static_obstacle,
            "cmd_not_in_obstacle": not cmd_entered_obstacle,
            "cmd_not_in_static_obstacle": not cmd_entered_static_obstacle,
            "bspline_visual_path_not_in_obstacle": not bspline_path_entered_obstacle,
            "bspline_curve_prefix_not_in_obstacle": first_bspline_curve_prefix_hit is None,
            "waypoints_not_in_obstacle": path_summaries["ego_input_waypoints"].get("first_point_in_inflated_obstacle") is None,
            "stage2_goal_not_in_obstacle": path_summaries["stage2_goal_pose"].get("first_point_in_inflated_obstacle") is None,
            "stage2_debug_goal_path_not_in_obstacle": path_summaries["stage2_debug_goal_path"].get("first_point_in_inflated_obstacle") is None,
        }
        status = "passed" if all(bool(value) for value in checks.values()) else "failed"

        return {
            "status": status,
            "goal": goal_info,
            "checks": checks,
            "required_nodes": required_nodes,
            "required_rviz_topics": required_rviz_topics,
            "watchers": self._watcher_counts(),
            "paths": path_summaries,
            "markers": marker_summaries,
            "clouds": cloud_summaries,
            "bspline": self._bspline_summary(self.watchers["bspline"].last_msg),
            "cmd_odom": self._cmd_odom_summary(),
            "odom_alignment": self._odom_alignment_summary(),
            "dynamic_bspline": {
                "distinct_traj_id_count": len(seen_bspline_traj_ids),
                "max_control_point_count": int(max_bspline_control_points),
                "max_duration_sec": round(max_bspline_duration_sec, 3),
                "max_path_pose_count": int(max_bspline_path_pose_count),
                "max_path_length_m": round(max_bspline_path_length_m, 3),
                "path_entered_inflated_obstacle": bool(bspline_path_entered_obstacle),
                "first_curve_full_hit": first_bspline_curve_full_hit,
                "first_curve_prefix_hit": first_bspline_curve_prefix_hit,
            },
            "motion": {
                "max_travel_xy_m": round(max_travel, 3),
                "max_tracking_error_xy_m": round(max_tracking_error, 3),
                "odom_entered_inflated_obstacle": bool(odom_entered_obstacle),
                "odom_entered_static_obstacle": bool(odom_entered_static_obstacle),
                "cmd_entered_inflated_obstacle": bool(cmd_entered_obstacle),
                "cmd_entered_static_obstacle": bool(cmd_entered_static_obstacle),
                "first_odom_hit": first_odom_hit,
                "first_cmd_hit": first_cmd_hit,
                "effective_clearance_m": round(self.map_inflation_m + self.extra_clearance_m, 3),
                "all_occupancy_cell_count": len(self.occupied_cells),
                "static_obstacle_cell_count": len(self.static_obstacle_cells),
                "foreground_cell_count": len(self.foreground_cells),
            },
            "state_transitions": states[-20:],
            "rviz_display_names": self._rviz_display_names(),
        }


class _TopicAsPath:
    @staticmethod
    def from_pose(msg, frame_id):
        path = RosPath()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame_id
        if msg is not None:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose = msg.pose
            path.poses.append(pose)
        return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-forward-m", type=float, default=10.5)
    parser.add_argument("--dwell-sec", type=float, default=14.0)
    parser.add_argument("--extra-clearance-m", type=float, default=0.05)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    rospy.init_node("stage2_ego_visual_completeness_probe", anonymous=True)
    probe = Stage2EgoVisualCompletenessProbe(extra_clearance_m=args.extra_clearance_m)
    result = probe.run(goal_forward_m=args.goal_forward_m, dwell_sec=args.dwell_sec)
    if args.json:
        print(json.dumps(result, indent=2, ensure_ascii=False, sort_keys=True))
    else:
        print(result)
    if result.get("status") != "passed":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
