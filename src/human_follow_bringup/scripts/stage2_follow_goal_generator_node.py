#!/usr/bin/env python3
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import FollowState
from human_follow_msgs.msg import Target3D


def yaw_from_quaternion(x_value, y_value, z_value, w_value):
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(angle_rad):
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


class Stage2FollowGoalGeneratorNode:
    def __init__(self):
        self.target_world_topic = rospy.get_param("~target_world_topic", "/follow/fusion/target_world")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/lio/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/follow/stage2/goal")
        self.debug_path_topic = rospy.get_param("~debug_path_topic", "/follow/stage2/debug_goal_path")
        self.state_topic = rospy.get_param("~state_topic", "/follow/stage2/state")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.rate_hz = float(rospy.get_param("~rate_hz", 15.0))
        self.target_timeout_sec = float(rospy.get_param("~target_timeout_sec", 0.35))
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.6))
        self.follow_distance_m = float(rospy.get_param("~follow_distance_m", 3.5))
        self.lateral_bias_m = float(rospy.get_param("~lateral_bias_m", 0.0))
        self.vertical_bias_m = float(rospy.get_param("~vertical_bias_m", 0.0))
        self.min_goal_shift_m = float(rospy.get_param("~min_goal_shift_m", 0.08))
        self.face_target_yaw = bool(rospy.get_param("~face_target_yaw", True))
        self.search_timeout_sec = float(rospy.get_param("~search_timeout_sec", 2.0))
        self.lost_timeout_sec = float(rospy.get_param("~lost_timeout_sec", 5.0))
        self.search_mode = str(rospy.get_param("~search_mode", "target_orbit")).strip() or "target_orbit"
        self.search_yaw_rate_rps = float(rospy.get_param("~search_yaw_rate_rps", 0.25))
        self.search_sweep_span_deg = float(rospy.get_param("~search_sweep_span_deg", 0.0))
        self.search_sweep_pass_count = max(int(rospy.get_param("~search_sweep_pass_count", 0)), 0)
        self.search_return_to_center = bool(rospy.get_param("~search_return_to_center", False))
        self.search_goal_radius_m = float(rospy.get_param("~search_goal_radius_m", 1.0))
        self.search_observation_distance_m = float(rospy.get_param("~search_observation_distance_m", -1.0))
        self.publish_every_tick = bool(rospy.get_param("~publish_every_tick", True))
        self.occupancy_inflate_topic = rospy.get_param("~occupancy_inflate_topic", "/grid_map/occupancy_inflate")
        self.occupancy_timeout_sec = float(rospy.get_param("~occupancy_timeout_sec", 0.6))
        self.occupancy_resolution_m = max(float(rospy.get_param("~occupancy_resolution_m", 0.1)), 0.02)
        self.goal_obstacle_clearance_m = max(float(rospy.get_param("~goal_obstacle_clearance_m", 0.18)), 0.0)
        self.goal_projection_radius_step_m = max(float(rospy.get_param("~goal_projection_radius_step_m", 0.25)), 0.05)
        self.goal_projection_extra_radius_m = max(float(rospy.get_param("~goal_projection_extra_radius_m", 1.5)), 0.0)
        self.goal_projection_arc_deg = max(float(rospy.get_param("~goal_projection_arc_deg", 180.0)), 0.0)
        self.goal_projection_angle_step_deg = max(float(rospy.get_param("~goal_projection_angle_step_deg", 10.0)), 1.0)
        self.enable_obstacle_aware_goal_projection = bool(
            rospy.get_param("~enable_obstacle_aware_goal_projection", True)
        )
        self.require_initial_occupancy_before_planning = bool(
            rospy.get_param("~require_initial_occupancy_before_planning", False)
        )
        self.enable_blocked_path_detour = bool(rospy.get_param("~enable_blocked_path_detour", True))
        self.detour_lateral_step_m = max(float(rospy.get_param("~detour_lateral_step_m", 0.6)), 0.05)
        self.detour_max_lateral_m = max(float(rospy.get_param("~detour_max_lateral_m", 2.4)), 0.0)
        self.detour_segment_sample_step_m = max(float(rospy.get_param("~detour_segment_sample_step_m", 0.10)), 0.02)
        self.detour_clearance_probe_radius_m = max(
            float(rospy.get_param("~detour_clearance_probe_radius_m", 0.6)),
            self.occupancy_resolution_m,
        )
        self.detour_goal_max_distance_m = max(
            float(rospy.get_param("~detour_goal_max_distance_m", max(self.follow_distance_m, 3.5))),
            0.0,
        )
        self.detour_release_extra_clearance_m = max(
            float(rospy.get_param("~detour_release_extra_clearance_m", 0.18)),
            0.0,
        )
        self.enable_blocked_path_escape_goal = bool(rospy.get_param("~enable_blocked_path_escape_goal", True))
        self.blocked_path_escape_radius_step_m = max(
            float(rospy.get_param("~blocked_path_escape_radius_step_m", 0.25)),
            0.05,
        )
        self.blocked_path_escape_max_radius_m = max(
            float(
                rospy.get_param(
                    "~blocked_path_escape_max_radius_m",
                    max(self.detour_max_lateral_m, self.follow_distance_m),
                )
            ),
            self.blocked_path_escape_radius_step_m,
        )
        self.blocked_path_escape_arc_deg = max(
            float(rospy.get_param("~blocked_path_escape_arc_deg", 180.0)),
            0.0,
        )
        self.blocked_path_escape_angle_step_deg = max(
            float(rospy.get_param("~blocked_path_escape_angle_step_deg", 10.0)),
            1.0,
        )
        self.blocked_path_escape_clearance_m = max(
            float(rospy.get_param("~blocked_path_escape_clearance_m", self.goal_obstacle_clearance_m)),
            0.0,
        )

        self.last_valid_target = None
        self.last_valid_target_stamp = None
        self.last_odom = None
        self.last_odom_stamp = None
        self.last_goal = None
        self.last_goal_state = "idle"
        self.last_goal_yaw = 0.0
        self.last_seen_lateral_sign = 1.0
        self.search_anchor = None
        self.search_anchor_stamp = None
        self.last_goal_projection_note = "free"
        self.active_detour_lateral_offset = None
        self._occupancy_lock = threading.Lock()
        self._occupied_cells = set()
        self._occupancy_stamp = None

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10, latch=True)
        self.path_pub = rospy.Publisher(self.debug_path_topic, Path, queue_size=10)
        self.state_pub = rospy.Publisher(self.state_topic, FollowState, queue_size=10)

        self.target_sub = rospy.Subscriber(self.target_world_topic, Target3D, self._target_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=10)
        self.occupancy_sub = rospy.Subscriber(
            self.occupancy_inflate_topic,
            PointCloud2,
            self._occupancy_callback,
            queue_size=2,
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "stage2_follow_goal_generator ready: target_world=%s odom=%s goal=%s follow_distance=%.2f search_mode=%s search_radius=%.2f search_observation_distance=%.2f occupancy=%s projection=%s",
            self.target_world_topic,
            self.odom_topic,
            self.goal_topic,
            self.follow_distance_m,
            self.search_mode,
            self.search_goal_radius_m,
            self.search_observation_distance_m,
            self.occupancy_inflate_topic,
            self.enable_obstacle_aware_goal_projection,
        )

    def _state_enum(self, state_name):
        if state_name == "target_acquired":
            return FollowState.TARGET_ACQUIRED
        if state_name == "follow":
            return FollowState.FOLLOW
        if state_name == "search":
            return FollowState.SEARCH
        if state_name == "lost":
            return FollowState.LOST
        return FollowState.IDLE

    def _publish_state(self, stamp, state_name, detail):
        msg = FollowState()
        msg.header.stamp = stamp
        msg.state = self._state_enum(state_name)
        msg.state_name = state_name
        msg.detail = detail
        try:
            self.state_pub.publish(msg)
        except rospy.ROSException:
            return

    def _target_callback(self, msg):
        if not msg.valid:
            return
        now = rospy.Time.now()
        self.last_valid_target = msg
        self.last_valid_target_stamp = now
        relative_body = self._world_target_to_body(msg)
        if relative_body is not None and abs(relative_body[1]) > 1e-3:
            self.last_seen_lateral_sign = 1.0 if relative_body[1] >= 0.0 else -1.0

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_stamp = rospy.Time.now()

    def _occupancy_callback(self, msg):
        occupied_cells = set()
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            occupied_cells.add(self._cell_key(point[0], point[1], point[2]))
        with self._occupancy_lock:
            self._occupied_cells = occupied_cells
            self._occupancy_stamp = rospy.Time.now()

    def _target_is_fresh(self, now):
        return (
            self.last_valid_target is not None
            and self.last_valid_target_stamp is not None
            and (now - self.last_valid_target_stamp).to_sec() <= self.target_timeout_sec
        )

    def _odom_is_fresh(self, now):
        return (
            self.last_odom is not None
            and self.last_odom_stamp is not None
            and (now - self.last_odom_stamp).to_sec() <= self.odom_timeout_sec
        )

    def _occupancy_is_fresh(self, now):
        with self._occupancy_lock:
            if self._occupancy_stamp is None:
                return False
            return (now - self._occupancy_stamp).to_sec() <= self.occupancy_timeout_sec

    def _has_seen_any_occupancy(self):
        with self._occupancy_lock:
            return self._occupancy_stamp is not None

    def _occupancy_cells_snapshot(self, now):
        if not self.enable_obstacle_aware_goal_projection:
            return None
        with self._occupancy_lock:
            occupancy_stamp = self._occupancy_stamp
            occupied_cells = self._occupied_cells
        if occupancy_stamp is None:
            return None
        if (now - occupancy_stamp).to_sec() > self.occupancy_timeout_sec:
            return None
        return occupied_cells

    def _cell_key(self, x_value, y_value, z_value):
        resolution = self.occupancy_resolution_m
        return (
            int(round(float(x_value) / resolution)),
            int(round(float(y_value) / resolution)),
            int(round(float(z_value) / resolution)),
        )

    def _point_is_in_inflated_obstacle(
        self,
        x_value,
        y_value,
        z_value,
        now,
        clearance_m=None,
        occupied_cells=None,
    ):
        if not self.enable_obstacle_aware_goal_projection:
            return False
        occupied_cells = self._occupancy_cells_snapshot(now) if occupied_cells is None else occupied_cells
        if occupied_cells is None:
            return False

        clearance = self.goal_obstacle_clearance_m if clearance_m is None else max(float(clearance_m), 0.0)
        radius_cells = int(math.ceil(clearance / self.occupancy_resolution_m))
        center_key = self._cell_key(x_value, y_value, z_value)
        for dx_value in range(-radius_cells, radius_cells + 1):
            for dy_value in range(-radius_cells, radius_cells + 1):
                for dz_value in range(-radius_cells, radius_cells + 1):
                    if dx_value * dx_value + dy_value * dy_value + dz_value * dz_value > radius_cells * radius_cells:
                        continue
                    neighbor_key = (
                        center_key[0] + dx_value,
                        center_key[1] + dy_value,
                        center_key[2] + dz_value,
                    )
                    if neighbor_key in occupied_cells:
                        return True
        return False

    def _segment_hits_inflated_obstacle(
        self,
        start_xyz,
        end_xyz,
        now,
        clearance_m=None,
        occupied_cells=None,
        allow_occupied_start=False,
    ):
        if not self.enable_obstacle_aware_goal_projection:
            return False
        occupied_cells = self._occupancy_cells_snapshot(now) if occupied_cells is None else occupied_cells
        if occupied_cells is None:
            return False

        start_x, start_y, start_z = start_xyz
        end_x, end_y, end_z = end_xyz
        distance_m = math.sqrt(
            (end_x - start_x) * (end_x - start_x)
            + (end_y - start_y) * (end_y - start_y)
            + (end_z - start_z) * (end_z - start_z)
        )
        if distance_m <= 1e-6:
            return self._point_is_in_inflated_obstacle(
                end_x,
                end_y,
                end_z,
                now,
                clearance_m=clearance_m,
                occupied_cells=occupied_cells,
            )

        start_in_obstacle = self._point_is_in_inflated_obstacle(
            start_x,
            start_y,
            start_z,
            now,
            clearance_m=clearance_m,
            occupied_cells=occupied_cells,
        )
        if start_in_obstacle and not allow_occupied_start:
            return True

        # If the current vehicle pose is already inside an inflated safety shell,
        # an escape segment must be allowed to leave that shell. Reject only if it
        # re-enters after first reaching free space, or if the endpoint is unsafe.
        has_left_start_obstacle = not start_in_obstacle
        sample_count = max(int(math.ceil(distance_m / self.detour_segment_sample_step_m)), 2)
        for sample_index in range(1, sample_count + 1):
            ratio = float(sample_index) / float(sample_count)
            sample_x = start_x + (end_x - start_x) * ratio
            sample_y = start_y + (end_y - start_y) * ratio
            sample_z = start_z + (end_z - start_z) * ratio
            sample_in_obstacle = self._point_is_in_inflated_obstacle(
                sample_x,
                sample_y,
                sample_z,
                now,
                clearance_m=clearance_m,
                occupied_cells=occupied_cells,
            )
            if sample_in_obstacle:
                if allow_occupied_start and not has_left_start_obstacle:
                    continue
                return True
            has_left_start_obstacle = True
        return False

    def _point_clearance_from_occupancy(
        self,
        x_value,
        y_value,
        z_value,
        now,
        probe_radius_m=None,
        occupied_cells=None,
    ):
        if not self.enable_obstacle_aware_goal_projection:
            return None
        occupied_cells = self._occupancy_cells_snapshot(now) if occupied_cells is None else occupied_cells
        if occupied_cells is None:
            return None

        search_radius_m = max(
            self.detour_clearance_probe_radius_m if probe_radius_m is None else float(probe_radius_m),
            self.occupancy_resolution_m,
        )
        radius_cells = max(int(math.ceil(search_radius_m / self.occupancy_resolution_m)), 1)
        center_key = self._cell_key(x_value, y_value, z_value)
        nearest_distance_m = None
        for dx_value in range(-radius_cells, radius_cells + 1):
            for dy_value in range(-radius_cells, radius_cells + 1):
                for dz_value in range(-radius_cells, radius_cells + 1):
                    neighbor_key = (
                        center_key[0] + dx_value,
                        center_key[1] + dy_value,
                        center_key[2] + dz_value,
                    )
                    if neighbor_key not in occupied_cells:
                        continue
                    distance_m = math.sqrt(
                        float(dx_value * dx_value + dy_value * dy_value + dz_value * dz_value)
                    ) * self.occupancy_resolution_m
                    if nearest_distance_m is None or distance_m < nearest_distance_m:
                        nearest_distance_m = distance_m
        if nearest_distance_m is None:
            return search_radius_m + self.occupancy_resolution_m
        return nearest_distance_m

    def _segment_clearance_score(self, start_xyz, end_xyz, now, probe_radius_m=None, occupied_cells=None):
        if not self.enable_obstacle_aware_goal_projection:
            return None
        occupied_cells = self._occupancy_cells_snapshot(now) if occupied_cells is None else occupied_cells
        if occupied_cells is None:
            return None

        start_x, start_y, start_z = start_xyz
        end_x, end_y, end_z = end_xyz
        distance_m = math.sqrt(
            (end_x - start_x) * (end_x - start_x)
            + (end_y - start_y) * (end_y - start_y)
            + (end_z - start_z) * (end_z - start_z)
        )
        if distance_m <= 1e-6:
            return self._point_clearance_from_occupancy(
                end_x,
                end_y,
                end_z,
                now,
                probe_radius_m=probe_radius_m,
                occupied_cells=occupied_cells,
            )

        sample_count = max(int(math.ceil(distance_m / self.detour_segment_sample_step_m)), 2)
        min_clearance_m = None
        for sample_index in range(sample_count + 1):
            ratio = float(sample_index) / float(sample_count)
            sample_x = start_x + (end_x - start_x) * ratio
            sample_y = start_y + (end_y - start_y) * ratio
            sample_z = start_z + (end_z - start_z) * ratio
            sample_clearance_m = self._point_clearance_from_occupancy(
                sample_x,
                sample_y,
                sample_z,
                now,
                probe_radius_m=probe_radius_m,
                occupied_cells=occupied_cells,
            )
            if sample_clearance_m is None:
                return None
            if min_clearance_m is None or sample_clearance_m < min_clearance_m:
                min_clearance_m = sample_clearance_m
        return min_clearance_m

    def _candidate_clearance_score(self, start_xyz, end_xyz, now, probe_radius_m=None, occupied_cells=None):
        segment_clearance_m = self._segment_clearance_score(
            start_xyz,
            end_xyz,
            now,
            probe_radius_m=probe_radius_m,
            occupied_cells=occupied_cells,
        )
        if segment_clearance_m is None:
            return None
        point_clearance_m = self._point_clearance_from_occupancy(
            end_xyz[0],
            end_xyz[1],
            end_xyz[2],
            now,
            probe_radius_m=probe_radius_m,
            occupied_cells=occupied_cells,
        )
        if point_clearance_m is None:
            return None
        return min(segment_clearance_m, point_clearance_m)

    def _limit_goal_distance_from_vehicle(self, goal_tuple, target_x=None, target_y=None):
        if self.last_odom is None or self.detour_goal_max_distance_m <= 0.0:
            return goal_tuple

        vehicle_x = float(self.last_odom.pose.pose.position.x)
        vehicle_y = float(self.last_odom.pose.pose.position.y)
        goal_x, goal_y, goal_z, goal_yaw = goal_tuple
        delta_x = goal_x - vehicle_x
        delta_y = goal_y - vehicle_y
        distance_m = math.hypot(delta_x, delta_y)
        if distance_m <= self.detour_goal_max_distance_m + 1e-6:
            return goal_tuple

        scale = self.detour_goal_max_distance_m / max(distance_m, 1e-6)
        limited_x = vehicle_x + delta_x * scale
        limited_y = vehicle_y + delta_y * scale
        limited_yaw = goal_yaw
        if self.face_target_yaw and target_x is not None and target_y is not None:
            limited_yaw = math.atan2(target_y - limited_y, target_x - limited_x)
        return (limited_x, limited_y, goal_z, limited_yaw)

    def _make_detour_candidate(
        self,
        lateral_offset,
        target_x,
        target_y,
        target_z,
        goal_z,
        goal_yaw,
        now,
        occupied_cells=None,
    ):
        delta_x = float(self.last_odom.pose.pose.position.x) - target_x
        delta_y = float(self.last_odom.pose.pose.position.y) - target_y
        distance = math.hypot(delta_x, delta_y)
        if distance < 1e-6:
            yaw_rad = self._current_odom_yaw()
            unit_x = math.cos(yaw_rad)
            unit_y = math.sin(yaw_rad)
        else:
            unit_x = delta_x / distance
            unit_y = delta_y / distance
        left_x = -unit_y
        left_y = unit_x

        candidate_x = target_x + unit_x * self.follow_distance_m + left_x * lateral_offset
        candidate_y = target_y + unit_y * self.follow_distance_m + left_y * lateral_offset
        if self.face_target_yaw:
            candidate_yaw = math.atan2(target_y - candidate_y, target_x - candidate_x)
        else:
            candidate_yaw = goal_yaw
        projected_candidate = self._project_goal_out_of_obstacle(
            target_x,
            target_y,
            target_z,
            (candidate_x, candidate_y, goal_z, candidate_yaw),
            now,
            "follow_detour",
            occupied_cells=occupied_cells,
        )
        return self._limit_goal_distance_from_vehicle(
            projected_candidate,
            target_x=target_x,
            target_y=target_y,
        )

    def _projection_angle_offsets_rad(self):
        max_step_index = int(math.floor(self.goal_projection_arc_deg / self.goal_projection_angle_step_deg))
        offsets = [0.0]
        for step_index in range(1, max_step_index + 1):
            offset_rad = math.radians(step_index * self.goal_projection_angle_step_deg)
            offsets.append(offset_rad)
            offsets.append(-offset_rad)
        return offsets

    def _projection_radius_candidates(self, preferred_radius_m):
        base_radius = max(preferred_radius_m, self.goal_projection_radius_step_m)
        inner_radii = []
        outer_radii = []
        radius_offset = self.goal_projection_radius_step_m
        while radius_offset <= self.goal_projection_extra_radius_m + 1e-6:
            inner_radius = preferred_radius_m - radius_offset
            if inner_radius >= self.goal_projection_radius_step_m:
                inner_radii.append(inner_radius)
            outer_radii.append(max(preferred_radius_m + radius_offset, self.goal_projection_radius_step_m))
            radius_offset += self.goal_projection_radius_step_m
        return inner_radii + [base_radius] + outer_radii

    def _project_goal_out_of_obstacle(
        self,
        reference_x,
        reference_y,
        reference_z,
        goal_tuple,
        now,
        detail_label,
        occupied_cells=None,
    ):
        goal_x, goal_y, goal_z, goal_yaw = goal_tuple
        if not self._point_is_in_inflated_obstacle(
            goal_x,
            goal_y,
            goal_z,
            now,
            occupied_cells=occupied_cells,
        ):
            self.last_goal_projection_note = "free"
            return goal_tuple

        preferred_radius = max(math.hypot(goal_x - reference_x, goal_y - reference_y), self.goal_projection_radius_step_m)
        preferred_bearing = math.atan2(goal_y - reference_y, goal_x - reference_x)
        best_candidate = None
        best_candidate_note = None
        best_candidate_score = None
        best_radius_delta = None
        best_angle_abs_deg = None

        for radius_value in self._projection_radius_candidates(preferred_radius):
            for angle_offset_rad in self._projection_angle_offsets_rad():
                candidate_x = reference_x + math.cos(preferred_bearing + angle_offset_rad) * radius_value
                candidate_y = reference_y + math.sin(preferred_bearing + angle_offset_rad) * radius_value
                if self._point_is_in_inflated_obstacle(
                    candidate_x,
                    candidate_y,
                    goal_z,
                    now,
                    occupied_cells=occupied_cells,
                ):
                    continue
                candidate_score = self._point_clearance_from_occupancy(
                    candidate_x,
                    candidate_y,
                    goal_z,
                    now,
                    occupied_cells=occupied_cells,
                )
                if candidate_score is None:
                    candidate_score = 0.0
                radius_delta = abs(radius_value - preferred_radius)
                angle_abs_deg = abs(math.degrees(angle_offset_rad))
                if self.face_target_yaw:
                    candidate_yaw = math.atan2(reference_y - candidate_y, reference_x - candidate_x)
                else:
                    candidate_yaw = goal_yaw
                if (
                    best_candidate is None
                    or candidate_score > best_candidate_score + 1e-6
                    or (
                        abs(candidate_score - best_candidate_score) <= 1e-6
                        and radius_delta < best_radius_delta - 1e-6
                    )
                    or (
                        abs(candidate_score - best_candidate_score) <= 1e-6
                        and abs(radius_delta - best_radius_delta) <= 1e-6
                        and angle_abs_deg < best_angle_abs_deg - 1e-6
                    )
                ):
                    best_candidate = (candidate_x, candidate_y, goal_z, candidate_yaw)
                    best_candidate_note = (
                        "%s_projected radius=%.2f angle_deg=%.1f"
                        % (detail_label, radius_value, math.degrees(angle_offset_rad))
                    )
                    best_candidate_score = candidate_score
                    best_radius_delta = radius_delta
                    best_angle_abs_deg = angle_abs_deg

        if best_candidate is not None:
            self.last_goal_projection_note = best_candidate_note
            return best_candidate

        self.last_goal_projection_note = "%s_blocked_hold" % detail_label
        rospy.logwarn_throttle(
            1.0,
            "stage2_follow_goal_generator could not find collision-free %s goal near (%.2f, %.2f, %.2f); holding position",
            detail_label,
            goal_x,
            goal_y,
            goal_z,
        )
        return self._compute_hold_goal()

    def _candidate_is_safe_after_limit(
        self,
        candidate_goal,
        vehicle_xyz,
        now,
        clearance_m=None,
        occupied_cells=None,
        allow_occupied_start=False,
    ):
        if candidate_goal is None:
            return False
        goal_xyz = candidate_goal[:3]
        if self._point_is_in_inflated_obstacle(
            goal_xyz[0],
            goal_xyz[1],
            goal_xyz[2],
            now,
            clearance_m=clearance_m,
            occupied_cells=occupied_cells,
        ):
            return False
        if self._segment_hits_inflated_obstacle(
            vehicle_xyz,
            goal_xyz,
            now,
            clearance_m=clearance_m,
            occupied_cells=occupied_cells,
            allow_occupied_start=allow_occupied_start,
        ):
            return False
        return True

    def _blocked_path_escape_angle_offsets_rad(self):
        max_step_index = int(
            math.floor(self.blocked_path_escape_arc_deg / self.blocked_path_escape_angle_step_deg)
        )
        offsets = [0.0]
        for step_index in range(1, max_step_index + 1):
            offset_rad = math.radians(step_index * self.blocked_path_escape_angle_step_deg)
            offsets.append(offset_rad)
            offsets.append(-offset_rad)
        return offsets

    def _blocked_path_escape_radius_candidates(self):
        radii = []
        radius_value = self.blocked_path_escape_radius_step_m
        while radius_value <= self.blocked_path_escape_max_radius_m + 1e-6:
            radii.append(max(radius_value, self.blocked_path_escape_radius_step_m))
            radius_value += self.blocked_path_escape_radius_step_m
        if not radii:
            radii.append(self.blocked_path_escape_radius_step_m)
        return radii

    def _find_blocked_path_escape_goal(self, target_x, target_y, goal_z, now, occupied_cells=None):
        if not self.enable_blocked_path_escape_goal:
            return None
        if self.last_odom is None:
            return None

        vehicle_x = float(self.last_odom.pose.pose.position.x)
        vehicle_y = float(self.last_odom.pose.pose.position.y)
        vehicle_z = float(self.last_odom.pose.pose.position.z)
        vehicle_xyz = (vehicle_x, vehicle_y, vehicle_z)
        vehicle_hits_escape_clearance = self._point_is_in_inflated_obstacle(
            vehicle_x,
            vehicle_y,
            vehicle_z,
            now,
            clearance_m=self.blocked_path_escape_clearance_m,
            occupied_cells=occupied_cells,
        )

        forward_x = target_x - vehicle_x
        forward_y = target_y - vehicle_y
        forward_norm = math.hypot(forward_x, forward_y)
        if forward_norm < 1e-6:
            preferred_bearing = self._current_odom_yaw()
        else:
            preferred_bearing = math.atan2(forward_y, forward_x)

        best_escape_goal = None
        best_escape_score = None
        best_escape_radius = None
        best_escape_angle_abs = None
        for radius_value in self._blocked_path_escape_radius_candidates():
            for angle_offset_rad in self._blocked_path_escape_angle_offsets_rad():
                candidate_x = vehicle_x + math.cos(preferred_bearing + angle_offset_rad) * radius_value
                candidate_y = vehicle_y + math.sin(preferred_bearing + angle_offset_rad) * radius_value
                candidate_xyz = (candidate_x, candidate_y, goal_z)
                if self._point_is_in_inflated_obstacle(
                    candidate_x,
                    candidate_y,
                    goal_z,
                    now,
                    clearance_m=self.blocked_path_escape_clearance_m,
                    occupied_cells=occupied_cells,
                ):
                    continue
                if self._segment_hits_inflated_obstacle(
                    vehicle_xyz,
                    candidate_xyz,
                    now,
                    clearance_m=self.blocked_path_escape_clearance_m,
                    occupied_cells=occupied_cells,
                    allow_occupied_start=vehicle_hits_escape_clearance,
                ):
                    continue
                if self.face_target_yaw:
                    candidate_yaw = math.atan2(target_y - candidate_y, target_x - candidate_x)
                else:
                    candidate_yaw = self.last_goal_yaw
                candidate_score = self._candidate_clearance_score(
                    vehicle_xyz,
                    candidate_xyz,
                    now,
                    occupied_cells=occupied_cells,
                )
                if candidate_score is None:
                    candidate_score = 0.0
                angle_abs_deg = abs(math.degrees(angle_offset_rad))
                if (
                    best_escape_goal is None
                    or candidate_score > best_escape_score + 1e-6
                    or (
                        abs(candidate_score - best_escape_score) <= 1e-6
                        and radius_value < best_escape_radius - 1e-6
                    )
                    or (
                        abs(candidate_score - best_escape_score) <= 1e-6
                        and abs(radius_value - best_escape_radius) <= 1e-6
                        and angle_abs_deg < best_escape_angle_abs - 1e-6
                    )
                ):
                    best_escape_goal = (
                        candidate_x,
                        candidate_y,
                        goal_z,
                        candidate_yaw,
                        radius_value,
                        math.degrees(angle_offset_rad),
                    )
                    best_escape_score = candidate_score
                    best_escape_radius = radius_value
                    best_escape_angle_abs = angle_abs_deg
        return best_escape_goal

    def _current_odom_yaw(self):
        if self.last_odom is None:
            return 0.0
        return yaw_from_quaternion(
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
            self.last_odom.pose.pose.orientation.w,
        )

    def _world_target_to_body(self, target_msg):
        if self.last_odom is None:
            return None
        vehicle_x = float(self.last_odom.pose.pose.position.x)
        vehicle_y = float(self.last_odom.pose.pose.position.y)
        yaw_rad = self._current_odom_yaw()
        delta_x = float(target_msg.position.x) - vehicle_x
        delta_y = float(target_msg.position.y) - vehicle_y
        body_x = math.cos(yaw_rad) * delta_x + math.sin(yaw_rad) * delta_y
        body_y = -math.sin(yaw_rad) * delta_x + math.cos(yaw_rad) * delta_y
        body_z = float(target_msg.position.z) - float(self.last_odom.pose.pose.position.z)
        return body_x, body_y, body_z

    def _goal_tuple_xyz(self, goal_tuple):
        return (
            float(goal_tuple[0]),
            float(goal_tuple[1]),
            float(goal_tuple[2]),
        )

    def _current_state(self, now):
        if not self._odom_is_fresh(now):
            return "idle"
        if self._target_is_fresh(now):
            return "follow"
        if self.last_valid_target is None or self.last_valid_target_stamp is None:
            return "idle"
        lost_age_sec = max(0.0, (now - self.last_valid_target_stamp).to_sec() - self.target_timeout_sec)
        if lost_age_sec <= self.search_timeout_sec:
            return "search"
        if lost_age_sec <= self.search_timeout_sec + self.lost_timeout_sec:
            return "lost"
        return "idle"

    def _compute_follow_goal(self):
        target_x = float(self.last_valid_target.position.x)
        target_y = float(self.last_valid_target.position.y)
        target_z = float(self.last_valid_target.position.z)
        vehicle_x = float(self.last_odom.pose.pose.position.x)
        vehicle_y = float(self.last_odom.pose.pose.position.y)
        vehicle_z = float(self.last_odom.pose.pose.position.z)

        delta_x = vehicle_x - target_x
        delta_y = vehicle_y - target_y
        distance = math.hypot(delta_x, delta_y)
        if distance < 1e-6:
            yaw_rad = self._current_odom_yaw()
            unit_x = math.cos(yaw_rad)
            unit_y = math.sin(yaw_rad)
        else:
            unit_x = delta_x / distance
            unit_y = delta_y / distance

        left_x = -unit_y
        left_y = unit_x

        goal_x = target_x + unit_x * self.follow_distance_m + left_x * self.lateral_bias_m
        goal_y = target_y + unit_y * self.follow_distance_m + left_y * self.lateral_bias_m
        goal_z = max(vehicle_z, target_z + self.vertical_bias_m)

        yaw_rad = self.last_goal_yaw
        if self.face_target_yaw:
            yaw_rad = math.atan2(target_y - goal_y, target_x - goal_x)
        return goal_x, goal_y, goal_z, yaw_rad, target_x, target_y, target_z

    def _detour_lateral_offsets(self):
        if self.detour_max_lateral_m < self.detour_lateral_step_m:
            return []

        sign_order = [1.0, -1.0]
        if self.last_seen_lateral_sign < 0.0:
            sign_order = [-1.0, 1.0]

        offsets = []
        offset_value = self.detour_lateral_step_m
        while offset_value <= self.detour_max_lateral_m + 1e-6:
            for sign_value in sign_order:
                offsets.append(sign_value * offset_value)
            offset_value += self.detour_lateral_step_m
        return offsets

    def _detour_tie_break_key(self, lateral_offset):
        if self.active_detour_lateral_offset is None:
            return (0, abs(float(lateral_offset)))

        active_offset = float(self.active_detour_lateral_offset)
        lateral_offset = float(lateral_offset)
        same_active = abs(lateral_offset - active_offset) <= 1e-6
        same_side = lateral_offset * active_offset > 0.0
        return (
            0 if same_active else (1 if same_side else 2),
            abs(abs(lateral_offset) - abs(active_offset)),
            abs(lateral_offset),
        )

    def _resolve_follow_goal(self, follow_goal, now, occupied_cells=None):
        goal_x, goal_y, goal_z, goal_yaw, target_x, target_y, target_z = follow_goal
        vehicle_xyz = (
            float(self.last_odom.pose.pose.position.x),
            float(self.last_odom.pose.pose.position.y),
            float(self.last_odom.pose.pose.position.z),
        )
        nominal_goal = self._project_goal_out_of_obstacle(
            target_x,
            target_y,
            target_z,
            (goal_x, goal_y, goal_z, goal_yaw),
            now,
            "follow_goal",
            occupied_cells=occupied_cells,
        )

        if not self.enable_blocked_path_detour:
            return nominal_goal
        release_clearance_m = self.goal_obstacle_clearance_m + self.detour_release_extra_clearance_m
        nominal_hits_base_clearance = self._segment_hits_inflated_obstacle(
            vehicle_xyz,
            nominal_goal[:3],
            now,
            occupied_cells=occupied_cells,
        )
        nominal_hits_release_clearance = self._segment_hits_inflated_obstacle(
            vehicle_xyz,
            nominal_goal[:3],
            now,
            clearance_m=release_clearance_m,
            occupied_cells=occupied_cells,
        )
        vehicle_still_tight = self._point_is_in_inflated_obstacle(
            vehicle_xyz[0],
            vehicle_xyz[1],
            vehicle_xyz[2],
            now,
            clearance_m=release_clearance_m,
            occupied_cells=occupied_cells,
        )
        if not nominal_hits_base_clearance and not nominal_hits_release_clearance and not vehicle_still_tight:
            self.active_detour_lateral_offset = None
            return nominal_goal

        if (
            not nominal_hits_base_clearance
            and not nominal_hits_release_clearance
            and self.active_detour_lateral_offset is None
        ):
            return nominal_goal

        if not nominal_hits_base_clearance and self.active_detour_lateral_offset is None:
            self.last_goal_projection_note = "follow_detour_release_margin"

        best_detour_goal = None
        best_detour_lateral_offset = None
        best_detour_score = None
        best_detour_tie_key = None
        for lateral_offset in self._detour_lateral_offsets():
            projected_candidate = self._make_detour_candidate(
                lateral_offset,
                target_x,
                target_y,
                target_z,
                goal_z,
                goal_yaw,
                now,
                occupied_cells=occupied_cells,
            )
            if self._segment_hits_inflated_obstacle(
                vehicle_xyz,
                projected_candidate[:3],
                now,
                occupied_cells=occupied_cells,
            ):
                continue
            candidate_score = self._candidate_clearance_score(
                vehicle_xyz,
                projected_candidate[:3],
                now,
                occupied_cells=occupied_cells,
            )
            if candidate_score is None:
                candidate_score = 0.0
            candidate_tie_key = self._detour_tie_break_key(lateral_offset)
            if (
                best_detour_goal is None
                or candidate_score > best_detour_score + 1e-6
                or (
                    abs(candidate_score - best_detour_score) <= 1e-6
                    and candidate_tie_key < best_detour_tie_key
                )
            ):
                best_detour_goal = projected_candidate
                best_detour_lateral_offset = lateral_offset
                best_detour_score = candidate_score
                best_detour_tie_key = candidate_tie_key

        if best_detour_goal is not None:
            if not self._candidate_is_safe_after_limit(
                best_detour_goal,
                vehicle_xyz,
                now,
                clearance_m=release_clearance_m,
                occupied_cells=occupied_cells,
                allow_occupied_start=vehicle_still_tight,
            ):
                best_detour_goal = None
                best_detour_lateral_offset = None
                best_detour_score = None
            else:
                previous_detour_lateral_offset = self.active_detour_lateral_offset
                self.active_detour_lateral_offset = best_detour_lateral_offset
                note_suffix = ""
                if (
                    previous_detour_lateral_offset is not None
                    and abs(best_detour_lateral_offset - previous_detour_lateral_offset) <= 1e-6
                ):
                    note_suffix = " hold"
                self.last_goal_projection_note = (
                    "follow_detour lateral=%.2f clearance=%.2f%s"
                    % (best_detour_lateral_offset, best_detour_score, note_suffix)
                )
                return best_detour_goal

        self.active_detour_lateral_offset = None
        escape_goal = self._find_blocked_path_escape_goal(
            target_x,
            target_y,
            goal_z,
            now,
            occupied_cells=occupied_cells,
        )
        if escape_goal is not None:
            (
                escape_x,
                escape_y,
                escape_z,
                escape_yaw,
                escape_radius_m,
                escape_angle_deg,
            ) = escape_goal
            bounded_escape_goal = (escape_x, escape_y, escape_z, escape_yaw)
            if not self._candidate_is_safe_after_limit(
                bounded_escape_goal,
                vehicle_xyz,
                now,
                clearance_m=self.blocked_path_escape_clearance_m,
                occupied_cells=occupied_cells,
                allow_occupied_start=vehicle_still_tight,
            ):
                self.last_goal_projection_note = "follow_path_blocked_hold"
                return self._compute_hold_goal()
            self.last_goal_projection_note = (
                "follow_path_blocked_escape radius=%.2f angle_deg=%.1f"
                % (escape_radius_m, escape_angle_deg)
            )
            return escape_x, escape_y, escape_z, escape_yaw

        self.last_goal_projection_note = "follow_path_blocked_hold"
        return self._compute_hold_goal()

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

    def _clear_search_anchor(self):
        self.search_anchor = None
        self.search_anchor_stamp = None

    def _start_search_anchor(self, now):
        if self.last_odom is None:
            return
        if self.search_mode == "target_orbit" and self.last_valid_target is not None:
            vehicle_x = float(self.last_odom.pose.pose.position.x)
            vehicle_y = float(self.last_odom.pose.pose.position.y)
            target_x = float(self.last_valid_target.position.x)
            target_y = float(self.last_valid_target.position.y)
            entry_bearing = math.atan2(vehicle_y - target_y, vehicle_x - target_x)
            self.search_anchor = (
                target_x,
                target_y,
                float(self.last_valid_target.position.z),
                entry_bearing,
            )
        else:
            self.search_anchor = (
                float(self.last_odom.pose.pose.position.x),
                float(self.last_odom.pose.pose.position.y),
                float(self.last_odom.pose.pose.position.z),
                self._current_odom_yaw(),
            )
        self.search_anchor_stamp = now

    def _search_angle_offset(self, now):
        if self.search_anchor_stamp is None:
            return 0.0
        elapsed_sec = max(0.0, (now - self.search_anchor_stamp).to_sec())
        direction = 1.0 if self.last_seen_lateral_sign >= 0.0 else -1.0
        base_rate = abs(self.search_yaw_rate_rps)

        if not self._search_sweep_enabled():
            return direction * base_rate * elapsed_sec

        angle_offset = 0.0
        remaining = elapsed_sec
        for duration_sec in self._search_segment_durations_sec():
            segment_time = min(remaining, duration_sec)
            angle_offset += direction * base_rate * segment_time
            if remaining <= duration_sec:
                return angle_offset
            remaining -= duration_sec
            direction *= -1.0
        return angle_offset

    def _compute_search_goal(self, now):
        if self.search_anchor is None:
            self._start_search_anchor(now)
        if self.search_anchor is None:
            return self._compute_hold_goal()

        anchor_x, anchor_y, anchor_z, anchor_yaw = self.search_anchor
        search_heading = wrap_pi(anchor_yaw + self._search_angle_offset(now))
        if self.search_mode == "target_orbit":
            observation_distance = self.search_observation_distance_m
            if observation_distance <= 0.0:
                observation_distance = max(self.follow_distance_m, self.search_goal_radius_m)
            goal_x = anchor_x + math.cos(search_heading) * observation_distance
            goal_y = anchor_y + math.sin(search_heading) * observation_distance
            goal_yaw = math.atan2(anchor_y - goal_y, anchor_x - goal_x)
            return goal_x, goal_y, anchor_z, goal_yaw, anchor_x, anchor_y, anchor_z

        goal_x = anchor_x + math.cos(search_heading) * self.search_goal_radius_m
        goal_y = anchor_y + math.sin(search_heading) * self.search_goal_radius_m
        return goal_x, goal_y, anchor_z, search_heading, anchor_x, anchor_y, anchor_z

    def _compute_hold_goal(self):
        if self.last_odom is None:
            return None
        return (
            float(self.last_odom.pose.pose.position.x),
            float(self.last_odom.pose.pose.position.y),
            float(self.last_odom.pose.pose.position.z),
            self._current_odom_yaw(),
        )

    def _should_publish_goal(self, goal_tuple, state_name):
        if self.publish_every_tick:
            return True
        if self.last_goal_state != state_name:
            return True
        if self.last_goal is None:
            return True
        dx = goal_tuple[0] - self.last_goal[0]
        dy = goal_tuple[1] - self.last_goal[1]
        dz = goal_tuple[2] - self.last_goal[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) >= self.min_goal_shift_m

    def _make_pose(self, stamp, goal_tuple):
        goal_x, goal_y, goal_z, yaw_rad = goal_tuple
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame_id
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = goal_z
        msg.pose.orientation.z = math.sin(0.5 * yaw_rad)
        msg.pose.orientation.w = math.cos(0.5 * yaw_rad)
        return msg

    def _reference_xyz_for_debug(self, state_name, goal_tuple):
        if state_name == "follow" and self.last_valid_target is not None:
            return (
                float(self.last_valid_target.position.x),
                float(self.last_valid_target.position.y),
                float(self.last_valid_target.position.z),
            )
        if state_name == "search" and self.search_anchor is not None:
            return (
                float(self.search_anchor[0]),
                float(self.search_anchor[1]),
                float(self.search_anchor[2]),
            )
        if self.last_valid_target is not None:
            return (
                float(self.last_valid_target.position.x),
                float(self.last_valid_target.position.y),
                float(self.last_valid_target.position.z),
            )
        return (goal_tuple[0], goal_tuple[1], goal_tuple[2])

    def _publish_debug_path(self, stamp, goal_tuple, state_name):
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.world_frame_id
        reference_xyz = self._reference_xyz_for_debug(state_name, goal_tuple)

        for point_xyz in (
            (
                float(self.last_odom.pose.pose.position.x),
                float(self.last_odom.pose.pose.position.y),
                float(self.last_odom.pose.pose.position.z),
            ),
            (goal_tuple[0], goal_tuple[1], goal_tuple[2]),
            reference_xyz,
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
        now = rospy.Time.now()
        state_name = self._current_state(now)
        goal_tuple = None
        detail = ""
        occupancy_cells = self._occupancy_cells_snapshot(now)

        if state_name == "follow":
            self._clear_search_anchor()
            if (
                self.require_initial_occupancy_before_planning
                and self.enable_obstacle_aware_goal_projection
                and not self._has_seen_any_occupancy()
            ):
                goal_tuple = self._compute_hold_goal()
                detail = "waiting_initial_occupancy"
            else:
                follow_goal = self._compute_follow_goal()
                goal_tuple = self._resolve_follow_goal(
                    follow_goal,
                    now,
                    occupied_cells=occupancy_cells,
                )
                detail = "rolling_goal"
        elif state_name == "search":
            if self.last_goal_state != "search":
                self._start_search_anchor(now)
            if (
                self.require_initial_occupancy_before_planning
                and self.enable_obstacle_aware_goal_projection
                and not self._has_seen_any_occupancy()
            ):
                goal_tuple = self._compute_hold_goal()
                detail = "waiting_initial_occupancy"
            else:
                search_goal = self._compute_search_goal(now)
                goal_tuple = self._project_goal_out_of_obstacle(
                    search_goal[4],
                    search_goal[5],
                    search_goal[6],
                    search_goal[:4],
                    now,
                    "search_goal",
                    occupied_cells=occupancy_cells,
                )
                detail = "mode=%s" % self.search_mode
        elif state_name == "lost":
            goal_tuple = self._compute_hold_goal()
            detail = "hold_position"
        else:
            if self.last_goal_state != "idle":
                rospy.loginfo("stage2_follow_goal_generator state=idle")
            self._publish_state(now, "idle", "waiting_target_or_odom")
            self.last_goal_state = "idle"
            self._clear_search_anchor()
            return

        if goal_tuple is None:
            self._publish_state(now, "idle", "goal_unavailable")
            return

        if state_name != self.last_goal_state:
            rospy.loginfo("stage2_follow_goal_generator state=%s", state_name)

        self.last_goal_yaw = goal_tuple[3]
        if self.last_goal_projection_note != "free":
            detail = "%s | %s" % (detail, self.last_goal_projection_note)
        self._publish_state(now, state_name, detail)
        if self._should_publish_goal(goal_tuple, state_name):
            msg = self._make_pose(now, goal_tuple)
            try:
                self.goal_pub.publish(msg)
            except rospy.ROSException:
                return
            self.last_goal = goal_tuple
        self._publish_debug_path(now, goal_tuple, state_name)
        self.last_goal_state = state_name


if __name__ == "__main__":
    rospy.init_node("stage2_follow_goal_generator")
    Stage2FollowGoalGeneratorNode()
    rospy.spin()
