#!/usr/bin/env python3
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import FollowState, Target3D

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:
    PositionCommand = None


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


def normalize2(x_value, y_value, fallback_x=1.0, fallback_y=0.0):
    norm = math.hypot(x_value, y_value)
    if norm < 1e-6:
        return fallback_x, fallback_y
    return x_value / norm, y_value / norm


def mat_eye(size, scale=1.0):
    return [[scale if row == col else 0.0 for col in range(size)] for row in range(size)]


def mat_add(left, right):
    return [
        [left[row][col] + right[row][col] for col in range(len(left[0]))]
        for row in range(len(left))
    ]


def mat_sub(left, right):
    return [
        [left[row][col] - right[row][col] for col in range(len(left[0]))]
        for row in range(len(left))
    ]


def mat_mul(left, right):
    out = [[0.0 for _ in range(len(right[0]))] for _ in range(len(left))]
    for row in range(len(left)):
        for col in range(len(right[0])):
            out[row][col] = sum(left[row][idx] * right[idx][col] for idx in range(len(right)))
    return out


def mat_transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def mat_vec_mul(matrix, vector):
    return [sum(matrix[row][col] * vector[col] for col in range(len(vector))) for row in range(len(matrix))]


def inv2(matrix):
    det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
    if abs(det) < 1e-9:
        det = 1e-9 if det >= 0.0 else -1e-9
    return [
        [matrix[1][1] / det, -matrix[0][1] / det],
        [-matrix[1][0] / det, matrix[0][0] / det],
    ]


class ConstantVelocityKalmanFilter:
    def __init__(self, process_var, measurement_var, initial_covariance):
        self.process_var = max(float(process_var), 1e-6)
        self.measurement_var = max(float(measurement_var), 1e-6)
        self.initial_covariance = max(float(initial_covariance), 1e-6)
        self.state = None
        self.covariance = mat_eye(4, self.initial_covariance)
        self.last_update_sec = None

    def update(self, stamp_sec, meas_x, meas_y):
        stamp_sec = float(stamp_sec)
        if self.state is None:
            self.state = [float(meas_x), float(meas_y), 0.0, 0.0]
            self.covariance = mat_eye(4, self.initial_covariance)
            self.last_update_sec = stamp_sec
            return

        dt_sec = clamp(stamp_sec - self.last_update_sec, 1e-3, 0.5)
        self.last_update_sec = stamp_sec
        self._predict_in_place(dt_sec)

        h_mat = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
        h_t = mat_transpose(h_mat)
        r_mat = mat_eye(2, self.measurement_var)
        innovation = [float(meas_x) - self.state[0], float(meas_y) - self.state[1]]
        s_mat = mat_add(mat_mul(mat_mul(h_mat, self.covariance), h_t), r_mat)
        k_mat = mat_mul(mat_mul(self.covariance, h_t), inv2(s_mat))
        correction = mat_vec_mul(k_mat, innovation)
        self.state = [self.state[idx] + correction[idx] for idx in range(4)]
        self.covariance = mat_mul(mat_sub(mat_eye(4), mat_mul(k_mat, h_mat)), self.covariance)

    def predict(self, horizon_sec):
        if self.state is None:
            return None, None
        f_mat, q_mat = self._motion_matrices(max(float(horizon_sec), 0.0))
        state = mat_vec_mul(f_mat, list(self.state))
        covariance = mat_add(mat_mul(mat_mul(f_mat, self.covariance), mat_transpose(f_mat)), q_mat)
        return state, covariance

    def _predict_in_place(self, dt_sec):
        f_mat, q_mat = self._motion_matrices(dt_sec)
        self.state = mat_vec_mul(f_mat, self.state)
        self.covariance = mat_add(mat_mul(mat_mul(f_mat, self.covariance), mat_transpose(f_mat)), q_mat)

    def _motion_matrices(self, dt_sec):
        dt2 = dt_sec * dt_sec
        dt3 = dt2 * dt_sec
        dt4 = dt2 * dt2
        q = self.process_var
        f_mat = [
            [1.0, 0.0, dt_sec, 0.0],
            [0.0, 1.0, 0.0, dt_sec],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        q_mat = [
            [0.25 * dt4 * q, 0.0, 0.5 * dt3 * q, 0.0],
            [0.0, 0.25 * dt4 * q, 0.0, 0.5 * dt3 * q],
            [0.5 * dt3 * q, 0.0, dt2 * q, 0.0],
            [0.0, 0.5 * dt3 * q, 0.0, dt2 * q],
        ]
        return f_mat, q_mat


class Candidate:
    def __init__(self, name, x_value, y_value, z_value, yaw_rad, ref_x, ref_y):
        self.name = name
        self.x = float(x_value)
        self.y = float(y_value)
        self.z = float(z_value)
        self.yaw = float(yaw_rad)
        self.ref_x = float(ref_x)
        self.ref_y = float(ref_y)
        self.score = 0.0
        self.detail = ""


class PaperLineStage2GoalNode:
    def __init__(self):
        self.target_world_topic = rospy.get_param("~target_world_topic", "/follow/fusion/target_world")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/lio/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/follow/stage2/goal")
        self.debug_path_topic = rospy.get_param("~debug_path_topic", "/follow/stage2/debug_goal_path")
        self.state_topic = rospy.get_param("~state_topic", "/follow/stage2/state")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.rate_hz = max(float(rospy.get_param("~rate_hz", 15.0)), 1e-3)
        self.target_timeout_sec = float(rospy.get_param("~target_timeout_sec", 0.35))
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.6))
        self.follow_distance_m = float(rospy.get_param("~follow_distance_m", 3.5))
        self.lateral_bias_m = float(rospy.get_param("~lateral_bias_m", 0.0))
        self.vertical_bias_m = float(rospy.get_param("~vertical_bias_m", 0.0))
        self.min_goal_shift_m = float(rospy.get_param("~min_goal_shift_m", 0.08))
        self.face_target_yaw = bool(rospy.get_param("~face_target_yaw", True))
        self.publish_every_tick = bool(rospy.get_param("~publish_every_tick", True))

        self.prediction_horizon_sec = max(float(rospy.get_param("~prediction_horizon_sec", 0.6)), 0.0)
        self.predict_hold_timeout_sec = max(float(rospy.get_param("~predict_hold_timeout_sec", 1.2)), 0.0)
        self.lost_timeout_sec = max(float(rospy.get_param("~lost_timeout_sec", 5.0)), 0.0)
        self.search_goal_radius_m = max(float(rospy.get_param("~search_goal_radius_m", 1.0)), 0.1)
        self.search_observation_distance_m = float(rospy.get_param("~search_observation_distance_m", -1.0))
        self.min_motion_speed_mps = max(float(rospy.get_param("~min_motion_speed_mps", 0.15)), 0.0)

        self.kf = ConstantVelocityKalmanFilter(
            process_var=float(rospy.get_param("~kf_process_var", 1.2)),
            measurement_var=float(rospy.get_param("~kf_measurement_var", 0.09)),
            initial_covariance=float(rospy.get_param("~kf_initial_covariance", 1.0)),
        )

        self.occupancy_topic = rospy.get_param("~occupancy_inflate_topic", "/grid_map/occupancy_inflate")
        self.occupancy_timeout_sec = float(rospy.get_param("~occupancy_timeout_sec", 0.6))
        self.occupancy_resolution_m = max(float(rospy.get_param("~occupancy_resolution_m", 0.1)), 0.02)
        self.enable_obstacle_safety_filter = bool(rospy.get_param("~enable_obstacle_safety_filter", True))
        self.require_initial_occupancy_before_planning = bool(
            rospy.get_param("~require_initial_occupancy_before_planning", False)
        )
        self.base_safety_margin_m = max(float(rospy.get_param("~base_safety_margin_m", 0.28)), 0.0)
        self.obstacle_margin_m = max(float(rospy.get_param("~obstacle_margin_m", 0.18)), 0.0)
        self.speed_margin_gain = max(float(rospy.get_param("~speed_margin_gain", 0.35)), 0.0)
        self.covariance_margin_gain = max(float(rospy.get_param("~covariance_margin_gain", 1.0)), 0.0)
        self.segment_sample_step_m = max(float(rospy.get_param("~segment_sample_step_m", 0.15)), 0.03)

        self.enable_planner_feedback = bool(rospy.get_param("~enable_planner_feedback", True))
        self.require_planner_feedback = bool(rospy.get_param("~require_planner_feedback", False))
        self.planner_feedback_topic = rospy.get_param("~planner_feedback_topic", "/follow/stage2/ego_position_cmd")
        self.planner_feedback_timeout_sec = max(
            float(rospy.get_param("~planner_feedback_timeout_sec", 1.0)),
            0.1,
        )
        self.planner_feedback_startup_grace_sec = max(
            float(rospy.get_param("~planner_feedback_startup_grace_sec", 3.0)),
            0.0,
        )
        self.failed_candidate_cooldown_sec = max(
            float(rospy.get_param("~failed_candidate_cooldown_sec", 2.5)),
            0.0,
        )
        self.failed_candidate_radius_m = max(float(rospy.get_param("~failed_candidate_radius_m", 0.7)), 0.0)

        self.weight_distance = float(rospy.get_param("~weight_distance", 0.24))
        self.weight_clearance = float(rospy.get_param("~weight_clearance", 0.26))
        self.weight_motion = float(rospy.get_param("~weight_motion", 0.20))
        self.weight_visibility = float(rospy.get_param("~weight_visibility", 0.12))
        self.weight_switching = float(rospy.get_param("~weight_switching", 0.10))
        self.weight_uncertainty = float(rospy.get_param("~weight_uncertainty", 0.08))

        self.start_monotonic_sec = time.monotonic()
        self.last_valid_target = None
        self.last_valid_target_stamp = None
        self.last_target_seen_monotonic_sec = None
        self.last_odom = None
        self.last_odom_stamp = None
        self.last_goal = None
        self.last_goal_state = "idle"
        self.last_goal_yaw = 0.0
        self.last_candidate_name = ""
        self.last_candidate_stamp_sec = None
        self.last_planner_feedback_monotonic_sec = None
        self.last_planner_checked_goal_stamp_sec = None
        self.search_phase = 0
        self.failed_candidates = []
        self._occupancy_lock = threading.Lock()
        self._occupied_cells = set()
        self._occupancy_stamp = None

        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10, latch=True)
        self.path_pub = rospy.Publisher(self.debug_path_topic, Path, queue_size=10)
        self.state_pub = rospy.Publisher(self.state_topic, FollowState, queue_size=10)

        rospy.Subscriber(self.target_world_topic, Target3D, self._target_callback, queue_size=20)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(self.occupancy_topic, PointCloud2, self._occupancy_callback, queue_size=2)
        if self.enable_planner_feedback:
            feedback_type = PositionCommand if PositionCommand is not None else rospy.AnyMsg
            rospy.Subscriber(self.planner_feedback_topic, feedback_type, self._planner_feedback_callback, queue_size=20)

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)
        rospy.loginfo(
            "paper_line_stage2_goal ready: target=%s odom=%s goal=%s feedback=%s",
            self.target_world_topic,
            self.odom_topic,
            self.goal_topic,
            self.planner_feedback_topic if self.enable_planner_feedback else "disabled",
        )

    def _target_callback(self, msg):
        if not msg.valid:
            return
        now = rospy.Time.now()
        self.last_valid_target = msg
        self.last_valid_target_stamp = now
        self.last_target_seen_monotonic_sec = time.monotonic()
        self.kf.update(now.to_sec(), msg.position.x, msg.position.y)

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_stamp = rospy.Time.now()

    def _occupancy_callback(self, msg):
        if not self.enable_obstacle_safety_filter:
            return
        occupied = set()
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            occupied.add(self._cell_key(point[0], point[1], point[2]))
        with self._occupancy_lock:
            self._occupied_cells = occupied
            self._occupancy_stamp = rospy.Time.now()

    def _planner_feedback_callback(self, _msg):
        self.last_planner_feedback_monotonic_sec = time.monotonic()

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

    def _target_age_sec(self):
        if self.last_target_seen_monotonic_sec is None:
            return None
        return max(time.monotonic() - self.last_target_seen_monotonic_sec, 0.0)

    def _has_seen_any_occupancy(self):
        with self._occupancy_lock:
            return self._occupancy_stamp is not None

    def _occupancy_cells_snapshot(self, now):
        if not self.enable_obstacle_safety_filter:
            return None
        with self._occupancy_lock:
            stamp = self._occupancy_stamp
            occupied = set(self._occupied_cells)
        if stamp is None or (now - stamp).to_sec() > self.occupancy_timeout_sec:
            return None
        return occupied

    def _cell_key(self, x_value, y_value, z_value):
        res = self.occupancy_resolution_m
        return (
            int(round(float(x_value) / res)),
            int(round(float(y_value) / res)),
            int(round(float(z_value) / res)),
        )

    def _point_in_obstacle(self, x_value, y_value, z_value, margin_m, occupied):
        if not self.enable_obstacle_safety_filter or occupied is None:
            return False
        radius_cells = int(math.ceil(max(float(margin_m), 0.0) / self.occupancy_resolution_m))
        center = self._cell_key(x_value, y_value, z_value)
        for ix in range(center[0] - radius_cells, center[0] + radius_cells + 1):
            for iy in range(center[1] - radius_cells, center[1] + radius_cells + 1):
                for iz in range(center[2] - radius_cells, center[2] + radius_cells + 1):
                    if (ix, iy, iz) in occupied:
                        return True
        return False

    def _segment_in_obstacle(self, start_xyz, end_xyz, margin_m, occupied):
        if not self.enable_obstacle_safety_filter or occupied is None:
            return False
        dx = end_xyz[0] - start_xyz[0]
        dy = end_xyz[1] - start_xyz[1]
        dz = end_xyz[2] - start_xyz[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        sample_count = max(int(math.ceil(distance / self.segment_sample_step_m)), 1)
        for idx in range(sample_count + 1):
            ratio = float(idx) / float(sample_count)
            if self._point_in_obstacle(
                start_xyz[0] + ratio * dx,
                start_xyz[1] + ratio * dy,
                start_xyz[2] + ratio * dz,
                margin_m,
                occupied,
            ):
                return True
        return False

    def _clearance_score(self, candidate, margin_m, occupied):
        if not self.enable_obstacle_safety_filter or occupied is None:
            return 1.0
        candidate_key = self._cell_key(candidate.x, candidate.y, candidate.z)
        scan_cells = int(math.ceil(max(2.0 * margin_m, 1.0) / self.occupancy_resolution_m))
        best = None
        for cell in occupied:
            if (
                abs(cell[0] - candidate_key[0]) > scan_cells
                or abs(cell[1] - candidate_key[1]) > scan_cells
                or abs(cell[2] - candidate_key[2]) > scan_cells
            ):
                continue
            dx = (cell[0] - candidate_key[0]) * self.occupancy_resolution_m
            dy = (cell[1] - candidate_key[1]) * self.occupancy_resolution_m
            dz = (cell[2] - candidate_key[2]) * self.occupancy_resolution_m
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            best = distance if best is None else min(best, distance)
        if best is None:
            return 1.0
        return clamp((best - margin_m) / max(2.0 * margin_m, 1e-3), 0.0, 1.0)

    def _vehicle_xyz_yaw_speed(self):
        pose = self.last_odom.pose.pose
        twist = self.last_odom.twist.twist
        yaw_rad = yaw_from_quaternion(pose.orientation)
        speed = math.sqrt(
            twist.linear.x * twist.linear.x
            + twist.linear.y * twist.linear.y
            + twist.linear.z * twist.linear.z
        )
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            yaw_rad,
            speed,
        )

    def _state_name(self, now):
        if not self._odom_is_fresh(now):
            return "idle"
        if self._target_is_fresh(now):
            return "follow"
        age = self._target_age_sec()
        if age is None:
            return "idle"
        if age <= self.predict_hold_timeout_sec:
            return "predict_hold"
        if age <= self.lost_timeout_sec:
            return "search_safe_viewpoint"
        return "hold_safe"

    def _state_enum(self, state_name):
        if state_name == "follow":
            return FollowState.FOLLOW
        if state_name == "predict_hold":
            return FollowState.TARGET_ACQUIRED
        if state_name == "search_safe_viewpoint":
            return FollowState.SEARCH
        if state_name in ("hold_safe", "failsafe"):
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

    def _prediction(self, state_name):
        horizon = self.prediction_horizon_sec
        if state_name == "predict_hold":
            horizon = max(horizon, min(self.predict_hold_timeout_sec, horizon + 0.4))
        predicted, covariance = self.kf.predict(horizon)
        if predicted is None and self.last_valid_target is not None:
            predicted = [
                float(self.last_valid_target.position.x),
                float(self.last_valid_target.position.y),
                0.0,
                0.0,
            ]
            covariance = mat_eye(4, self.kf.initial_covariance)
        return predicted, covariance

    def _dynamic_margin(self, vehicle_speed, covariance):
        cov_term = 0.0
        if covariance is not None:
            cov_term = math.sqrt(max(covariance[0][0] + covariance[1][1], 0.0))
        return (
            self.base_safety_margin_m
            + self.obstacle_margin_m
            + self.speed_margin_gain * max(vehicle_speed, 0.0)
            + self.covariance_margin_gain * cov_term
        )

    def _make_candidate(self, name, x_value, y_value, z_value, ref_x, ref_y):
        yaw_rad = self.last_goal_yaw
        if self.face_target_yaw:
            yaw_rad = math.atan2(ref_y - y_value, ref_x - x_value)
        return Candidate(name, x_value, y_value, z_value, yaw_rad, ref_x, ref_y)

    def _generate_follow_candidates(self, predicted, vehicle_xyz):
        target_x, target_y, vel_x, vel_y = predicted[0], predicted[1], predicted[2], predicted[3]
        vehicle_x, vehicle_y, vehicle_z = vehicle_xyz
        target_z = float(self.last_valid_target.position.z) if self.last_valid_target is not None else vehicle_z
        goal_z = max(vehicle_z, target_z + self.vertical_bias_m)

        if math.hypot(vel_x, vel_y) >= self.min_motion_speed_mps:
            forward_x, forward_y = normalize2(vel_x, vel_y)
            behind_x, behind_y = -forward_x, -forward_y
        else:
            behind_x, behind_y = normalize2(vehicle_x - target_x, vehicle_y - target_y)
            forward_x, forward_y = -behind_x, -behind_y
        left_x, left_y = -forward_y, forward_x
        far_distance = self.follow_distance_m + max(0.8, 0.35 * self.follow_distance_m)

        return [
            self._make_candidate(
                "behind",
                target_x + behind_x * self.follow_distance_m + left_x * self.lateral_bias_m,
                target_y + behind_y * self.follow_distance_m + left_y * self.lateral_bias_m,
                goal_z,
                target_x,
                target_y,
            ),
            self._make_candidate(
                "left",
                target_x - forward_x * 0.35 * self.follow_distance_m + left_x * self.follow_distance_m,
                target_y - forward_y * 0.35 * self.follow_distance_m + left_y * self.follow_distance_m,
                goal_z,
                target_x,
                target_y,
            ),
            self._make_candidate(
                "right",
                target_x - forward_x * 0.35 * self.follow_distance_m - left_x * self.follow_distance_m,
                target_y - forward_y * 0.35 * self.follow_distance_m - left_y * self.follow_distance_m,
                goal_z,
                target_x,
                target_y,
            ),
            self._make_candidate(
                "far_safe",
                target_x + behind_x * far_distance,
                target_y + behind_y * far_distance,
                goal_z,
                target_x,
                target_y,
            ),
        ]

    def _generate_search_candidates(self, predicted, vehicle_xyz):
        anchor_x, anchor_y = predicted[0], predicted[1]
        anchor_z = float(self.last_valid_target.position.z) if self.last_valid_target is not None else vehicle_xyz[2]
        observation_distance = self.search_observation_distance_m
        if observation_distance <= 0.0:
            observation_distance = max(self.follow_distance_m, self.search_goal_radius_m)
        base_heading = math.atan2(vehicle_xyz[1] - anchor_y, vehicle_xyz[0] - anchor_x)
        offsets = [0.0, math.radians(45.0), -math.radians(45.0), math.radians(90.0), -math.radians(90.0)]
        candidates = []
        for idx, offset in enumerate(offsets):
            heading = base_heading + offset + 0.15 * float(self.search_phase)
            candidates.append(
                self._make_candidate(
                    "search_reacquire_%d" % (idx + 1),
                    anchor_x + math.cos(heading) * observation_distance,
                    anchor_y + math.sin(heading) * observation_distance,
                    max(vehicle_xyz[2], anchor_z + self.vertical_bias_m),
                    anchor_x,
                    anchor_y,
                )
            )
        self.search_phase = (self.search_phase + 1) % 20
        return candidates

    def _candidate_in_cooldown(self, candidate, now_sec):
        kept = []
        blocked = False
        for record in self.failed_candidates:
            if now_sec - record["time_sec"] <= self.failed_candidate_cooldown_sec:
                kept.append(record)
                same_name = record["name"] == candidate.name
                distance = math.hypot(record["x"] - candidate.x, record["y"] - candidate.y)
                if same_name or distance <= self.failed_candidate_radius_m:
                    blocked = True
        self.failed_candidates = kept
        return blocked

    def _mark_candidate_failed(self, candidate, now_sec):
        if candidate is None:
            return
        self.failed_candidates.append({"name": candidate.name, "x": candidate.x, "y": candidate.y, "time_sec": now_sec})
        rospy.logwarn_throttle(
            1.0,
            "paper_line_stage2_goal cooldown candidate=%s x=%.2f y=%.2f",
            candidate.name,
            candidate.x,
            candidate.y,
        )

    def _planner_feedback_is_late(self):
        if not self.enable_planner_feedback or self.last_candidate_stamp_sec is None:
            return False
        if time.monotonic() - self.start_monotonic_sec < self.planner_feedback_startup_grace_sec:
            return False
        if self.last_planner_feedback_monotonic_sec is None:
            if not self.require_planner_feedback:
                return False
            return (time.monotonic() - self.last_candidate_stamp_sec) > self.planner_feedback_timeout_sec
        return (time.monotonic() - self.last_planner_feedback_monotonic_sec) > self.planner_feedback_timeout_sec

    def _filter_and_score_candidates(self, candidates, vehicle_xyz, vehicle_speed, covariance, occupied, now_sec):
        margin = self._dynamic_margin(vehicle_speed, covariance)
        safe = []
        for candidate in candidates:
            if self._candidate_in_cooldown(candidate, now_sec):
                continue
            if self._point_in_obstacle(candidate.x, candidate.y, candidate.z, margin, occupied):
                continue
            if self._segment_in_obstacle(vehicle_xyz, (candidate.x, candidate.y, candidate.z), margin, occupied):
                continue

            follow_distance = math.hypot(candidate.x - candidate.ref_x, candidate.y - candidate.ref_y)
            distance_score = clamp(
                1.0 - abs(follow_distance - self.follow_distance_m) / max(self.follow_distance_m, 1e-3),
                0.0,
                1.0,
            )
            clearance_score = self._clearance_score(candidate, margin, occupied)
            move_distance = math.sqrt(
                (candidate.x - vehicle_xyz[0]) ** 2
                + (candidate.y - vehicle_xyz[1]) ** 2
                + (candidate.z - vehicle_xyz[2]) ** 2
            )
            motion_score = 1.0 / (1.0 + move_distance)
            target_yaw = math.atan2(candidate.ref_y - candidate.y, candidate.ref_x - candidate.x)
            visibility_score = clamp((math.cos(wrap_pi(candidate.yaw - target_yaw)) + 1.0) * 0.5, 0.0, 1.0)
            switching_score = 1.0 if candidate.name == self.last_candidate_name else 0.45
            uncertainty = 0.0
            if covariance is not None:
                uncertainty = math.sqrt(max(covariance[0][0] + covariance[1][1], 0.0))
            uncertainty_score = 1.0 / (1.0 + uncertainty)
            candidate.score = (
                self.weight_distance * distance_score
                + self.weight_clearance * clearance_score
                + self.weight_motion * motion_score
                + self.weight_visibility * visibility_score
                + self.weight_switching * switching_score
                + self.weight_uncertainty * uncertainty_score
            )
            candidate.detail = "candidate=%s score=%.3f margin=%.2f" % (candidate.name, candidate.score, margin)
            safe.append(candidate)
        safe.sort(key=lambda item: item.score, reverse=True)
        return safe, margin

    def _compute_hold_candidate(self):
        if self.last_goal is not None:
            return Candidate(
                "hold_last_goal",
                self.last_goal[0],
                self.last_goal[1],
                self.last_goal[2],
                self.last_goal[3],
                self.last_goal[0],
                self.last_goal[1],
            )
        if self.last_odom is None:
            return None
        vehicle_x, vehicle_y, vehicle_z, vehicle_yaw, _speed = self._vehicle_xyz_yaw_speed()
        return Candidate("hold_vehicle", vehicle_x, vehicle_y, vehicle_z, vehicle_yaw, vehicle_x, vehicle_y)

    def _should_publish_goal(self, candidate, state_name):
        if self.publish_every_tick:
            return True
        if self.last_goal_state != state_name or self.last_goal is None:
            return True
        dx = candidate.x - self.last_goal[0]
        dy = candidate.y - self.last_goal[1]
        dz = candidate.z - self.last_goal[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) >= self.min_goal_shift_m

    def _make_pose(self, stamp, candidate):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame_id
        msg.pose.position.x = candidate.x
        msg.pose.position.y = candidate.y
        msg.pose.position.z = candidate.z
        half_yaw = candidate.yaw * 0.5
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        return msg

    def _publish_debug_path(self, stamp, candidate):
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.world_frame_id
        points = []
        if self.last_odom is not None:
            pose = self.last_odom.pose.pose.position
            points.append((float(pose.x), float(pose.y), float(pose.z)))
        points.append((candidate.x, candidate.y, candidate.z))
        points.append((candidate.ref_x, candidate.ref_y, candidate.z))
        for point in points:
            pose_msg = PoseStamped()
            pose_msg.header = path.header
            pose_msg.pose.position.x = point[0]
            pose_msg.pose.position.y = point[1]
            pose_msg.pose.position.z = point[2]
            pose_msg.pose.orientation.w = 1.0
            path.poses.append(pose_msg)
        try:
            self.path_pub.publish(path)
        except rospy.ROSException:
            return

    def _publish_candidate(self, now, state_name, candidate, detail):
        if state_name != self.last_goal_state:
            rospy.loginfo("paper_line_stage2_goal state=%s", state_name)
        if self._should_publish_goal(candidate, state_name):
            try:
                self.goal_pub.publish(self._make_pose(now, candidate))
            except rospy.ROSException:
                return
            self.last_candidate_stamp_sec = time.monotonic()
            self.last_goal = (candidate.x, candidate.y, candidate.z, candidate.yaw)
            self.last_goal_yaw = candidate.yaw
            self.last_candidate_name = candidate.name
        self._publish_state(now, state_name, detail)
        self._publish_debug_path(now, candidate)
        self.last_goal_state = state_name

    def _tick(self, _event):
        now = rospy.Time.now()
        now_sec = now.to_sec()
        state_name = self._state_name(now)
        if state_name == "idle":
            self._publish_state(now, "idle", "waiting_target_or_odom")
            self.last_goal_state = "idle"
            return

        occupied = self._occupancy_cells_snapshot(now)
        if (
            self.require_initial_occupancy_before_planning
            and self.enable_obstacle_safety_filter
            and not self._has_seen_any_occupancy()
        ):
            candidate = self._compute_hold_candidate()
            if candidate is not None:
                self._publish_candidate(now, "hold_safe", candidate, "waiting_initial_occupancy")
            return

        if self._planner_feedback_is_late() and self.last_goal is not None:
            failed_candidate = Candidate(
                self.last_candidate_name or "unknown",
                self.last_goal[0],
                self.last_goal[1],
                self.last_goal[2],
                self.last_goal[3],
                self.last_goal[0],
                self.last_goal[1],
            )
            if self.last_planner_checked_goal_stamp_sec != self.last_candidate_stamp_sec:
                self._mark_candidate_failed(failed_candidate, now_sec)
                self.last_planner_checked_goal_stamp_sec = self.last_candidate_stamp_sec

        vehicle_x, vehicle_y, vehicle_z, _vehicle_yaw, vehicle_speed = self._vehicle_xyz_yaw_speed()
        vehicle_xyz = (vehicle_x, vehicle_y, vehicle_z)
        predicted, covariance = self._prediction(state_name)
        if predicted is None:
            candidate = self._compute_hold_candidate()
            if candidate is None:
                self._publish_state(now, "idle", "prediction_unavailable")
                return
            self._publish_candidate(now, "hold_safe", candidate, "prediction_unavailable_hold")
            return

        if state_name in ("follow", "predict_hold"):
            candidates = self._generate_follow_candidates(predicted, vehicle_xyz)
        elif state_name == "search_safe_viewpoint":
            candidates = self._generate_search_candidates(predicted, vehicle_xyz)
        else:
            candidates = []

        safe, margin = self._filter_and_score_candidates(
            candidates,
            vehicle_xyz,
            vehicle_speed,
            covariance,
            occupied,
            now_sec,
        )
        if safe:
            candidate = safe[0]
            detail = "%s margin=%.2f" % (candidate.detail, margin)
            self._publish_candidate(now, state_name, candidate, detail)
            return

        candidate = self._compute_hold_candidate()
        if candidate is None:
            self._publish_state(now, "failsafe", "no_safe_candidate_no_hold")
            self.last_goal_state = "failsafe"
            return
        self._publish_candidate(now, "hold_safe", candidate, "no_safe_candidate_hold margin=%.2f" % margin)


if __name__ == "__main__":
    rospy.init_node("paper_line_stage2_goal")
    PaperLineStage2GoalNode()
    rospy.spin()
