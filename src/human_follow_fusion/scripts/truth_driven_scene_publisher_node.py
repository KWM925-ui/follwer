#!/usr/bin/env python3
import os
import sys

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from human_follow_msgs.msg import Target2D, Target3D

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from projection_math import (  # noqa: E402
    load_camera_intrinsics_yaml,
    load_camera_lidar_extrinsics_yaml,
    load_rigid_transform_yaml,
    project_camera_points_to_pixels,
    transform_points,
)


def parse_phase_descriptor(text):
    parsed = {
        "label": "",
        "visible": False,
        "truth_valid": False,
        "phase_index": -1,
    }
    if not text:
        return parsed

    for item in str(text).split(";"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        key = key.strip()
        value = value.strip()
        if key == "label":
            parsed["label"] = value
        elif key == "visible":
            parsed["visible"] = value in ("1", "true", "True", "yes")
        elif key == "truth_valid":
            parsed["truth_valid"] = value in ("1", "true", "True", "yes")
        elif key == "phase_index":
            try:
                parsed["phase_index"] = int(value)
            except ValueError:
                parsed["phase_index"] = -1
    return parsed


class TruthDrivenScenePublisherNode:
    def __init__(self):
        self.truth_topic = rospy.get_param("~truth_topic", "/follow/sim/truth_target_body")
        self.phase_topic = rospy.get_param("~phase_topic", "/follow/sim/truth_phase")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/follow/lidar/points")
        self.foreground_pointcloud_topic = str(
            rospy.get_param("~foreground_pointcloud_topic", "/follow/lidar/foreground_points")
        ).strip()
        self.obstacle_pointcloud_topic = str(
            rospy.get_param("~obstacle_pointcloud_topic", "/follow/lidar/obstacle_points")
        ).strip()
        self.target_topic = rospy.get_param("~target_topic", "/follow/detector/person_target")
        self.debug_target_topic = rospy.get_param("~debug_target_topic", "/follow/sim/truth_target_2d")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.intrinsics_yaml = rospy.get_param("~intrinsics_yaml")
        self.extrinsics_yaml = rospy.get_param("~extrinsics_yaml")
        self.camera_body_yaml = rospy.get_param("~camera_body_yaml")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 15.0))
        self.target_bbox_padding = float(rospy.get_param("~target_bbox_padding", 1.12))
        self.truth_timeout_sec = float(rospy.get_param("~truth_timeout_sec", 0.5))
        self.min_projected_points = int(rospy.get_param("~min_projected_points", 6))
        self.phase_visible_affects_tracking = bool(rospy.get_param("~phase_visible_affects_tracking", True))
        self.person_width_m = float(rospy.get_param("~person_width_m", 0.60))
        self.person_height_m = float(rospy.get_param("~person_height_m", 1.70))
        self.person_depth_m = float(rospy.get_param("~person_depth_m", 0.18))
        self.person_columns = max(2, int(rospy.get_param("~person_columns", 4)))
        self.person_rows = max(2, int(rospy.get_param("~person_rows", 6)))
        self.static_obstacle_layout = str(rospy.get_param("~static_obstacle_layout", "")).strip()
        self.static_obstacle_point_spacing_m = max(
            float(rospy.get_param("~static_obstacle_point_spacing_m", 0.18)),
            0.05,
        )
        self.static_obstacle_max_depth_m = max(
            float(rospy.get_param("~static_obstacle_max_depth_m", 20.0)),
            0.5,
        )
        self.occlusion_depth_margin_m = max(
            float(rospy.get_param("~occlusion_depth_margin_m", 0.10)),
            0.0,
        )
        self.occlusion_pixel_radius_px = max(
            int(rospy.get_param("~occlusion_pixel_radius_px", 3)),
            0,
        )

        self.intrinsics = load_camera_intrinsics_yaml(self.intrinsics_yaml)
        self.extrinsics = load_camera_lidar_extrinsics_yaml(self.extrinsics_yaml)
        self.body_transform = load_rigid_transform_yaml(self.camera_body_yaml)
        self.camera_T_body = np.linalg.inv(self.body_transform["target_T_source"])
        self.lidar_T_camera = np.linalg.inv(self.extrinsics["camera_T_lidar"])

        self.last_truth = None
        self.last_truth_time = None
        self.last_phase = parse_phase_descriptor("")
        self.last_phase_time = None
        self.last_phase_label = None
        self.last_odom = None
        self.last_odom_time = None

        self.pointcloud_pub = rospy.Publisher(self.pointcloud_topic, PointCloud2, queue_size=1)
        self.foreground_pointcloud_pub = None
        if self.foreground_pointcloud_topic:
            self.foreground_pointcloud_pub = rospy.Publisher(
                self.foreground_pointcloud_topic,
                PointCloud2,
                queue_size=1,
            )
        self.obstacle_pointcloud_pub = None
        if self.obstacle_pointcloud_topic:
            self.obstacle_pointcloud_pub = rospy.Publisher(
                self.obstacle_pointcloud_topic,
                PointCloud2,
                queue_size=1,
            )
        self.target_pub = rospy.Publisher(self.target_topic, Target2D, queue_size=10)
        self.debug_target_pub = rospy.Publisher(self.debug_target_topic, Target2D, queue_size=10)

        self.truth_sub = rospy.Subscriber(self.truth_topic, Target3D, self._truth_callback, queue_size=10)
        self.phase_sub = rospy.Subscriber(self.phase_topic, String, self._phase_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        self.foreground_offsets_camera = self._build_foreground_offsets()
        self.static_obstacle_points_world = self._build_static_obstacle_layout_points_world()

        rospy.loginfo(
            "truth_driven_scene_publisher ready: truth=%s phase=%s odom=%s pointcloud=%s foreground_pointcloud=%s obstacle_pointcloud=%s target=%s obstacle_layout=%s",
            self.truth_topic,
            self.phase_topic,
            self.odom_topic,
            self.pointcloud_topic,
            self.foreground_pointcloud_topic or "disabled",
            self.obstacle_pointcloud_topic or "disabled",
            self.target_topic,
            self.static_obstacle_layout or "none",
        )

    def _publish_foreground_pointcloud(self, points_camera_xyz):
        if self.foreground_pointcloud_pub is None:
            return
        self.foreground_pointcloud_pub.publish(self._make_pointcloud_msg(points_camera_xyz))

    def _publish_obstacle_pointcloud(self, points_camera_xyz):
        if self.obstacle_pointcloud_pub is None:
            return
        self.obstacle_pointcloud_pub.publish(self._make_pointcloud_msg(points_camera_xyz))

    def _build_foreground_offsets(self):
        x_offsets = np.linspace(-0.5 * self.person_width_m, 0.5 * self.person_width_m, self.person_columns)
        y_offsets = np.linspace(-0.5 * self.person_height_m, 0.5 * self.person_height_m, self.person_rows)
        z_offsets = np.linspace(-0.5 * self.person_depth_m, 0.5 * self.person_depth_m, 2)

        offsets = []
        for y_value in y_offsets:
            for x_value in x_offsets:
                for z_value in z_offsets:
                    offsets.append([x_value, y_value, z_value])
        return np.array(offsets, dtype=np.float64)

    def _truth_callback(self, msg):
        self.last_truth = msg
        self.last_truth_time = rospy.Time.now()

    def _phase_callback(self, msg):
        self.last_phase = parse_phase_descriptor(msg.data)
        self.last_phase_time = rospy.Time.now()

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_time = rospy.Time.now()

    def _truth_is_fresh(self):
        if self.last_truth is None or self.last_truth_time is None:
            return False
        return (rospy.Time.now() - self.last_truth_time).to_sec() <= self.truth_timeout_sec

    def _phase_is_fresh(self):
        if self.last_phase_time is None:
            return False
        return (rospy.Time.now() - self.last_phase_time).to_sec() <= self.truth_timeout_sec

    def _odom_is_fresh(self):
        if self.last_odom is None or self.last_odom_time is None:
            return False
        return (rospy.Time.now() - self.last_odom_time).to_sec() <= self.truth_timeout_sec

    def _sample_axis(self, min_value, max_value):
        if max_value <= min_value:
            return np.array([min_value], dtype=np.float64)
        point_count = max(int(np.ceil((max_value - min_value) / self.static_obstacle_point_spacing_m)) + 1, 2)
        return np.linspace(min_value, max_value, point_count, dtype=np.float64)

    def _sample_box_volume(self, center_xyz, size_xyz):
        half_x = 0.5 * float(size_xyz[0])
        half_y = 0.5 * float(size_xyz[1])
        half_z = 0.5 * float(size_xyz[2])
        x_values = self._sample_axis(-half_x, half_x)
        y_values = self._sample_axis(-half_y, half_y)
        z_values = self._sample_axis(-half_z, half_z)

        points = []
        for x_value in x_values:
            for y_value in y_values:
                for z_value in z_values:
                    points.append([center_xyz[0] + x_value, center_xyz[1] + y_value, center_xyz[2] + z_value])
        return np.asarray(points, dtype=np.float64)

    def _build_static_obstacle_layout_points_world(self):
        if not self.static_obstacle_layout:
            return np.zeros((0, 3), dtype=np.float64)

        boxes = []
        if self.static_obstacle_layout == "ego_demo_three_boxes":
            boxes = [
                ((4.8, 0.0, 1.0), (0.8, 0.8, 2.0)),
                ((7.6, 1.55, 1.0), (0.9, 0.9, 2.0)),
                ((7.6, -1.55, 1.0), (0.9, 0.9, 2.0)),
            ]
        elif self.static_obstacle_layout == "ego_demo_gate":
            boxes = [
                ((4.5, 1.4, 1.0), (0.6, 2.0, 2.0)),
                ((4.5, -1.4, 1.0), (0.6, 2.0, 2.0)),
                ((7.5, 0.0, 1.0), (1.0, 0.8, 2.0)),
            ]
        elif self.static_obstacle_layout == "ego_demo_replan_slalom":
            boxes = [
                ((4.0, 1.55, 1.0), (0.8, 1.2, 2.0)),
                ((4.0, -1.55, 1.0), (0.8, 1.2, 2.0)),
                ((5.8, 0.55, 1.0), (0.9, 1.1, 2.0)),
                ((7.6, -0.55, 1.0), (0.9, 1.1, 2.0)),
                ((9.4, 1.55, 1.0), (0.8, 1.2, 2.0)),
                ((9.4, -1.55, 1.0), (0.8, 1.2, 2.0)),
            ]
        elif self.static_obstacle_layout == "ego_demo_manual_center_block":
            boxes = [
                # A deliberate manual-replan showcase geometry:
                # block the straight-ahead center line and remove the easy
                # "climb over the box" solution so the planner must detour laterally.
                ((4.90, 0.00, 1.35), (1.20, 1.10, 2.70)),
                ((6.90, 2.30, 1.35), (0.90, 1.00, 2.70)),
                ((6.90, -2.30, 1.35), (0.90, 1.00, 2.70)),
            ]
        elif self.static_obstacle_layout == "ego_demo_acceptance_dense_weave":
            boxes = [
                # Simple acceptance world: keep the original working
                # staggered center obstacles, then add only a few side
                # obstacles to make the scene richer without shrinking the
                # valid avoidance corridor.
                ((3.90, 0.05, 1.35), (0.95, 0.85, 2.70)),
                ((5.85, 2.10, 1.35), (0.85, 0.90, 2.70)),
                ((6.90, -1.95, 1.35), (0.90, 0.90, 2.70)),
                ((8.55, 1.80, 1.35), (0.85, 0.90, 2.70)),
                ((9.65, -1.60, 1.35), (0.85, 0.85, 2.70)),
                ((6.40, 4.40, 1.35), (0.72, 0.78, 2.70)),
                ((9.20, 4.85, 1.35), (0.72, 0.78, 2.70)),
                ((12.00, 4.55, 1.35), (0.72, 0.78, 2.70)),
                ((6.70, -4.55, 1.35), (0.72, 0.78, 2.70)),
                ((9.50, -4.95, 1.35), (0.72, 0.78, 2.70)),
                ((12.30, -4.65, 1.35), (0.72, 0.78, 2.70)),
            ]
        else:
            rospy.logwarn("unknown static_obstacle_layout '%s'; disabling static obstacles", self.static_obstacle_layout)
            return np.zeros((0, 3), dtype=np.float64)

        clouds = [self._sample_box_volume(center_xyz, size_xyz) for center_xyz, size_xyz in boxes]
        if not clouds:
            return np.zeros((0, 3), dtype=np.float64)
        return np.vstack(clouds)

    def _static_obstacles_camera_points(self):
        if self.static_obstacle_points_world.shape[0] == 0 or not self._odom_is_fresh():
            return np.zeros((0, 3), dtype=np.float64)

        vehicle_x = float(self.last_odom.pose.pose.position.x)
        vehicle_y = float(self.last_odom.pose.pose.position.y)
        vehicle_z = float(self.last_odom.pose.pose.position.z)
        orientation = self.last_odom.pose.pose.orientation
        yaw_rad = np.arctan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        cos_yaw = np.cos(yaw_rad)
        sin_yaw = np.sin(yaw_rad)

        delta_world = self.static_obstacle_points_world - np.array([[vehicle_x, vehicle_y, vehicle_z]], dtype=np.float64)
        points_body = np.zeros_like(delta_world)
        points_body[:, 0] = cos_yaw * delta_world[:, 0] + sin_yaw * delta_world[:, 1]
        points_body[:, 1] = -sin_yaw * delta_world[:, 0] + cos_yaw * delta_world[:, 1]
        points_body[:, 2] = delta_world[:, 2]

        points_camera = transform_points(points_body, self.camera_T_body)
        range_norm = np.linalg.norm(points_body, axis=1)
        valid_mask = (
            np.isfinite(points_camera[:, 0])
            & np.isfinite(points_camera[:, 1])
            & np.isfinite(points_camera[:, 2])
            # This cloud models lidar obstacles for both Stage1 fusion and Stage2 EGO.
            # It should not disappear just because an obstacle left the camera FOV.
            & np.isfinite(range_norm)
            & (range_norm <= self.static_obstacle_max_depth_m)
        )
        return points_camera[valid_mask]

    def _pointcloud_stamp(self):
        if (
            self.last_odom is not None
            and self._odom_is_fresh()
            and self.last_odom.header.stamp != rospy.Time(0)
        ):
            return self.last_odom.header.stamp
        return rospy.Time.now()

    def _empty_pointcloud(self):
        msg = PointCloud2()
        msg.header = Header(stamp=self._pointcloud_stamp(), frame_id=self.extrinsics["lidar_frame"])
        msg.height = 1
        msg.width = 0
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 0
        msg.is_dense = True
        msg.data = b""
        return msg

    def _make_pointcloud_msg(self, points_camera_xyz):
        if points_camera_xyz is None or points_camera_xyz.shape[0] == 0:
            return self._empty_pointcloud()

        lidar_points = transform_points(points_camera_xyz, self.lidar_T_camera).astype(np.float32)
        msg = PointCloud2()
        msg.header = Header(stamp=self._pointcloud_stamp(), frame_id=self.extrinsics["lidar_frame"])
        msg.height = 1
        msg.width = lidar_points.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = lidar_points.tobytes()
        return msg

    def _invalid_target_msg(self, label):
        msg = Target2D()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.intrinsics["camera_frame"]
        msg.track_id = -1
        msg.valid = False
        msg.confidence = 0.0
        msg.cx = 0.0
        msg.cy = 0.0
        msg.width = 0.0
        msg.height = 0.0
        msg.source = "truth_scene_hidden:%s" % label
        return msg

    def _truth_center_body_to_camera(self, truth_msg):
        point_body = np.array([[truth_msg.position.x, truth_msg.position.y, truth_msg.position.z]], dtype=np.float64)
        return transform_points(point_body, self.camera_T_body)[0]

    def _build_visible_cluster(self, truth_msg):
        center_camera = self._truth_center_body_to_camera(truth_msg)
        return self.foreground_offsets_camera + center_camera.reshape((1, 3))

    def _make_target_msg(self, label, cluster_camera_xyz):
        pixels_uv, valid_depth = project_camera_points_to_pixels(cluster_camera_xyz, self.intrinsics, min_depth_m=0.1)
        in_image = (
            valid_depth
            & np.isfinite(pixels_uv[:, 0])
            & np.isfinite(pixels_uv[:, 1])
            & (pixels_uv[:, 0] >= 0.0)
            & (pixels_uv[:, 0] < float(self.intrinsics["image_width"]))
            & (pixels_uv[:, 1] >= 0.0)
            & (pixels_uv[:, 1] < float(self.intrinsics["image_height"]))
        )

        if int(np.count_nonzero(in_image)) < self.min_projected_points:
            return self._invalid_target_msg(label)

        points_uv = pixels_uv[in_image]
        x_min = float(np.min(points_uv[:, 0]))
        x_max = float(np.max(points_uv[:, 0]))
        y_min = float(np.min(points_uv[:, 1]))
        y_max = float(np.max(points_uv[:, 1]))

        msg = Target2D()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.intrinsics["camera_frame"]
        msg.track_id = 1
        msg.valid = True
        msg.confidence = 0.96
        msg.cx = ((x_min + x_max) * 0.5) / float(self.intrinsics["image_width"])
        msg.cy = ((y_min + y_max) * 0.5) / float(self.intrinsics["image_height"])
        msg.width = ((x_max - x_min) / float(self.intrinsics["image_width"])) * self.target_bbox_padding
        msg.height = ((y_max - y_min) / float(self.intrinsics["image_height"])) * self.target_bbox_padding
        msg.source = "truth_scene:%s" % label
        return msg

    def _build_obstacle_depth_map(self, obstacle_camera_xyz):
        if obstacle_camera_xyz is None or obstacle_camera_xyz.shape[0] == 0:
            return {}

        pixels_uv, valid_depth = project_camera_points_to_pixels(
            obstacle_camera_xyz,
            self.intrinsics,
            min_depth_m=0.1,
        )
        depth_map = {}
        image_width = int(self.intrinsics["image_width"])
        image_height = int(self.intrinsics["image_height"])
        for point_index, is_valid in enumerate(valid_depth):
            if not is_valid:
                continue
            pixel_x = int(round(float(pixels_uv[point_index, 0])))
            pixel_y = int(round(float(pixels_uv[point_index, 1])))
            if pixel_x < 0 or pixel_x >= image_width or pixel_y < 0 or pixel_y >= image_height:
                continue
            depth_value = float(obstacle_camera_xyz[point_index, 2])
            key = (pixel_x, pixel_y)
            old_depth = depth_map.get(key)
            if old_depth is None or depth_value < old_depth:
                depth_map[key] = depth_value
        return depth_map

    def _filter_cluster_by_static_occlusion(self, cluster_camera_xyz, obstacle_camera_xyz):
        if cluster_camera_xyz is None or cluster_camera_xyz.shape[0] == 0:
            return cluster_camera_xyz
        if obstacle_camera_xyz is None or obstacle_camera_xyz.shape[0] == 0:
            return cluster_camera_xyz

        obstacle_depth_map = self._build_obstacle_depth_map(obstacle_camera_xyz)
        if not obstacle_depth_map:
            return cluster_camera_xyz

        pixels_uv, valid_depth = project_camera_points_to_pixels(
            cluster_camera_xyz,
            self.intrinsics,
            min_depth_m=0.1,
        )
        image_width = int(self.intrinsics["image_width"])
        image_height = int(self.intrinsics["image_height"])
        keep_mask = np.zeros((cluster_camera_xyz.shape[0],), dtype=bool)

        for point_index, is_valid in enumerate(valid_depth):
            if not is_valid:
                continue
            pixel_x = int(round(float(pixels_uv[point_index, 0])))
            pixel_y = int(round(float(pixels_uv[point_index, 1])))
            if pixel_x < 0 or pixel_x >= image_width or pixel_y < 0 or pixel_y >= image_height:
                continue

            point_depth = float(cluster_camera_xyz[point_index, 2])
            nearest_obstacle_depth = None
            for offset_x in range(-self.occlusion_pixel_radius_px, self.occlusion_pixel_radius_px + 1):
                for offset_y in range(-self.occlusion_pixel_radius_px, self.occlusion_pixel_radius_px + 1):
                    neighbor_x = pixel_x + offset_x
                    neighbor_y = pixel_y + offset_y
                    if neighbor_x < 0 or neighbor_x >= image_width or neighbor_y < 0 or neighbor_y >= image_height:
                        continue
                    obstacle_depth = obstacle_depth_map.get((neighbor_x, neighbor_y))
                    if obstacle_depth is None:
                        continue
                    if nearest_obstacle_depth is None or obstacle_depth < nearest_obstacle_depth:
                        nearest_obstacle_depth = obstacle_depth

            if nearest_obstacle_depth is None:
                keep_mask[point_index] = True
                continue

            # Keep only points that are still in front of the nearest static obstacle
            # at this pixel neighborhood; deeper points are occluded by the obstacle.
            if point_depth <= nearest_obstacle_depth - self.occlusion_depth_margin_m:
                keep_mask[point_index] = True

        return cluster_camera_xyz[keep_mask]

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        if not self._truth_is_fresh() or not self._phase_is_fresh():
            invalid = self._invalid_target_msg("waiting_for_truth")
            try:
                self.pointcloud_pub.publish(self._empty_pointcloud())
                self._publish_foreground_pointcloud(None)
                self._publish_obstacle_pointcloud(None)
                self.target_pub.publish(invalid)
                self.debug_target_pub.publish(invalid)
            except rospy.ROSException:
                return
            return

        label = self.last_phase.get("label", "") or "phase"
        if label != self.last_phase_label:
            rospy.loginfo(
                "truth_driven_scene phase=%s visible=%s truth_valid=%s",
                label,
                self.last_phase.get("visible", False),
                self.last_phase.get("truth_valid", False),
            )
            self.last_phase_label = label

        phase_visible = bool(self.last_phase.get("visible", False))
        tracking_visible = phase_visible if self.phase_visible_affects_tracking else True
        truth_valid = bool(self.last_truth.valid) and bool(self.last_phase.get("truth_valid", self.last_truth.valid))
        obstacle_camera_xyz = self._static_obstacles_camera_points()

        if not tracking_visible or not truth_valid:
            invalid = self._invalid_target_msg(label)
            try:
                self.pointcloud_pub.publish(self._make_pointcloud_msg(obstacle_camera_xyz))
                self._publish_foreground_pointcloud(None)
                self._publish_obstacle_pointcloud(obstacle_camera_xyz)
                self.target_pub.publish(invalid)
                self.debug_target_pub.publish(invalid)
            except rospy.ROSException:
                return
            return

        cluster_camera_xyz = self._build_visible_cluster(self.last_truth)
        cluster_camera_xyz = self._filter_cluster_by_static_occlusion(cluster_camera_xyz, obstacle_camera_xyz)
        target_msg = self._make_target_msg(label, cluster_camera_xyz)

        if not target_msg.valid:
            try:
                self.pointcloud_pub.publish(self._make_pointcloud_msg(obstacle_camera_xyz))
                self._publish_foreground_pointcloud(cluster_camera_xyz)
                self._publish_obstacle_pointcloud(obstacle_camera_xyz)
                self.target_pub.publish(target_msg)
                self.debug_target_pub.publish(target_msg)
            except rospy.ROSException:
                return
            return

        scene_camera_xyz = cluster_camera_xyz
        if obstacle_camera_xyz.shape[0] > 0:
            scene_camera_xyz = np.vstack((cluster_camera_xyz, obstacle_camera_xyz))

        try:
            self.pointcloud_pub.publish(self._make_pointcloud_msg(scene_camera_xyz))
            self._publish_foreground_pointcloud(cluster_camera_xyz)
            self._publish_obstacle_pointcloud(obstacle_camera_xyz)
            self.target_pub.publish(target_msg)
            self.debug_target_pub.publish(target_msg)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("truth_driven_scene_publisher")
    TruthDrivenScenePublisherNode()
    rospy.spin()
