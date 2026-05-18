#!/usr/bin/env python3
import argparse
import json
import os
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from human_follow_msgs.msg import FollowState, Target2D, Target3D, TrackerStatus

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FUSION_SCRIPT_DIR = os.path.abspath(
    os.path.join(SCRIPT_DIR, "..", "..", "src", "human_follow_fusion", "scripts")
)
if FUSION_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, FUSION_SCRIPT_DIR)

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


def pointcloud2_to_xyz(msg):
    points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float64)
    if points.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if points.ndim == 1:
        points = points.reshape((1, 3))
    return points


def build_foreground_offsets(person_width_m, person_height_m, person_depth_m, person_columns, person_rows):
    x_offsets = np.linspace(-0.5 * person_width_m, 0.5 * person_width_m, max(2, int(person_columns)))
    y_offsets = np.linspace(-0.5 * person_height_m, 0.5 * person_height_m, max(2, int(person_rows)))
    z_offsets = np.linspace(-0.5 * person_depth_m, 0.5 * person_depth_m, 2)

    offsets = []
    for y_value in y_offsets:
        for x_value in x_offsets:
            for z_value in z_offsets:
                offsets.append([x_value, y_value, z_value])
    return np.asarray(offsets, dtype=np.float64)


class SceneReplica:
    def __init__(self):
        intrinsics_yaml = rospy.get_param("/truth_driven_scene_publisher/intrinsics_yaml")
        extrinsics_yaml = rospy.get_param("/truth_driven_scene_publisher/extrinsics_yaml")
        body_yaml = rospy.get_param("/truth_driven_scene_publisher/camera_body_yaml")

        self.intrinsics = load_camera_intrinsics_yaml(intrinsics_yaml)
        self.extrinsics = load_camera_lidar_extrinsics_yaml(extrinsics_yaml)
        self.camera_T_body = np.linalg.inv(load_rigid_transform_yaml(body_yaml)["target_T_source"])
        self.camera_T_lidar = self.extrinsics["camera_T_lidar"]

        self.truth_timeout_sec = float(rospy.get_param("/truth_driven_scene_publisher/truth_timeout_sec", 0.5))
        self.min_projected_points = int(rospy.get_param("/truth_driven_scene_publisher/min_projected_points", 6))
        self.phase_visible_affects_tracking = bool(
            rospy.get_param("/truth_driven_scene_publisher/phase_visible_affects_tracking", True)
        )
        self.occlusion_depth_margin_m = max(
            float(rospy.get_param("/truth_driven_scene_publisher/occlusion_depth_margin_m", 0.10)),
            0.0,
        )
        self.occlusion_pixel_radius_px = max(
            int(rospy.get_param("/truth_driven_scene_publisher/occlusion_pixel_radius_px", 3)),
            0,
        )
        self.foreground_offsets_camera = build_foreground_offsets(
            float(rospy.get_param("/truth_driven_scene_publisher/person_width_m", 0.60)),
            float(rospy.get_param("/truth_driven_scene_publisher/person_height_m", 1.70)),
            float(rospy.get_param("/truth_driven_scene_publisher/person_depth_m", 0.18)),
            int(rospy.get_param("/truth_driven_scene_publisher/person_columns", 4)),
            int(rospy.get_param("/truth_driven_scene_publisher/person_rows", 6)),
        )

        self.last_truth = None
        self.last_truth_time = None
        self.last_phase = None
        self.last_phase_time = None
        self.last_detector = None
        self.last_detector_time = None
        self.last_tracker = None
        self.last_tracker_time = None
        self.last_stage2 = None
        self.last_stage2_time = None
        self.last_odom = None
        self.last_odom_time = None
        self.last_foreground_cloud = None
        self.last_obstacle_cloud = None

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        rospy.Subscriber("/follow/sim/truth_target_body", Target3D, self._truth_callback, queue_size=50)
        rospy.Subscriber("/follow/sim/truth_phase", String, self._phase_callback, queue_size=50)
        rospy.Subscriber("/follow/detector/person_target", Target2D, self._detector_callback, queue_size=50)
        rospy.Subscriber("/follow/tracker/status", TrackerStatus, self._tracker_callback, queue_size=50)
        rospy.Subscriber("/follow/stage2/state", FollowState, self._stage2_callback, queue_size=50)
        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_callback, queue_size=50)
        rospy.Subscriber("/follow/lidar/foreground_points", PointCloud2, self._foreground_callback, queue_size=10)
        rospy.Subscriber("/follow/lidar/obstacle_points", PointCloud2, self._obstacle_callback, queue_size=10)

    def _truth_callback(self, msg):
        self.last_truth = msg
        self.last_truth_time = time.time()

    def _phase_callback(self, msg):
        self.last_phase = parse_phase_descriptor(msg.data)
        self.last_phase_time = time.time()

    def _detector_callback(self, msg):
        self.last_detector = msg
        self.last_detector_time = time.time()

    def _tracker_callback(self, msg):
        self.last_tracker = msg
        self.last_tracker_time = time.time()

    def _stage2_callback(self, msg):
        self.last_stage2 = msg
        self.last_stage2_time = time.time()

    def _odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_time = time.time()

    def _foreground_callback(self, msg):
        self.last_foreground_cloud = msg

    def _obstacle_callback(self, msg):
        self.last_obstacle_cloud = msg

    def wait_ready(self, timeout_sec):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_odom is not None:
                return True
            time.sleep(0.05)
        return False

    def publish_goal(self, goal_x, goal_y, goal_z):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(goal_x)
        msg.pose.position.y = float(goal_y)
        msg.pose.position.z = float(goal_z)
        msg.pose.orientation.w = 1.0
        for _ in range(3):
            self.goal_pub.publish(msg)
            time.sleep(0.05)

    def _truth_is_fresh(self, now_sec):
        if self.last_truth_time is None:
            return False
        return (now_sec - self.last_truth_time) <= self.truth_timeout_sec

    def _phase_is_fresh(self, now_sec):
        if self.last_phase_time is None:
            return False
        return (now_sec - self.last_phase_time) <= self.truth_timeout_sec

    def _static_obstacles_camera_points(self):
        if self.last_obstacle_cloud is None:
            return np.zeros((0, 3), dtype=np.float64)
        lidar_points = pointcloud2_to_xyz(self.last_obstacle_cloud)
        if lidar_points.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return transform_points(lidar_points, self.camera_T_lidar)

    def _build_visible_cluster(self):
        point_body = np.array(
            [[self.last_truth.position.x, self.last_truth.position.y, self.last_truth.position.z]],
            dtype=np.float64,
        )
        center_camera = transform_points(point_body, self.camera_T_body)[0]
        return self.foreground_offsets_camera + center_camera.reshape((1, 3)), center_camera

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

            if point_depth <= nearest_obstacle_depth - self.occlusion_depth_margin_m:
                keep_mask[point_index] = True

        return cluster_camera_xyz[keep_mask]

    def _in_image_count(self, points_camera_xyz):
        if points_camera_xyz is None or points_camera_xyz.shape[0] == 0:
            return 0
        pixels_uv, valid_depth = project_camera_points_to_pixels(
            points_camera_xyz,
            self.intrinsics,
            min_depth_m=0.1,
        )
        in_image = (
            valid_depth
            & np.isfinite(pixels_uv[:, 0])
            & np.isfinite(pixels_uv[:, 1])
            & (pixels_uv[:, 0] >= 0.0)
            & (pixels_uv[:, 0] < float(self.intrinsics["image_width"]))
            & (pixels_uv[:, 1] >= 0.0)
            & (pixels_uv[:, 1] < float(self.intrinsics["image_height"]))
        )
        return int(np.count_nonzero(in_image))

    def _foreground_count(self):
        if self.last_foreground_cloud is None:
            return 0
        return int(self.last_foreground_cloud.width)

    def _stage2_snapshot(self):
        if self.last_stage2 is None:
            return None
        return {
            "state_name": str(self.last_stage2.state_name),
            "detail": str(self.last_stage2.detail),
        }

    def _scene_snapshot(self, now_sec):
        truth_age_sec = None if self.last_truth_time is None else (now_sec - self.last_truth_time)
        phase_age_sec = None if self.last_phase_time is None else (now_sec - self.last_phase_time)
        detector_age_sec = None if self.last_detector_time is None else (now_sec - self.last_detector_time)
        tracking_visible = False
        truth_valid = False
        if self.last_phase is not None and self.last_truth is not None:
            phase_visible = bool(self.last_phase.get("visible", False))
            tracking_visible = phase_visible if self.phase_visible_affects_tracking else True
            truth_valid = bool(self.last_truth.valid) and bool(
                self.last_phase.get("truth_valid", self.last_truth.valid)
            )
        else:
            phase_visible = False

        reason = "waiting_for_truth"
        raw_count = 0
        filtered_count = 0
        center_camera_xyz = None
        obstacle_camera_count = 0

        if self.last_truth is not None and self.last_phase is not None:
            if self._truth_is_fresh(now_sec) and self._phase_is_fresh(now_sec):
                obstacle_camera_xyz = self._static_obstacles_camera_points()
                obstacle_camera_count = int(obstacle_camera_xyz.shape[0])
                raw_cluster, center_camera = self._build_visible_cluster()
                center_camera_xyz = [
                    round(float(center_camera[0]), 3),
                    round(float(center_camera[1]), 3),
                    round(float(center_camera[2]), 3),
                ]
                raw_count = self._in_image_count(raw_cluster)
                filtered_cluster = self._filter_cluster_by_static_occlusion(raw_cluster, obstacle_camera_xyz)
                filtered_count = self._in_image_count(filtered_cluster)

                if not tracking_visible or not truth_valid:
                    reason = "phase_or_truth_invalid"
                elif raw_count < self.min_projected_points:
                    reason = "out_of_image_or_behind_camera"
                elif filtered_count < self.min_projected_points:
                    reason = "static_occlusion_pruned"
                else:
                    reason = "should_be_valid"

        return {
            "reason": reason,
            "phase_visible": bool(phase_visible),
            "tracking_visible": bool(tracking_visible),
            "truth_valid": bool(truth_valid),
            "truth_age_sec": None if truth_age_sec is None else round(float(truth_age_sec), 3),
            "phase_age_sec": None if phase_age_sec is None else round(float(phase_age_sec), 3),
            "detector_age_sec": None if detector_age_sec is None else round(float(detector_age_sec), 3),
            "raw_in_image_count": int(raw_count),
            "filtered_in_image_count": int(filtered_count),
            "foreground_count": int(self._foreground_count()),
            "obstacle_camera_count": int(obstacle_camera_count),
            "center_camera_xyz": center_camera_xyz,
        }

    def run(self, goal_x, goal_y, goal_z, duration_sec):
        self.publish_goal(goal_x, goal_y, goal_z)
        truth_deadline = time.time() + 3.0
        while time.time() < truth_deadline and not rospy.is_shutdown():
            if self.last_truth is not None and self.last_phase is not None:
                break
            time.sleep(0.05)
        start_sec = time.time()
        prev_key = None
        first_detector_valid_t = None
        first_invalid_after_valid = None
        invalid_reason_counts = {}
        should_be_valid_but_invalid_count = 0
        events = []

        while (time.time() - start_sec) < duration_sec and not rospy.is_shutdown():
            now_sec = time.time()
            scene = self._scene_snapshot(now_sec)
            detector_valid = None if self.last_detector is None else bool(self.last_detector.valid)
            tracker_state = None if self.last_tracker is None else str(self.last_tracker.tracking_state)
            tracker_detail = None if self.last_tracker is None else str(self.last_tracker.detail)
            stage2 = self._stage2_snapshot()
            stage2_state = None if stage2 is None else stage2["state_name"]
            stage2_detail = None if stage2 is None else stage2["detail"]

            if detector_valid:
                if first_detector_valid_t is None:
                    first_detector_valid_t = now_sec - start_sec
            elif first_detector_valid_t is not None:
                invalid_reason_counts[scene["reason"]] = invalid_reason_counts.get(scene["reason"], 0) + 1
                if scene["reason"] == "should_be_valid":
                    should_be_valid_but_invalid_count += 1
                if first_invalid_after_valid is None:
                    first_invalid_after_valid = {
                        "t_sec": round(now_sec - start_sec, 3),
                        "reason": scene["reason"],
                        "scene": scene,
                        "tracker_state": tracker_state,
                        "tracker_detail": tracker_detail,
                        "stage2_state": stage2_state,
                        "stage2_detail": stage2_detail,
                    }

            key = (
                detector_valid,
                tracker_state,
                stage2_state,
                scene["reason"],
                scene["raw_in_image_count"],
                scene["filtered_in_image_count"],
                scene["foreground_count"],
            )
            if key != prev_key:
                events.append(
                    {
                        "t_sec": round(now_sec - start_sec, 3),
                        "detector_valid": detector_valid,
                        "detector_source": None if self.last_detector is None else str(self.last_detector.source),
                        "tracker_state": tracker_state,
                        "tracker_detail": tracker_detail,
                        "stage2_state": stage2_state,
                        "stage2_detail": stage2_detail,
                        "scene": scene,
                    }
                )
                prev_key = key

            time.sleep(0.05)

        return {
            "status": "ok",
            "goal_xyz": [round(float(goal_x), 3), round(float(goal_y), 3), round(float(goal_z), 3)],
            "duration_sec": round(float(duration_sec), 3),
            "min_projected_points": int(self.min_projected_points),
            "phase_visible_affects_tracking": bool(self.phase_visible_affects_tracking),
            "occlusion_depth_margin_m": round(float(self.occlusion_depth_margin_m), 3),
            "occlusion_pixel_radius_px": int(self.occlusion_pixel_radius_px),
            "first_detector_valid_t_sec": None
            if first_detector_valid_t is None
            else round(float(first_detector_valid_t), 3),
            "first_invalid_after_valid": first_invalid_after_valid,
            "invalid_reason_counts_after_first_valid": invalid_reason_counts,
            "should_be_valid_but_invalid_count": int(should_be_valid_but_invalid_count),
            "final_tracker_state": None if self.last_tracker is None else str(self.last_tracker.tracking_state),
            "final_stage2_state": None if self.last_stage2 is None else str(self.last_stage2.state_name),
            "final_stage2_detail": None if self.last_stage2 is None else str(self.last_stage2.detail),
            "events": events,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-x", type=float, default=10.5)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-z", type=float, default=0.0)
    parser.add_argument("--duration-sec", type=float, default=14.0)
    parser.add_argument("--wait-ready-timeout-sec", type=float, default=10.0)
    args = parser.parse_args()

    rospy.init_node("stage2_perception_drop_probe", anonymous=True)
    probe = SceneReplica()
    if not probe.wait_ready(args.wait_ready_timeout_sec):
        result = {"status": "failed", "reason": "wait_ready_timeout"}
    else:
        time.sleep(1.0)
        result = probe.run(args.goal_x, args.goal_y, args.goal_z, args.duration_sec)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
