#!/usr/bin/env python3
import math
import os
import sys

import numpy as np
import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import Target2D, Target3D

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from projection_math import (  # noqa: E402
    frontmost_depth_inlier_mask,
    load_camera_intrinsics_yaml,
    load_camera_lidar_extrinsics_yaml,
    project_lidar_points_to_image,
    robust_cluster_center,
    select_points_in_normalized_bbox,
)


class TargetFusionNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/tracker/selected_target")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/follow/lidar/points")
        self.output_topic = rospy.get_param("~output_topic", "/follow/fusion/target_body")
        self.output_frame_id = rospy.get_param("~output_frame_id", "base_link")
        self.enable_mock_projection = rospy.get_param("~enable_mock_projection", True)
        self.mock_forward_depth_m = rospy.get_param("~mock_forward_depth_m", 6.0)
        self.mock_lateral_scale_m = rospy.get_param("~mock_lateral_scale_m", 2.0)
        self.mock_support_points = rospy.get_param("~mock_support_points", 12)
        self.intrinsics_yaml = rospy.get_param("~intrinsics_yaml", "")
        self.extrinsics_yaml = rospy.get_param("~extrinsics_yaml", "")
        self.pointcloud_timeout_sec = float(rospy.get_param("~pointcloud_timeout_sec", 0.3))
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.1))
        self.min_support_points = int(rospy.get_param("~min_support_points", 6))
        self.depth_percentile = float(rospy.get_param("~depth_percentile", 0.35))
        self.depth_band_m = float(rospy.get_param("~depth_band_m", 1.0))
        self.max_points_to_process = int(rospy.get_param("~max_points_to_process", 30000))

        self.last_target = None
        self.last_pointcloud = None
        self.last_pointcloud_stamp = None
        self.intrinsics = None
        self.extrinsics = None
        self.resolved_output_frame_id = self.output_frame_id

        if not self.enable_mock_projection:
            self._load_calibration()

        self.publisher = rospy.Publisher(self.output_topic, Target3D, queue_size=10)
        self.subscriber = rospy.Subscriber(self.input_topic, Target2D, self._target_callback, queue_size=10)
        self.pointcloud_subscriber = rospy.Subscriber(
            self.pointcloud_topic, PointCloud2, self._pointcloud_callback, queue_size=1
        )

        rospy.loginfo(
            "human_follow_target_fusion ready: input=%s pointcloud=%s output=%s mock=%s",
            self.input_topic,
            self.pointcloud_topic,
            self.output_topic,
            self.enable_mock_projection,
        )

    def _load_calibration(self):
        if not self.intrinsics_yaml or not self.extrinsics_yaml:
            rospy.logerr("fusion real mode requires intrinsics_yaml and extrinsics_yaml")
            return

        try:
            self.intrinsics = load_camera_intrinsics_yaml(self.intrinsics_yaml)
            self.extrinsics = load_camera_lidar_extrinsics_yaml(self.extrinsics_yaml)
            camera_frame = self.intrinsics["camera_frame"]
            if self.output_frame_id != camera_frame:
                rospy.logwarn(
                    "fusion real mode overrides output frame from %s to %s because camera-to-body transform is not defined yet",
                    self.output_frame_id,
                    camera_frame,
                )
            self.resolved_output_frame_id = camera_frame
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logerr("failed to load fusion calibration: %s", exc)
            self.intrinsics = None
            self.extrinsics = None

    def _pointcloud_callback(self, msg):
        self.last_pointcloud = msg
        self.last_pointcloud_stamp = rospy.Time.now()

    def _make_default_output(self, track_id):
        out = Target3D()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.resolved_output_frame_id
        out.track_id = track_id
        out.frame_id = self.resolved_output_frame_id
        out.valid = False
        out.confidence = 0.0
        out.position.x = 0.0
        out.position.y = 0.0
        out.position.z = 0.0
        out.support_points = 0
        return out

    def _pointcloud_is_fresh(self):
        if self.last_pointcloud is None or self.last_pointcloud_stamp is None:
            return False
        return (rospy.Time.now() - self.last_pointcloud_stamp).to_sec() <= self.pointcloud_timeout_sec

    def _pointcloud_to_xyz_array(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float64)
        if points.size == 0:
            return np.zeros((0, 3), dtype=np.float64)
        if points.ndim == 1:
            points = points.reshape((1, 3))

        if self.max_points_to_process > 0 and points.shape[0] > self.max_points_to_process:
            stride = int(math.ceil(float(points.shape[0]) / float(self.max_points_to_process)))
            points = points[:: max(1, stride)]
        return points

    def _fuse_from_pointcloud(self, msg, out):
        if self.intrinsics is None or self.extrinsics is None:
            rospy.logwarn_throttle(5.0, "fusion real mode has no loaded calibration")
            return out
        if not self._pointcloud_is_fresh():
            rospy.logwarn_throttle(2.0, "fusion real mode waiting for fresh pointcloud")
            return out

        lidar_points = self._pointcloud_to_xyz_array(self.last_pointcloud)
        if lidar_points.shape[0] == 0:
            rospy.loginfo_throttle(2.0, "fusion real mode got empty pointcloud during non-visible or sparse phase")
            return out

        target_bbox = {
            "cx": float(msg.cx),
            "cy": float(msg.cy),
            "width": float(msg.width),
            "height": float(msg.height),
        }

        projection = project_lidar_points_to_image(
            lidar_points,
            self.intrinsics,
            self.extrinsics,
            min_depth_m=self.min_depth_m,
        )
        selected = select_points_in_normalized_bbox(
            projection["points_camera_xyz"],
            projection["pixels_uv"],
            target_bbox,
            self.intrinsics["image_width"],
            self.intrinsics["image_height"],
            valid_mask=projection["in_image_mask"],
        )
        selected_points = selected["points_camera_xyz"]
        if selected_points.shape[0] < self.min_support_points:
            rospy.logwarn_throttle(
                2.0,
                "fusion bbox selection too sparse: selected=%d min=%d",
                selected_points.shape[0],
                self.min_support_points,
            )
            return out

        inlier_mask, _anchor_depth = frontmost_depth_inlier_mask(
            selected_points, depth_percentile=self.depth_percentile, depth_band_m=self.depth_band_m
        )
        inlier_points = selected_points[inlier_mask]
        if inlier_points.shape[0] < self.min_support_points:
            rospy.logwarn_throttle(
                2.0,
                "fusion depth inliers too sparse: inliers=%d min=%d",
                inlier_points.shape[0],
                self.min_support_points,
            )
            return out

        center = robust_cluster_center(inlier_points)
        out.valid = True
        out.confidence = msg.confidence
        out.position.x = float(center[0])
        out.position.y = float(center[1])
        out.position.z = float(center[2])
        out.support_points = int(inlier_points.shape[0])
        out.header.frame_id = self.resolved_output_frame_id
        out.frame_id = self.resolved_output_frame_id
        return out

    def _target_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.last_target = msg
        out = self._make_default_output(msg.track_id)

        if msg.valid and self.enable_mock_projection:
            out.valid = True
            out.confidence = msg.confidence
            out.position.x = self.mock_forward_depth_m
            out.position.y = (0.5 - msg.cx) * 2.0 * self.mock_lateral_scale_m
            out.position.z = 0.0
            out.support_points = int(self.mock_support_points)
        elif msg.valid:
            out = self._fuse_from_pointcloud(msg, out)
        else:
            out.header.stamp = rospy.Time.now()

        try:
            self.publisher.publish(out)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("human_follow_target_fusion")
    TargetFusionNode()
    rospy.spin()
