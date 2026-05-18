#!/usr/bin/env python3
import os
import sys

import numpy as np
import rospy

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


class AssociationSanityNode:
    def __init__(self):
        self.intrinsics_yaml = rospy.get_param("~intrinsics_yaml")
        self.extrinsics_yaml = rospy.get_param("~extrinsics_yaml")
        self.depth_band_m = float(rospy.get_param("~depth_band_m", 1.0))
        self.depth_percentile = float(rospy.get_param("~depth_percentile", 0.35))
        self.min_selected_points = int(rospy.get_param("~min_selected_points", 12))
        self.min_inlier_ratio = float(rospy.get_param("~min_inlier_ratio", 0.45))
        self.max_center_error_m = float(rospy.get_param("~max_center_error_m", 0.15))
        self.exit_code = 0

        self._run()

    def _fail(self, reason):
        rospy.logerr("association sanity FAIL reason=%s", reason)
        self.exit_code = 1
        rospy.signal_shutdown(reason)

    def _pass(self, message):
        rospy.loginfo("association sanity PASS %s", message)
        self.exit_code = 0
        rospy.signal_shutdown("association sanity pass")

    def _assert(self, condition, reason):
        if not condition:
            raise RuntimeError(reason)

    def _make_foreground_cluster_camera(self):
        x_values = np.array([0.10, 0.40, 0.70], dtype=np.float64)
        y_values = np.array([-0.60, 0.00, 0.60], dtype=np.float64)
        z_value = 6.0

        cluster = []
        for y in y_values:
            for x in x_values:
                cluster.append([x, y, z_value])
        return np.array(cluster, dtype=np.float64)

    def _make_background_cluster_camera(self):
        foreground = self._make_foreground_cluster_camera()
        scale = 10.0 / 6.0
        background = foreground.copy()
        background[:, 0] *= scale
        background[:, 1] *= scale
        background[:, 2] = 10.0
        return background

    def _camera_to_lidar(self, points_camera, camera_T_lidar):
        lidar_T_camera = np.linalg.inv(camera_T_lidar)
        return (lidar_T_camera[:3, :3] @ points_camera.T).T + lidar_T_camera[:3, 3]

    def _run(self):
        try:
            self._assert(os.path.exists(self.intrinsics_yaml), "intrinsics yaml missing")
            self._assert(os.path.exists(self.extrinsics_yaml), "extrinsics yaml missing")

            intrinsics = load_camera_intrinsics_yaml(self.intrinsics_yaml)
            extrinsics = load_camera_lidar_extrinsics_yaml(self.extrinsics_yaml)

            foreground_camera = self._make_foreground_cluster_camera()
            background_camera = self._make_background_cluster_camera()
            all_camera = np.vstack((foreground_camera, background_camera))
            all_lidar = self._camera_to_lidar(all_camera, extrinsics["camera_T_lidar"])

            projection = project_lidar_points_to_image(all_lidar, intrinsics, extrinsics)
            pixels = projection["pixels_uv"]
            valid_mask = projection["in_image_mask"]

            foreground_pixels = pixels[: len(foreground_camera)]
            x_min = float(np.min(foreground_pixels[:, 0]))
            x_max = float(np.max(foreground_pixels[:, 0]))
            y_min = float(np.min(foreground_pixels[:, 1]))
            y_max = float(np.max(foreground_pixels[:, 1]))
            bbox = {
                "cx": ((x_min + x_max) * 0.5) / float(intrinsics["image_width"]),
                "cy": ((y_min + y_max) * 0.5) / float(intrinsics["image_height"]),
                "width": ((x_max - x_min) / float(intrinsics["image_width"])) * 1.15,
                "height": ((y_max - y_min) / float(intrinsics["image_height"])) * 1.15,
            }

            selected = select_points_in_normalized_bbox(
                projection["points_camera_xyz"],
                pixels,
                bbox,
                intrinsics["image_width"],
                intrinsics["image_height"],
                valid_mask=valid_mask,
            )
            selected_points = selected["points_camera_xyz"]
            self._assert(selected_points.shape[0] >= self.min_selected_points, "too few selected points")

            inlier_mask, anchor_depth = frontmost_depth_inlier_mask(
                selected_points, depth_percentile=self.depth_percentile, depth_band_m=self.depth_band_m
            )
            inlier_points = selected_points[inlier_mask]
            inlier_ratio = float(inlier_points.shape[0]) / float(selected_points.shape[0])
            self._assert(inlier_ratio >= self.min_inlier_ratio, "inlier ratio too low: %.3f" % inlier_ratio)

            estimated_center = robust_cluster_center(inlier_points)
            true_center = np.median(foreground_camera, axis=0)
            center_error = float(np.linalg.norm(estimated_center - true_center))
            self._assert(center_error <= self.max_center_error_m, "center error too large: %.3f" % center_error)

            message = (
                "selected=%d inliers=%d inlier_ratio=%.3f anchor_depth=%.3f center_error_m=%.3f"
                % (
                    int(selected_points.shape[0]),
                    int(inlier_points.shape[0]),
                    inlier_ratio,
                    float(anchor_depth),
                    center_error,
                )
            )
            self._pass(message)
        except Exception as exc:  # pylint: disable=broad-except
            self._fail(str(exc))


def main():
    rospy.init_node("association_sanity_node")
    node = AssociationSanityNode()
    rospy.spin()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
