#!/usr/bin/env python3
import os
import sys

import numpy as np
import rospy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from projection_math import (
    load_camera_intrinsics_yaml,
    load_camera_lidar_extrinsics_yaml,
    mask_points_inside_normalized_bbox,
    project_camera_points_to_pixels,
    project_lidar_points_to_image,
)


class ProjectionSanityNode:
    def __init__(self):
        self.intrinsics_yaml = rospy.get_param("~intrinsics_yaml")
        self.extrinsics_yaml = rospy.get_param("~extrinsics_yaml")
        self.max_center_pixel_error_px = float(rospy.get_param("~max_center_pixel_error_px", 1e-6))
        self.min_cluster_points = int(rospy.get_param("~min_cluster_points", 9))
        self.min_bbox_hit_ratio = float(rospy.get_param("~min_bbox_hit_ratio", 0.95))
        self.max_background_hit_ratio = float(rospy.get_param("~max_background_hit_ratio", 0.05))
        self.exit_code = 0

        self._run()

    def _fail(self, reason):
        rospy.logerr("projection sanity FAIL reason=%s", reason)
        self.exit_code = 1
        rospy.signal_shutdown(reason)

    def _pass(self, message):
        rospy.loginfo("projection sanity PASS %s", message)
        self.exit_code = 0
        rospy.signal_shutdown("projection sanity pass")

    def _assert(self, condition, reason):
        if not condition:
            raise RuntimeError(reason)

    def _make_cluster_camera_points(self):
        x_values = np.array([-0.30, 0.00, 0.30], dtype=np.float64)
        y_values = np.array([-0.60, 0.00, 0.60], dtype=np.float64)
        z_value = 6.0

        cluster = []
        for y in y_values:
            for x in x_values:
                cluster.append([x, y, z_value])
        return np.array(cluster, dtype=np.float64)

    def _make_background_camera_points(self):
        return np.array(
            [
                [2.4, 0.0, 6.0],
                [-2.4, 0.0, 6.0],
                [0.0, 1.9, 6.0],
                [0.0, -1.9, 6.0],
            ],
            dtype=np.float64,
        )

    def _run(self):
        try:
            self._assert(os.path.exists(self.intrinsics_yaml), "intrinsics yaml missing")
            self._assert(os.path.exists(self.extrinsics_yaml), "extrinsics yaml missing")

            intrinsics = load_camera_intrinsics_yaml(self.intrinsics_yaml)
            extrinsics = load_camera_lidar_extrinsics_yaml(self.extrinsics_yaml)

            center_camera = np.array([[0.0, 0.0, 6.0]], dtype=np.float64)
            center_pixels, center_valid = project_camera_points_to_pixels(center_camera, intrinsics)
            self._assert(bool(center_valid[0]), "center point invalid depth")
            expected_center = np.array([intrinsics["cx"], intrinsics["cy"]], dtype=np.float64)
            center_error = np.linalg.norm(center_pixels[0] - expected_center)
            self._assert(
                center_error <= self.max_center_pixel_error_px,
                "center pixel error too large: %.6f" % center_error,
            )

            cluster_camera = self._make_cluster_camera_points()
            background_camera = self._make_background_camera_points()
            all_camera = np.vstack((cluster_camera, background_camera))

            lidar_T_camera = np.linalg.inv(extrinsics["camera_T_lidar"])
            all_lidar = (lidar_T_camera[:3, :3] @ all_camera.T).T + lidar_T_camera[:3, 3]

            projection = project_lidar_points_to_image(all_lidar, intrinsics, extrinsics)
            projected_pixels = projection["pixels_uv"]
            in_image = projection["in_image_mask"]
            self._assert(int(np.sum(in_image[: len(cluster_camera)])) >= self.min_cluster_points, "cluster points projected out of image")

            cluster_pixels = projected_pixels[: len(cluster_camera)]
            background_pixels = projected_pixels[len(cluster_camera) :]
            self._assert(np.all(np.isfinite(cluster_pixels)), "cluster pixels contain non-finite values")

            x_min = float(np.min(cluster_pixels[:, 0]))
            x_max = float(np.max(cluster_pixels[:, 0]))
            y_min = float(np.min(cluster_pixels[:, 1]))
            y_max = float(np.max(cluster_pixels[:, 1]))

            width_norm = (x_max - x_min) / float(intrinsics["image_width"])
            height_norm = (y_max - y_min) / float(intrinsics["image_height"])
            bbox = {
                "cx": ((x_min + x_max) * 0.5) / float(intrinsics["image_width"]),
                "cy": ((y_min + y_max) * 0.5) / float(intrinsics["image_height"]),
                "width": width_norm * 1.10,
                "height": height_norm * 1.10,
            }

            cluster_mask = mask_points_inside_normalized_bbox(
                cluster_pixels, bbox, intrinsics["image_width"], intrinsics["image_height"]
            )
            background_mask = mask_points_inside_normalized_bbox(
                background_pixels, bbox, intrinsics["image_width"], intrinsics["image_height"]
            )

            cluster_hit_ratio = float(np.mean(cluster_mask))
            background_hit_ratio = float(np.mean(background_mask))
            self._assert(
                cluster_hit_ratio >= self.min_bbox_hit_ratio,
                "cluster bbox hit ratio too low: %.3f" % cluster_hit_ratio,
            )
            self._assert(
                background_hit_ratio <= self.max_background_hit_ratio,
                "background bbox hit ratio too high: %.3f" % background_hit_ratio,
            )

            message = (
                "center_error_px=%.6f cluster_hit_ratio=%.3f background_hit_ratio=%.3f image_points=%d"
                % (center_error, cluster_hit_ratio, background_hit_ratio, int(np.sum(in_image)))
            )
            self._pass(message)
        except Exception as exc:  # pylint: disable=broad-except
            self._fail(str(exc))


def main():
    rospy.init_node("projection_sanity_node")
    node = ProjectionSanityNode()
    rospy.spin()
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
