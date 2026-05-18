#!/usr/bin/env python3
import math

import numpy as np
import yaml


def _matrix_from_yaml(matrix_dict, expected_shape):
    rows = int(matrix_dict["rows"])
    cols = int(matrix_dict["cols"])
    data = matrix_dict["data"]
    matrix = np.array(data, dtype=np.float64).reshape((rows, cols))
    if matrix.shape != expected_shape:
        raise ValueError("matrix shape mismatch: got %s expected %s" % (matrix.shape, expected_shape))
    return matrix


def _normalize_quaternion_xyzw(quaternion_xyzw):
    q = np.array(quaternion_xyzw, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    return q / norm


def _build_transform_matrix_from_yaml(data):
    rowmajor = data.get("transform_matrix_rowmajor")
    if rowmajor:
        return np.array(rowmajor, dtype=np.float64).reshape((4, 4))

    translation = np.array(data.get("translation_xyz_m", [0.0, 0.0, 0.0]), dtype=np.float64).reshape((3,))
    if "quaternion_xyzw" in data:
        rotation = quaternion_xyzw_to_rotation_matrix(data["quaternion_xyzw"])
    else:
        rotation = rpy_deg_to_rotation_matrix(data.get("rotation_rpy_deg", [0.0, 0.0, 0.0]))

    transform_matrix = np.eye(4, dtype=np.float64)
    transform_matrix[:3, :3] = rotation
    transform_matrix[:3, 3] = translation
    return transform_matrix


def quaternion_xyzw_to_rotation_matrix(quaternion_xyzw):
    x, y, z, w = _normalize_quaternion_xyzw(quaternion_xyzw)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def rpy_deg_to_rotation_matrix(rotation_rpy_deg):
    roll, pitch, yaw = [math.radians(float(v)) for v in rotation_rpy_deg]

    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    rot_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rot_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rot_z @ rot_y @ rot_x


def load_camera_intrinsics_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    camera_matrix = _matrix_from_yaml(data["camera_matrix"], (3, 3))
    return {
        "camera_frame": data.get("camera_frame", "camera"),
        "image_width": int(data["image_width"]),
        "image_height": int(data["image_height"]),
        "distortion_model": data.get("distortion_model", "plumb_bob"),
        "camera_matrix": camera_matrix,
        "distortion_coefficients": np.array(data["distortion_coefficients"]["data"], dtype=np.float64),
        "rectification_matrix": _matrix_from_yaml(data["rectification_matrix"], (3, 3)),
        "projection_matrix": _matrix_from_yaml(data["projection_matrix"], (3, 4)),
        "fx": float(camera_matrix[0, 0]),
        "fy": float(camera_matrix[1, 1]),
        "cx": float(camera_matrix[0, 2]),
        "cy": float(camera_matrix[1, 2]),
    }


def load_camera_lidar_extrinsics_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    transform_matrix = _build_transform_matrix_from_yaml(data)

    return {
        "camera_frame": data.get("camera_frame", "camera"),
        "lidar_frame": data.get("lidar_frame", "mid360"),
        "base_frame": data.get("base_frame", "base_link"),
        "camera_T_lidar": transform_matrix,
    }


def load_rigid_transform_yaml(path, target_frame_key="target_frame", source_frame_key="source_frame"):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    return {
        "target_frame": data.get(target_frame_key, "target"),
        "source_frame": data.get(source_frame_key, "source"),
        "target_T_source": _build_transform_matrix_from_yaml(data),
    }


def transform_points(points_xyz, transform_matrix):
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape Nx3")

    ones = np.ones((points.shape[0], 1), dtype=np.float64)
    homogeneous = np.hstack((points, ones))
    transformed = (transform_matrix @ homogeneous.T).T
    return transformed[:, :3]


def project_camera_points_to_pixels(points_camera_xyz, intrinsics, min_depth_m=1e-3):
    points = np.asarray(points_camera_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("camera points must have shape Nx3")

    z = points[:, 2]
    valid_depth = z > float(min_depth_m)
    pixels = np.full((points.shape[0], 2), np.nan, dtype=np.float64)

    if np.any(valid_depth):
        pixels[valid_depth, 0] = intrinsics["fx"] * points[valid_depth, 0] / z[valid_depth] + intrinsics["cx"]
        pixels[valid_depth, 1] = intrinsics["fy"] * points[valid_depth, 1] / z[valid_depth] + intrinsics["cy"]

    return pixels, valid_depth


def project_lidar_points_to_image(points_lidar_xyz, intrinsics, extrinsics, min_depth_m=1e-3):
    points_camera = transform_points(points_lidar_xyz, extrinsics["camera_T_lidar"])
    pixels, valid_depth = project_camera_points_to_pixels(points_camera, intrinsics, min_depth_m=min_depth_m)

    width = intrinsics["image_width"]
    height = intrinsics["image_height"]
    in_bounds = (
        valid_depth
        & (pixels[:, 0] >= 0.0)
        & (pixels[:, 0] < float(width))
        & (pixels[:, 1] >= 0.0)
        & (pixels[:, 1] < float(height))
    )

    return {
        "points_camera_xyz": points_camera,
        "pixels_uv": pixels,
        "valid_depth_mask": valid_depth,
        "in_image_mask": in_bounds,
    }


def normalized_bbox_to_pixel_bounds(target_2d, image_width, image_height):
    bbox_width_px = float(target_2d["width"]) * float(image_width)
    bbox_height_px = float(target_2d["height"]) * float(image_height)
    center_x_px = float(target_2d["cx"]) * float(image_width)
    center_y_px = float(target_2d["cy"]) * float(image_height)

    x_min = center_x_px - 0.5 * bbox_width_px
    x_max = center_x_px + 0.5 * bbox_width_px
    y_min = center_y_px - 0.5 * bbox_height_px
    y_max = center_y_px + 0.5 * bbox_height_px
    return x_min, y_min, x_max, y_max


def mask_points_inside_normalized_bbox(pixels_uv, target_2d, image_width, image_height):
    pixels = np.asarray(pixels_uv, dtype=np.float64)
    if pixels.ndim != 2 or pixels.shape[1] != 2:
        raise ValueError("pixels must have shape Nx2")

    x_min, y_min, x_max, y_max = normalized_bbox_to_pixel_bounds(target_2d, image_width, image_height)
    return (
        np.isfinite(pixels[:, 0])
        & np.isfinite(pixels[:, 1])
        & (pixels[:, 0] >= x_min)
        & (pixels[:, 0] <= x_max)
        & (pixels[:, 1] >= y_min)
        & (pixels[:, 1] <= y_max)
    )


def select_points_in_normalized_bbox(points_camera_xyz, pixels_uv, target_2d, image_width, image_height, valid_mask=None):
    points = np.asarray(points_camera_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("camera points must have shape Nx3")

    bbox_mask = mask_points_inside_normalized_bbox(pixels_uv, target_2d, image_width, image_height)
    if valid_mask is not None:
        bbox_mask = bbox_mask & np.asarray(valid_mask, dtype=bool)

    return {
        "mask": bbox_mask,
        "points_camera_xyz": points[bbox_mask],
        "pixels_uv": np.asarray(pixels_uv, dtype=np.float64)[bbox_mask],
    }


def frontmost_depth_inlier_mask(points_camera_xyz, depth_percentile=0.35, depth_band_m=1.0):
    points = np.asarray(points_camera_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("camera points must have shape Nx3")
    if points.shape[0] == 0:
        return np.zeros((0,), dtype=bool), None

    depth_values = points[:, 2]
    anchor_depth = float(np.percentile(depth_values, float(depth_percentile) * 100.0))
    inlier_mask = (depth_values >= anchor_depth - 1e-6) & (depth_values <= anchor_depth + float(depth_band_m))
    return inlier_mask, anchor_depth


def robust_cluster_center(points_xyz):
    points = np.asarray(points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape Nx3")
    if points.shape[0] == 0:
        raise ValueError("cannot estimate center from empty points")
    return np.median(points, axis=0)
