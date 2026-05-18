#!/usr/bin/env python3
import argparse
import math
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from projection_math import (  # noqa: E402
    load_camera_intrinsics_yaml,
    load_camera_lidar_extrinsics_yaml,
    load_rigid_transform_yaml,
)


def _rotation_checks(name, rotation_matrix):
    orthogonality_error = np.linalg.norm(rotation_matrix.T @ rotation_matrix - np.eye(3), ord=np.inf)
    determinant = float(np.linalg.det(rotation_matrix))
    if orthogonality_error > 1e-3:
        raise ValueError("%s rotation matrix is not orthonormal enough: err=%.6f" % (name, orthogonality_error))
    if not math.isfinite(determinant) or abs(determinant - 1.0) > 1e-3:
        raise ValueError("%s rotation determinant invalid: det=%.6f" % (name, determinant))
    return orthogonality_error, determinant


def _translation_norm(transform_matrix):
    return float(np.linalg.norm(transform_matrix[:3, 3]))


def validate_assets(intrinsics_yaml, extrinsics_yaml, camera_body_yaml):
    intrinsics = load_camera_intrinsics_yaml(intrinsics_yaml)
    extrinsics = load_camera_lidar_extrinsics_yaml(extrinsics_yaml)
    camera_body = load_rigid_transform_yaml(camera_body_yaml)

    if intrinsics["image_width"] <= 0 or intrinsics["image_height"] <= 0:
        raise ValueError("camera image size must be positive")
    if intrinsics["fx"] <= 1e-6 or intrinsics["fy"] <= 1e-6:
        raise ValueError("camera focal length must be positive")

    distortion = intrinsics["distortion_coefficients"]
    if distortion.size not in (4, 5, 8):
        raise ValueError("unexpected distortion coefficient count: %d" % distortion.size)

    camera_frame = intrinsics["camera_frame"]
    if extrinsics["camera_frame"] != camera_frame:
        raise ValueError(
            "intrinsics camera_frame=%s does not match extrinsics camera_frame=%s"
            % (camera_frame, extrinsics["camera_frame"])
        )
    if camera_body["source_frame"] != camera_frame:
        raise ValueError(
            "camera_body source_frame=%s does not match intrinsics camera_frame=%s"
            % (camera_body["source_frame"], camera_frame)
        )
    if extrinsics["base_frame"] != camera_body["target_frame"]:
        raise ValueError(
            "extrinsics base_frame=%s does not match camera_body target_frame=%s"
            % (extrinsics["base_frame"], camera_body["target_frame"])
        )

    camera_t_lidar = extrinsics["camera_T_lidar"]
    target_t_source = camera_body["target_T_source"]
    camera_rot_err, camera_det = _rotation_checks("camera_T_lidar", camera_t_lidar[:3, :3])
    body_rot_err, body_det = _rotation_checks("target_T_source", target_t_source[:3, :3])

    metrics = {
        "camera_frame": camera_frame,
        "lidar_frame": extrinsics["lidar_frame"],
        "base_frame": extrinsics["base_frame"],
        "image_width": intrinsics["image_width"],
        "image_height": intrinsics["image_height"],
        "fx": intrinsics["fx"],
        "fy": intrinsics["fy"],
        "distortion_count": int(distortion.size),
        "camera_lidar_translation_norm_m": _translation_norm(camera_t_lidar),
        "camera_body_translation_norm_m": _translation_norm(target_t_source),
        "camera_lidar_rotation_orthogonality_err": camera_rot_err,
        "camera_body_rotation_orthogonality_err": body_rot_err,
        "camera_lidar_rotation_det": camera_det,
        "camera_body_rotation_det": body_det,
    }

    if metrics["camera_lidar_translation_norm_m"] > 2.0:
        raise ValueError("camera-lidar translation looks implausibly large: %.3fm" % metrics["camera_lidar_translation_norm_m"])
    if metrics["camera_body_translation_norm_m"] > 2.0:
        raise ValueError("camera-body translation looks implausibly large: %.3fm" % metrics["camera_body_translation_norm_m"])
    return metrics


def main():
    parser = argparse.ArgumentParser(description="Validate Stage1 camera/MID360 calibration assets.")
    parser.add_argument("--intrinsics", required=True)
    parser.add_argument("--extrinsics", required=True)
    parser.add_argument("--camera-body", required=True, dest="camera_body")
    args = parser.parse_args()

    for path in (args.intrinsics, args.extrinsics, args.camera_body):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)

    metrics = validate_assets(args.intrinsics, args.extrinsics, args.camera_body)
    print("[PASS] calibration assets valid")
    for key in sorted(metrics.keys()):
        print("[%s] %s" % (key, metrics[key]))


if __name__ == "__main__":
    main()
