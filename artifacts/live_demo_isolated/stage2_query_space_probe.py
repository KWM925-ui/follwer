#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FUSION_SCRIPT_DIR = os.path.abspath(
    os.path.join(SCRIPT_DIR, "..", "..", "src", "human_follow_fusion", "scripts")
)
if FUSION_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, FUSION_SCRIPT_DIR)

from projection_math import load_camera_lidar_extrinsics_yaml, load_rigid_transform_yaml, transform_points  # noqa: E402


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def points_from_cloud(msg):
    data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not data:
        return np.zeros((0, 3), dtype=np.float64)
    arr = np.asarray(data, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape((1, 3))
    return arr


def nearest_summary(points_xyz, query_xyz):
    if points_xyz is None or len(points_xyz) == 0:
        return {
            "count": 0,
            "nearest_distance_m": None,
            "nearest_point": None,
            "within_0_15m": 0,
            "within_0_30m": 0,
            "within_0_50m": 0,
            "within_1_00m": 0,
        }
    delta = points_xyz - query_xyz.reshape((1, 3))
    dists = np.linalg.norm(delta, axis=1)
    idx = int(np.argmin(dists))
    nearest = points_xyz[idx]
    return {
        "count": int(points_xyz.shape[0]),
        "nearest_distance_m": round(float(dists[idx]), 3),
        "nearest_point": {
            "x": round(float(nearest[0]), 3),
            "y": round(float(nearest[1]), 3),
            "z": round(float(nearest[2]), 3),
        },
        "within_0_15m": int(np.count_nonzero(dists <= 0.15)),
        "within_0_30m": int(np.count_nonzero(dists <= 0.30)),
        "within_0_50m": int(np.count_nonzero(dists <= 0.50)),
        "within_1_00m": int(np.count_nonzero(dists <= 1.00)),
    }


class Probe:
    def __init__(self, extrinsics_yaml, body_yaml):
        self.camera_lidar = load_camera_lidar_extrinsics_yaml(extrinsics_yaml)
        self.body_from_camera = load_rigid_transform_yaml(body_yaml)["target_T_source"]

        self.odom = None
        self.grid_cloud = None
        self.grid_occ = None
        self.grid_occ_inflate = None
        self.obstacle_cloud = None
        self.foreground_cloud = None

        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_cb, queue_size=20)
        rospy.Subscriber("/grid_map/cloud", PointCloud2, self._grid_cloud_cb, queue_size=5)
        rospy.Subscriber("/grid_map/occupancy", PointCloud2, self._grid_occ_cb, queue_size=5)
        rospy.Subscriber("/grid_map/occupancy_inflate", PointCloud2, self._grid_occ_inflate_cb, queue_size=5)
        rospy.Subscriber("/follow/lidar/obstacle_points", PointCloud2, self._obstacle_cb, queue_size=5)
        rospy.Subscriber("/follow/lidar/foreground_points", PointCloud2, self._foreground_cb, queue_size=5)

    def _odom_cb(self, msg):
        self.odom = msg

    def _grid_cloud_cb(self, msg):
        self.grid_cloud = msg

    def _grid_occ_cb(self, msg):
        self.grid_occ = msg

    def _grid_occ_inflate_cb(self, msg):
        self.grid_occ_inflate = msg

    def _obstacle_cb(self, msg):
        self.obstacle_cloud = msg

    def _foreground_cb(self, msg):
        self.foreground_cloud = msg

    def wait_ready(self, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if (
                self.odom is not None
                and self.grid_cloud is not None
                and self.grid_occ_inflate is not None
                and self.obstacle_cloud is not None
                and self.foreground_cloud is not None
            ):
                return True
            time.sleep(0.05)
        return False

    def lidar_to_world(self, msg):
        if self.odom is None:
            return np.zeros((0, 3), dtype=np.float64)
        lidar_points = points_from_cloud(msg)
        if lidar_points.shape[0] == 0:
            return lidar_points
        camera_points = transform_points(lidar_points, self.camera_lidar["camera_T_lidar"])
        body_points = transform_points(camera_points, self.body_from_camera)

        pose = self.odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_points = np.zeros_like(body_points)
        world_points[:, 0] = float(pose.position.x) + cos_yaw * body_points[:, 0] - sin_yaw * body_points[:, 1]
        world_points[:, 1] = float(pose.position.y) + sin_yaw * body_points[:, 0] + cos_yaw * body_points[:, 1]
        world_points[:, 2] = float(pose.position.z) + body_points[:, 2]
        return world_points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--query-x", type=float, required=True)
    parser.add_argument("--query-y", type=float, required=True)
    parser.add_argument("--query-z", type=float, required=True)
    parser.add_argument(
        "--extrinsics-yaml",
        default="/home/coco/follwer_ws/src/human_follow_fusion/config/camera_mid360_extrinsics.example.yaml",
    )
    parser.add_argument(
        "--body-yaml",
        default="/home/coco/follwer_ws/src/human_follow_fusion/config/camera_to_body_stage1_sim.yaml",
    )
    args = parser.parse_args()

    rospy.init_node("stage2_query_space_probe", anonymous=True)
    probe = Probe(args.extrinsics_yaml, args.body_yaml)
    if not probe.wait_ready():
        raise SystemExit("probe_timeout")

    query = np.asarray([args.query_x, args.query_y, args.query_z], dtype=np.float64)
    grid_cloud = points_from_cloud(probe.grid_cloud)
    grid_occ = points_from_cloud(probe.grid_occ) if probe.grid_occ is not None else np.zeros((0, 3), dtype=np.float64)
    grid_occ_inflate = points_from_cloud(probe.grid_occ_inflate)
    obstacle_world = probe.lidar_to_world(probe.obstacle_cloud)
    foreground_world = probe.lidar_to_world(probe.foreground_cloud)

    result = {
        "query": {
            "x": round(float(query[0]), 3),
            "y": round(float(query[1]), 3),
            "z": round(float(query[2]), 3),
        },
        "odom": {
            "x": round(float(probe.odom.pose.pose.position.x), 3),
            "y": round(float(probe.odom.pose.pose.position.y), 3),
            "z": round(float(probe.odom.pose.pose.position.z), 3),
        },
        "grid_map_cloud": nearest_summary(grid_cloud, query),
        "grid_map_occupancy": nearest_summary(grid_occ, query),
        "grid_map_occupancy_inflate": nearest_summary(grid_occ_inflate, query),
        "obstacle_points_world": nearest_summary(obstacle_world, query),
        "foreground_points_world": nearest_summary(foreground_world, query),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
