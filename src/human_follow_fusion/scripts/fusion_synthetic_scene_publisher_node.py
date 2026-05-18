#!/usr/bin/env python3
import os
import sys

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from human_follow_msgs.msg import Target2D

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from projection_math import (  # noqa: E402
    load_camera_intrinsics_yaml,
    load_camera_lidar_extrinsics_yaml,
    project_lidar_points_to_image,
)


class FusionSyntheticScenePublisherNode:
    def __init__(self):
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/follow/lidar/points")
        self.target_topic = rospy.get_param("~target_topic", "/follow/tracker/selected_target")
        self.intrinsics_yaml = rospy.get_param("~intrinsics_yaml")
        self.extrinsics_yaml = rospy.get_param("~extrinsics_yaml")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 5.0))
        self.frame_id = rospy.get_param("~frame_id", "")

        self.intrinsics = load_camera_intrinsics_yaml(self.intrinsics_yaml)
        self.extrinsics = load_camera_lidar_extrinsics_yaml(self.extrinsics_yaml)
        self.lidar_frame = self.frame_id or self.extrinsics["lidar_frame"]

        self.pointcloud_pub = rospy.Publisher(self.pointcloud_topic, PointCloud2, queue_size=1)
        self.target_pub = rospy.Publisher(self.target_topic, Target2D, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        self.scene_points_camera = self._build_scene_camera_points()
        self.scene_points_lidar = self._camera_to_lidar(self.scene_points_camera)
        self.target_bbox = self._build_target_bbox()

        rospy.loginfo(
            "fusion_synthetic_scene_publisher ready: pointcloud=%s target=%s points=%d",
            self.pointcloud_topic,
            self.target_topic,
            self.scene_points_lidar.shape[0],
        )

    def _build_foreground_cluster(self):
        x_values = np.array([0.10, 0.40, 0.70], dtype=np.float64)
        y_values = np.array([-0.60, 0.00, 0.60], dtype=np.float64)
        z_value = 6.0
        cluster = []
        for y in y_values:
            for x in x_values:
                cluster.append([x, y, z_value])
        return np.array(cluster, dtype=np.float64)

    def _build_background_cluster(self):
        foreground = self._build_foreground_cluster()
        background = foreground.copy()
        scale = 10.0 / 6.0
        background[:, 0] *= scale
        background[:, 1] *= scale
        background[:, 2] = 10.0
        return background

    def _build_scene_camera_points(self):
        return np.vstack((self._build_foreground_cluster(), self._build_background_cluster()))

    def _camera_to_lidar(self, points_camera):
        lidar_T_camera = np.linalg.inv(self.extrinsics["camera_T_lidar"])
        return (lidar_T_camera[:3, :3] @ points_camera.T).T + lidar_T_camera[:3, 3]

    def _build_target_bbox(self):
        projection = project_lidar_points_to_image(
            self._camera_to_lidar(self._build_foreground_cluster()),
            self.intrinsics,
            self.extrinsics,
        )
        pixels = projection["pixels_uv"]
        x_min = float(np.min(pixels[:, 0]))
        x_max = float(np.max(pixels[:, 0]))
        y_min = float(np.min(pixels[:, 1]))
        y_max = float(np.max(pixels[:, 1]))
        return {
            "cx": ((x_min + x_max) * 0.5) / float(self.intrinsics["image_width"]),
            "cy": ((y_min + y_max) * 0.5) / float(self.intrinsics["image_height"]),
            "width": ((x_max - x_min) / float(self.intrinsics["image_width"])) * 1.15,
            "height": ((y_max - y_min) / float(self.intrinsics["image_height"])) * 1.15,
        }

    def _make_pointcloud_msg(self):
        points = self.scene_points_lidar.astype(np.float32)
        msg = PointCloud2()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=self.lidar_frame)
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.tobytes()
        return msg

    def _make_target_msg(self):
        msg = Target2D()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.intrinsics["camera_frame"]
        msg.track_id = 1
        msg.valid = True
        msg.confidence = 0.95
        msg.cx = float(self.target_bbox["cx"])
        msg.cy = float(self.target_bbox["cy"])
        msg.width = float(self.target_bbox["width"])
        msg.height = float(self.target_bbox["height"])
        msg.source = "fusion_synthetic_scene"
        return msg

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        try:
            self.pointcloud_pub.publish(self._make_pointcloud_msg())
            self.target_pub.publish(self._make_target_msg())
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("fusion_synthetic_scene_publisher")
    FusionSyntheticScenePublisherNode()
    rospy.spin()
