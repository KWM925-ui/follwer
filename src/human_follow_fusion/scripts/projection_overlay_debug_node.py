#!/usr/bin/env python3
import os
import sys

import cv2
import numpy as np
import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from projection_math import (  # noqa: E402
    load_camera_intrinsics_yaml,
    load_camera_lidar_extrinsics_yaml,
    project_lidar_points_to_image,
)


class ProjectionOverlayDebugNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/usb_cam/image_raw")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/laserMapping/cloud_registered_body")
        self.output_topic = rospy.get_param("~output_topic", "/follow/debug/projection_overlay_image")
        self.intrinsics_yaml = rospy.get_param("~intrinsics_yaml", "")
        self.extrinsics_yaml = rospy.get_param("~extrinsics_yaml", "")
        self.min_depth_m = float(rospy.get_param("~min_depth_m", 0.1))
        self.max_points_to_process = int(rospy.get_param("~max_points_to_process", 5000))
        self.point_radius_px = int(rospy.get_param("~point_radius_px", 2))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 5.0))
        self.snapshot_path = rospy.get_param("~snapshot_path", "")
        self.snapshot_written = False

        self.intrinsics = load_camera_intrinsics_yaml(self.intrinsics_yaml)
        self.extrinsics = load_camera_lidar_extrinsics_yaml(self.extrinsics_yaml)

        self.last_image = None
        self.last_pointcloud = None

        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self._pointcloud_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._publish)

        rospy.loginfo(
            "projection_overlay_debug ready: image=%s pointcloud=%s output=%s snapshot=%s",
            self.image_topic,
            self.pointcloud_topic,
            self.output_topic,
            self.snapshot_path or "<none>",
        )

    def _image_callback(self, msg):
        self.last_image = msg

    def _pointcloud_callback(self, msg):
        self.last_pointcloud = msg

    def _decode_bgr(self, msg):
        if not msg or not msg.data:
            return None
        if msg.encoding == "rgb8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if msg.encoding == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
        if msg.encoding == "mono8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        if msg.encoding == "rgba8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        if msg.encoding == "bgra8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        rospy.logwarn_throttle(5.0, "unsupported image encoding for projection overlay: %s", msg.encoding)
        return None

    def _pointcloud_xyz(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float64)
        if points.size == 0:
            return np.zeros((0, 3), dtype=np.float64)
        if points.ndim == 1:
            points = points.reshape((1, 3))
        if self.max_points_to_process > 0 and points.shape[0] > self.max_points_to_process:
            stride = max(1, int(np.ceil(float(points.shape[0]) / float(self.max_points_to_process))))
            points = points[::stride]
        return points

    def _depth_color(self, depth_m, min_depth, max_depth):
        alpha = 0.0 if max_depth <= min_depth else (float(depth_m) - min_depth) / (max_depth - min_depth)
        alpha = min(1.0, max(0.0, alpha))
        blue = int(round(255.0 * (1.0 - alpha)))
        red = int(round(255.0 * alpha))
        return (blue, 200, red)

    def _publish(self, _event):
        image_msg = self.last_image
        pointcloud_msg = self.last_pointcloud
        if image_msg is None or pointcloud_msg is None:
            return

        frame = self._decode_bgr(image_msg)
        if frame is None:
            return

        lidar_points = self._pointcloud_xyz(pointcloud_msg)
        if lidar_points.shape[0] == 0:
            return

        projection = project_lidar_points_to_image(
            lidar_points,
            self.intrinsics,
            self.extrinsics,
            min_depth_m=self.min_depth_m,
        )
        in_image_mask = projection["in_image_mask"]
        pixels = projection["pixels_uv"][in_image_mask]
        depths = projection["points_camera_xyz"][in_image_mask, 2]
        if pixels.shape[0] == 0:
            rospy.logwarn_throttle(2.0, "projection overlay found no in-image points")
            return

        min_depth = float(np.min(depths))
        max_depth = float(np.max(depths))
        for pixel, depth_m in zip(pixels, depths):
            u = int(round(pixel[0]))
            v = int(round(pixel[1]))
            if 0 <= u < frame.shape[1] and 0 <= v < frame.shape[0]:
                cv2.circle(frame, (u, v), self.point_radius_px, self._depth_color(depth_m, min_depth, max_depth), -1)

        cv2.putText(
            frame,
            "proj_pts=%d depth=[%.2f, %.2f]m" % (pixels.shape[0], min_depth, max_depth),
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

        if self.snapshot_path and not self.snapshot_written:
            os.makedirs(os.path.dirname(self.snapshot_path), exist_ok=True)
            cv2.imwrite(self.snapshot_path, frame)
            rospy.loginfo("projection overlay snapshot saved to %s", self.snapshot_path)
            self.snapshot_written = True

        out = Image()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = image_msg.header.frame_id or self.intrinsics.get("camera_frame", "camera")
        out.height = frame.shape[0]
        out.width = frame.shape[1]
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = frame.shape[1] * frame.shape[2]
        out.data = frame.tobytes()
        self.publisher.publish(out)


if __name__ == "__main__":
    rospy.init_node("projection_overlay_debug")
    ProjectionOverlayDebugNode()
    rospy.spin()
