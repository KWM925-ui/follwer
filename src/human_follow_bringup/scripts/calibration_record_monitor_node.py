#!/usr/bin/env python3
import os

import rospy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage


class CalibrationRecordMonitorNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/usb_cam/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/usb_cam/camera_info")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/laserMapping/cloud_registered_body")
        self.raw_lidar_topic = rospy.get_param("~raw_lidar_topic", "/livox/lidar")
        self.imu_topic = rospy.get_param("~imu_topic", "/livox/imu")
        self.tf_static_topic = rospy.get_param("~tf_static_topic", "/tf_static")
        self.output_manifest = rospy.get_param("~output_manifest", "")
        self.min_image = int(rospy.get_param("~min_image", 3))
        self.min_camera_info = int(rospy.get_param("~min_camera_info", 1))
        self.min_pointcloud = int(rospy.get_param("~min_pointcloud", 3))
        self.min_raw_lidar = int(rospy.get_param("~min_raw_lidar", 1))
        self.min_imu = int(rospy.get_param("~min_imu", 1))
        self.min_tf_static = int(rospy.get_param("~min_tf_static", 0))
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 15.0))

        self.image_count = 0
        self.camera_info_count = 0
        self.pointcloud_count = 0
        self.raw_lidar_count = 0
        self.imu_count = 0
        self.tf_static_count = 0
        self.done = False
        self.result = "running"

        rospy.Subscriber(self.image_topic, Image, self._image_cb, queue_size=10)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=10)
        rospy.Subscriber(self.pointcloud_topic, PointCloud2, self._pointcloud_cb, queue_size=10)
        rospy.Subscriber(self.raw_lidar_topic, rospy.AnyMsg, self._raw_lidar_cb, queue_size=10)
        rospy.Subscriber(self.imu_topic, Imu, self._imu_cb, queue_size=10)
        rospy.Subscriber(self.tf_static_topic, TFMessage, self._tf_static_cb, queue_size=10)
        rospy.Timer(rospy.Duration(0.5), self._check)
        rospy.Timer(rospy.Duration(self.timeout_sec), self._timeout, oneshot=True)

        rospy.loginfo(
            "calibration_record_monitor ready: image=%s camera_info=%s pointcloud=%s raw_lidar=%s imu=%s tf_static=%s",
            self.image_topic,
            self.camera_info_topic,
            self.pointcloud_topic,
            self.raw_lidar_topic,
            self.imu_topic,
            self.tf_static_topic,
        )

    def _image_cb(self, _msg):
        self.image_count += 1

    def _camera_info_cb(self, _msg):
        self.camera_info_count += 1

    def _pointcloud_cb(self, _msg):
        self.pointcloud_count += 1

    def _raw_lidar_cb(self, _msg):
        self.raw_lidar_count += 1

    def _imu_cb(self, _msg):
        self.imu_count += 1

    def _tf_static_cb(self, _msg):
        self.tf_static_count += 1

    def _manifest_lines(self):
        return [
            "result=%s" % self.result,
            "image_count=%d" % self.image_count,
            "camera_info_count=%d" % self.camera_info_count,
            "pointcloud_count=%d" % self.pointcloud_count,
            "raw_lidar_count=%d" % self.raw_lidar_count,
            "imu_count=%d" % self.imu_count,
            "tf_static_count=%d" % self.tf_static_count,
        ]

    def _write_manifest(self):
        if not self.output_manifest:
            return
        os.makedirs(os.path.dirname(self.output_manifest), exist_ok=True)
        with open(self.output_manifest, "w", encoding="utf-8") as handle:
            handle.write("\n".join(self._manifest_lines()) + "\n")

    def _success(self):
        return (
            self.image_count >= self.min_image
            and self.camera_info_count >= self.min_camera_info
            and self.pointcloud_count >= self.min_pointcloud
            and self.raw_lidar_count >= self.min_raw_lidar
            and self.imu_count >= self.min_imu
            and self.tf_static_count >= self.min_tf_static
        )

    def _check(self, _event):
        if self.done:
            return
        if self._success():
            self.done = True
            self.result = "pass"
            self._write_manifest()
            rospy.loginfo("calibration record monitor PASS: %s", " ".join(self._manifest_lines()))
            rospy.signal_shutdown("calibration record monitor pass")

    def _timeout(self, _event):
        if self.done:
            return
        self.done = True
        self.result = "fail"
        self._write_manifest()
        rospy.logerr("calibration record monitor FAIL: %s", " ".join(self._manifest_lines()))
        rospy.signal_shutdown("calibration record monitor timeout")


if __name__ == "__main__":
    rospy.init_node("calibration_record_monitor")
    CalibrationRecordMonitorNode()
    rospy.spin()
