#!/usr/bin/env python3
import os

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from human_follow_msgs.msg import Target2D


class SyntheticCameraCanvasNode:
    def __init__(self):
        self.output_topic = rospy.get_param("~output_topic", "/follow/camera/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/follow/camera/camera_info")
        self.phase_topic = rospy.get_param("~phase_topic", "/follow/sim/truth_phase")
        self.target_topic = rospy.get_param("~target_topic", "/follow/sim/truth_target_2d")
        self.intrinsics_yaml = rospy.get_param("~intrinsics_yaml", "")
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.image_width = int(rospy.get_param("~image_width", 1280))
        self.image_height = int(rospy.get_param("~image_height", 720))
        self.last_phase_text = "label=idle_boot"
        self.last_target = None
        self.person_fill_bgr = tuple(int(v) for v in rospy.get_param("~person_fill_bgr", [40, 220, 40]))
        self.person_border_bgr = tuple(int(v) for v in rospy.get_param("~person_border_bgr", [220, 255, 220]))
        self.person_head_bgr = tuple(int(v) for v in rospy.get_param("~person_head_bgr", [80, 245, 80]))

        if self.intrinsics_yaml:
            self._load_intrinsics()

        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=1)
        self.phase_sub = rospy.Subscriber(self.phase_topic, String, self._phase_callback, queue_size=10)
        self.target_sub = rospy.Subscriber(self.target_topic, Target2D, self._target_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "synthetic_camera_canvas ready: image=%s camera_info=%s size=%dx%d",
            self.output_topic,
            self.camera_info_topic,
            self.image_width,
            self.image_height,
        )

    def _load_intrinsics(self):
        if not os.path.isfile(self.intrinsics_yaml):
            raise RuntimeError("intrinsics yaml not found: %s" % self.intrinsics_yaml)
        with open(self.intrinsics_yaml, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
        self.frame_id = data.get("camera_frame", self.frame_id)
        self.image_width = int(data.get("image_width", self.image_width))
        self.image_height = int(data.get("image_height", self.image_height))
        self.camera_info_template = data

    def _phase_callback(self, msg):
        self.last_phase_text = str(msg.data or "label=phase")

    def _target_callback(self, msg):
        self.last_target = msg

    def _draw_target(self, frame):
        if self.last_target is None or not self.last_target.valid:
            return

        cx_px = int(round(float(self.last_target.cx) * self.image_width))
        cy_px = int(round(float(self.last_target.cy) * self.image_height))
        width_px = int(round(float(self.last_target.width) * self.image_width))
        height_px = int(round(float(self.last_target.height) * self.image_height))

        if width_px <= 0 or height_px <= 0:
            return

        x_min = max(0, cx_px - width_px // 2)
        y_min = max(0, cy_px - height_px // 2)
        x_max = min(self.image_width - 1, x_min + width_px)
        y_max = min(self.image_height - 1, y_min + height_px)
        if x_max <= x_min or y_max <= y_min:
            return

        torso_top = int(round(y_min + 0.18 * (y_max - y_min)))
        cv2.rectangle(frame, (x_min, torso_top), (x_max, y_max), self.person_fill_bgr, thickness=-1)
        head_radius = max(6, int(round(min(width_px, height_px) * 0.14)))
        head_center = (cx_px, max(y_min + head_radius, torso_top - head_radius))
        cv2.circle(frame, head_center, head_radius, self.person_head_bgr, thickness=-1)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), self.person_border_bgr, thickness=2)

    def _build_canvas(self):
        frame = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
        frame[:, :, 0] = 22
        frame[:, :, 1] = 28
        frame[:, :, 2] = 36

        for x_idx in range(0, self.image_width, 80):
            cv2.line(frame, (x_idx, 0), (x_idx, self.image_height - 1), (38, 44, 56), 1)
        for y_idx in range(0, self.image_height, 80):
            cv2.line(frame, (0, y_idx), (self.image_width - 1, y_idx), (38, 44, 56), 1)

        cx = self.image_width // 2
        cy = self.image_height // 2
        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (160, 160, 190), 1)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (160, 160, 190), 1)

        title = "Stage1 Synthetic Camera Canvas"
        info = "phase: %s" % self.last_phase_text
        cv2.putText(frame, title, (24, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (240, 240, 240), 2, cv2.LINE_AA)
        cv2.putText(frame, info, (24, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (120, 255, 120), 1, cv2.LINE_AA)
        cv2.putText(
            frame,
            "overlay node will render detector/tracker boxes here",
            (24, self.image_height - 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (200, 210, 220),
            1,
            cv2.LINE_AA,
        )
        self._draw_target(frame)
        return frame

    def _build_camera_info(self, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = self.image_width
        msg.height = self.image_height

        template = getattr(self, "camera_info_template", None)
        if template is None:
            msg.distortion_model = "plumb_bob"
            msg.K = [800.0, 0.0, self.image_width / 2.0, 0.0, 800.0, self.image_height / 2.0, 0.0, 0.0, 1.0]
            msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.P = [800.0, 0.0, self.image_width / 2.0, 0.0, 0.0, 800.0, self.image_height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            return msg

        msg.distortion_model = template.get("distortion_model", "plumb_bob")
        msg.K = [float(v) for v in template["camera_matrix"]["data"]]
        msg.R = [float(v) for v in template["rectification_matrix"]["data"]]
        msg.P = [float(v) for v in template["projection_matrix"]["data"]]
        msg.D = [float(v) for v in template["distortion_coefficients"]["data"]]
        return msg

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        stamp = rospy.Time.now()
        frame = self._build_canvas()

        image_msg = Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id
        image_msg.height = frame.shape[0]
        image_msg.width = frame.shape[1]
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = 0
        image_msg.step = frame.shape[1] * frame.shape[2]
        image_msg.data = frame.tobytes()
        try:
            self.image_pub.publish(image_msg)
            self.camera_info_pub.publish(self._build_camera_info(stamp))
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("synthetic_camera_canvas")
    SyntheticCameraCanvasNode()
    rospy.spin()
