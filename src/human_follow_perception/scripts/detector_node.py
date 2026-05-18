#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from human_follow_msgs.msg import Target2D


def parse_phase_label(text):
    raw = str(text or "").strip()
    if not raw:
        return ""
    for item in raw.split(";"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        if key.strip() == "label":
            return value.strip()
    return raw


class DetectorNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/follow/camera/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/follow/detector/person_target")
        self.phase_topic = rospy.get_param("~phase_topic", "/follow/sim/truth_phase")
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 10.0)
        self.require_image_heartbeat = rospy.get_param("~require_image_heartbeat", False)
        self.image_timeout_sec = rospy.get_param("~image_timeout_sec", 1.0)
        self.default_frame_id = rospy.get_param("~default_frame_id", "camera")
        self.backend = rospy.get_param("~backend", "hog").strip().lower()
        self.debug_log_every_n_frames = int(rospy.get_param("~debug_log_every_n_frames", 30))
        self.image_rotation_deg = int(rospy.get_param("~image_rotation_deg", 0))
        self.flip_horizontal = bool(rospy.get_param("~flip_horizontal", False))
        self.flip_vertical = bool(rospy.get_param("~flip_vertical", False))
        self.resize_width_px = int(rospy.get_param("~resize_width_px", 640))
        self.hog_stride_px = int(rospy.get_param("~hog_stride_px", 8))
        self.hog_padding_px = int(rospy.get_param("~hog_padding_px", 8))
        self.hog_scale = float(rospy.get_param("~hog_scale", 1.05))
        self.hog_hit_threshold = float(rospy.get_param("~hog_hit_threshold", 0.0))
        self.min_bbox_area_px = int(rospy.get_param("~min_bbox_area_px", 4096))
        self.min_bbox_confidence = float(rospy.get_param("~min_bbox_confidence", 0.0))
        self.target_selection = rospy.get_param("~target_selection", "largest_area")
        self.mock_valid_detection = rospy.get_param("~mock_valid_detection", False)
        self.synthetic_hsv_lower = tuple(int(v) for v in rospy.get_param("~synthetic_hsv_lower", [45, 80, 80]))
        self.synthetic_hsv_upper = tuple(int(v) for v in rospy.get_param("~synthetic_hsv_upper", [85, 255, 255]))

        self.last_image_stamp = None
        self.last_frame_id = self.default_frame_id
        self.last_image_msg = None
        self.last_phase_label = ""
        self.frame_counter = 0

        self.hog = None
        if self.backend == "hog":
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.publisher = rospy.Publisher(self.output_topic, Target2D, queue_size=10)
        self.subscriber = rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        self.phase_subscriber = rospy.Subscriber(self.phase_topic, String, self._phase_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._publish)

        rospy.loginfo(
            "human_follow_detector ready: backend=%s image=%s output=%s",
            self.backend,
            self.image_topic,
            self.output_topic,
        )

    def _image_callback(self, msg):
        self.last_image_stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        self.last_frame_id = msg.header.frame_id or self.default_frame_id
        self.last_image_msg = msg

    def _phase_callback(self, msg):
        self.last_phase_label = parse_phase_label(msg.data)

    def _image_is_fresh(self, now):
        if self.last_image_stamp is None:
            return False
        return (now - self.last_image_stamp).to_sec() <= self.image_timeout_sec

    def _decode_image(self, msg):
        if not msg.data:
            return None

        channels = 3
        if "mono8" in msg.encoding:
            channels = 1
        elif "rgba8" in msg.encoding or "bgra8" in msg.encoding:
            channels = 4

        dtype = np.uint8
        if msg.encoding in ("16UC1", "mono16"):
            dtype = np.uint16

        arr = np.frombuffer(msg.data, dtype=dtype)
        expected = msg.height * msg.width * channels
        if arr.size < expected:
            rospy.logwarn_throttle(5.0, "detector received truncated image buffer")
            return None

        if channels == 1:
            frame = arr[: msg.height * msg.width].reshape((msg.height, msg.width))
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            frame = arr[:expected].reshape((msg.height, msg.width, channels))
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "rgba8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif msg.encoding == "bgra8":
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            elif msg.encoding != "bgr8":
                frame = frame[:, :, :3]

        if self.image_rotation_deg == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.image_rotation_deg == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.image_rotation_deg == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)
        if self.flip_vertical:
            frame = cv2.flip(frame, 0)

        return frame

    def _detect_hog(self, frame):
        if frame is None or self.hog is None:
            return None

        original_h, original_w = frame.shape[:2]
        work = frame
        scale_ratio = 1.0
        if self.resize_width_px > 0 and original_w > self.resize_width_px:
            scale_ratio = float(self.resize_width_px) / float(original_w)
            resized_h = max(1, int(round(original_h * scale_ratio)))
            work = cv2.resize(frame, (self.resize_width_px, resized_h))

        boxes, weights = self.hog.detectMultiScale(
            work,
            winStride=(self.hog_stride_px, self.hog_stride_px),
            padding=(self.hog_padding_px, self.hog_padding_px),
            scale=self.hog_scale,
            hitThreshold=self.hog_hit_threshold,
        )

        if len(boxes) == 0:
            return None

        candidates = []
        inv_scale = 1.0 / scale_ratio
        for idx, (x, y, w, h) in enumerate(boxes):
            confidence = float(weights[idx]) if len(weights) > idx else 1.0
            area = int(w * h)
            if area < self.min_bbox_area_px:
                continue
            if confidence < self.min_bbox_confidence:
                continue

            x = int(round(x * inv_scale))
            y = int(round(y * inv_scale))
            w = int(round(w * inv_scale))
            h = int(round(h * inv_scale))

            candidates.append(
                {
                    "confidence": confidence,
                    "area": w * h,
                    "cx": (x + 0.5 * w) / float(original_w),
                    "cy": (y + 0.5 * h) / float(original_h),
                    "width": w / float(original_w),
                    "height": h / float(original_h),
                }
            )

        if not candidates:
            return None

        if self.target_selection == "largest_area":
            best = max(candidates, key=lambda item: (item["area"], item["confidence"]))
        else:
            best = max(candidates, key=lambda item: (item["confidence"], item["area"]))
        return best

    def _detect_synthetic_box(self, frame):
        if frame is None:
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.synthetic_hsv_lower, dtype=np.uint8), np.array(self.synthetic_hsv_upper, dtype=np.uint8))
        mask = cv2.medianBlur(mask, 5)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        image_h, image_w = frame.shape[:2]
        candidates = []
        for contour in contours:
            x_value, y_value, width_px, height_px = cv2.boundingRect(contour)
            area_px = int(width_px * height_px)
            if area_px < self.min_bbox_area_px:
                continue
            candidates.append(
                {
                    "confidence": min(0.999, max(0.50, float(area_px) / float(max(image_w * image_h, 1)) * 8.0)),
                    "area": area_px,
                    "cx": (x_value + 0.5 * width_px) / float(image_w),
                    "cy": (y_value + 0.5 * height_px) / float(image_h),
                    "width": width_px / float(image_w),
                    "height": height_px / float(image_h),
                }
            )

        if not candidates:
            return None

        if self.target_selection == "largest_area":
            return max(candidates, key=lambda item: (item["area"], item["confidence"]))
        return max(candidates, key=lambda item: (item["confidence"], item["area"]))

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        target = Target2D()
        target.header.stamp = now
        target.header.frame_id = self.last_frame_id or self.default_frame_id
        target.source = self.backend

        allow_mock = self.mock_valid_detection and (
            not self.require_image_heartbeat or self._image_is_fresh(now)
        )

        if allow_mock:
            target.track_id = rospy.get_param("~mock_track_id", 1)
            target.valid = True
            target.confidence = rospy.get_param("~mock_confidence", 0.9)
            target.cx = rospy.get_param("~mock_cx", 0.5)
            target.cy = rospy.get_param("~mock_cy", 0.5)
            target.width = rospy.get_param("~mock_width", 0.25)
            target.height = rospy.get_param("~mock_height", 0.5)
        else:
            detection = None
            if self.last_image_msg and (not self.require_image_heartbeat or self._image_is_fresh(now)):
                frame = self._decode_image(self.last_image_msg)
                if self.backend == "hog":
                    detection = self._detect_hog(frame)
                elif self.backend == "synthetic_box":
                    detection = self._detect_synthetic_box(frame)

                self.frame_counter += 1
                if self.debug_log_every_n_frames > 0 and self.frame_counter % self.debug_log_every_n_frames == 0:
                    rospy.loginfo(
                        "detector frame=%d detection=%s",
                        self.frame_counter,
                        "valid" if detection else "none",
                    )

            if detection:
                target.track_id = 1
                target.valid = True
                target.confidence = detection["confidence"]
                target.cx = detection["cx"]
                target.cy = detection["cy"]
                target.width = detection["width"]
                target.height = detection["height"]
                if self.backend == "synthetic_box":
                    suffix = self.last_phase_label or "phase"
                    target.source = "synthetic_box:%s" % suffix
                else:
                    target.source = "hog_person_detector"
            else:
                target.track_id = -1
                target.valid = False
                target.confidence = 0.0
                target.cx = 0.0
                target.cy = 0.0
                target.width = 0.0
                target.height = 0.0

        try:
            self.publisher.publish(target)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("human_follow_detector")
    DetectorNode()
    rospy.spin()
