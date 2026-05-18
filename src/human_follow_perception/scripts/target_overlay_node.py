#!/usr/bin/env python3
import copy

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from human_follow_msgs.msg import Target2D, TrackerStatus


class TargetOverlayNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/follow/camera/image_raw")
        self.detector_topic = rospy.get_param("~detector_topic", "/follow/detector/person_target")
        self.tracker_topic = rospy.get_param("~tracker_topic", "/follow/tracker/selected_target")
        self.tracker_status_topic = rospy.get_param("~tracker_status_topic", "/follow/tracker/status")
        self.output_topic = rospy.get_param("~output_topic", "/follow/debug/overlay_image")
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 10.0)

        self.last_image = None
        self.last_detector_target = None
        self.last_tracker_target = None
        self.last_tracker_status = None

        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        self.detector_sub = rospy.Subscriber(self.detector_topic, Target2D, self._detector_callback, queue_size=10)
        self.tracker_sub = rospy.Subscriber(self.tracker_topic, Target2D, self._tracker_callback, queue_size=10)
        self.tracker_status_sub = rospy.Subscriber(
            self.tracker_status_topic, TrackerStatus, self._tracker_status_callback, queue_size=10
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._publish)

        rospy.loginfo(
            "target_overlay_node ready: image=%s detector=%s tracker=%s tracker_status=%s output=%s",
            self.image_topic,
            self.detector_topic,
            self.tracker_topic,
            self.tracker_status_topic,
            self.output_topic,
        )

    def _image_callback(self, msg):
        self.last_image = msg

    def _detector_callback(self, msg):
        self.last_detector_target = copy.deepcopy(msg)

    def _tracker_callback(self, msg):
        self.last_tracker_target = copy.deepcopy(msg)

    def _tracker_status_callback(self, msg):
        self.last_tracker_status = copy.deepcopy(msg)

    def _decode_bgr(self, msg):
        if not msg or not msg.data:
            return None

        channels = 3
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

        expected = msg.height * msg.width * channels
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size < expected:
            return None
        return arr[:expected].reshape((msg.height, msg.width, channels)).copy()

    def _draw_target(self, frame, target, color, label):
        if frame is None or target is None or not target.valid:
            return

        h, w = frame.shape[:2]
        bw = int(round(target.width * w))
        bh = int(round(target.height * h))
        cx = int(round(target.cx * w))
        cy = int(round(target.cy * h))
        x1 = max(0, cx - bw // 2)
        y1 = max(0, cy - bh // 2)
        x2 = min(w - 1, x1 + bw)
        y2 = min(h - 1, y1 + bh)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        text = f"{label} id={target.track_id} conf={target.confidence:.2f} src={target.source}"
        cv2.putText(frame, text, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    def _draw_tracker_status(self, frame):
        if frame is None or self.last_tracker_status is None:
            return

        status = self.last_tracker_status
        lines = [
            "tracker state=%s age=%.2fs hold_left=%.2fs" % (
                status.tracking_state,
                status.age_since_valid_sec,
                status.remaining_hold_sec,
            ),
            "streak valid=%d invalid=%d reacq=%d timeout=%d" % (
                status.valid_input_streak,
                status.invalid_input_streak,
                status.reacquire_count,
                status.timeout_count,
            ),
            "detail=%s out=%s conf=%.2f" % (
                status.detail,
                status.output_source,
                status.output_confidence,
            ),
        ]

        y = 22
        for line in lines:
            text_size, _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (6, y - 14), (14 + text_size[0], y + 6), (0, 0, 0), -1)
            cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 255, 50), 1, cv2.LINE_AA)
            y += 20

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        frame = self._decode_bgr(self.last_image)
        if frame is None:
            return

        self._draw_target(frame, self.last_detector_target, (0, 255, 255), "det")
        self._draw_target(frame, self.last_tracker_target, (0, 255, 0), "trk")
        self._draw_tracker_status(frame)

        out = Image()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.last_image.header.frame_id or "camera"
        out.height = frame.shape[0]
        out.width = frame.shape[1]
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = frame.shape[1] * frame.shape[2]
        out.data = frame.tobytes()
        try:
            self.publisher.publish(out)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("target_overlay_node")
    TargetOverlayNode()
    rospy.spin()
