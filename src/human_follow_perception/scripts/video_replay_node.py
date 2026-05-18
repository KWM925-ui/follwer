#!/usr/bin/env python3
import os

import cv2
import rospy

from sensor_msgs.msg import Image


class VideoReplayNode:
    def __init__(self):
        self.video_path = rospy.get_param("~video_path", "")
        self.output_topic = rospy.get_param("~output_topic", "/follow/camera/image_raw")
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.loop = bool(rospy.get_param("~loop", True))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 15.0))
        self.resize_width_px = int(rospy.get_param("~resize_width_px", 0))

        if not self.video_path:
            raise RuntimeError("~video_path is required for video_replay_node")
        if not os.path.exists(self.video_path):
            raise RuntimeError("video path does not exist: %s" % self.video_path)

        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.capture = cv2.VideoCapture(self.video_path)
        if not self.capture.isOpened():
            raise RuntimeError("failed to open video: %s" % self.video_path)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)
        rospy.loginfo("video_replay_node ready: video=%s output=%s", self.video_path, self.output_topic)

    def _reopen_or_stop(self):
        self.capture.release()
        if self.loop:
            self.capture = cv2.VideoCapture(self.video_path)
            return self.capture.isOpened()
        rospy.signal_shutdown("video replay finished")
        return False

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        ok, frame = self.capture.read()
        if not ok:
            if not self._reopen_or_stop():
                return
            ok, frame = self.capture.read()
            if not ok:
                return

        if self.resize_width_px > 0 and frame.shape[1] > self.resize_width_px:
            resized_h = max(1, int(round(frame.shape[0] * float(self.resize_width_px) / float(frame.shape[1]))))
            frame = cv2.resize(frame, (self.resize_width_px, resized_h))

        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        try:
            self.publisher.publish(msg)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("video_replay_node")
    VideoReplayNode()
    rospy.spin()
