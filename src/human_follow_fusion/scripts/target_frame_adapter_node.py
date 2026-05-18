#!/usr/bin/env python3
import os
import sys

import numpy as np
import rospy

from human_follow_msgs.msg import Target3D

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from projection_math import load_rigid_transform_yaml, transform_points  # noqa: E402


class TargetFrameAdapterNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/fusion/target_camera")
        self.output_topic = rospy.get_param("~output_topic", "/follow/fusion/target_body")
        self.transform_yaml = rospy.get_param("~transform_yaml", "")

        if not self.transform_yaml:
            raise RuntimeError("target_frame_adapter requires ~transform_yaml")

        self.transform = load_rigid_transform_yaml(self.transform_yaml)
        self.publisher = rospy.Publisher(self.output_topic, Target3D, queue_size=10)
        self.subscriber = rospy.Subscriber(self.input_topic, Target3D, self._callback, queue_size=10)

        rospy.loginfo(
            "target_frame_adapter ready: input=%s output=%s %s->%s",
            self.input_topic,
            self.output_topic,
            self.transform["source_frame"],
            self.transform["target_frame"],
        )

    def _callback(self, msg):
        if rospy.is_shutdown():
            return
        out = Target3D()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.transform["target_frame"]
        out.track_id = msg.track_id
        out.valid = msg.valid
        out.confidence = msg.confidence
        out.frame_id = self.transform["target_frame"]
        out.support_points = msg.support_points

        if not msg.valid:
            out.position.x = 0.0
            out.position.y = 0.0
            out.position.z = 0.0
            try:
                self.publisher.publish(out)
            except rospy.ROSException:
                return
            return

        if msg.frame_id and msg.frame_id != self.transform["source_frame"]:
            rospy.logwarn_throttle(
                2.0,
                "target_frame_adapter input frame mismatch: got=%s expected=%s",
                msg.frame_id,
                self.transform["source_frame"],
            )

        point_in = np.array([[msg.position.x, msg.position.y, msg.position.z]], dtype=np.float64)
        point_out = transform_points(point_in, self.transform["target_T_source"])[0]
        out.position.x = float(point_out[0])
        out.position.y = float(point_out[1])
        out.position.z = float(point_out[2])
        try:
            self.publisher.publish(out)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("target_frame_adapter")
    TargetFrameAdapterNode()
    rospy.spin()
