#!/usr/bin/env python3

import copy

import rospy
from nav_msgs.msg import Odometry


class OdomTopicRelay:
    def __init__(self):
        self._input_topic = rospy.get_param("~input_topic", "/laserMapping/odometry")
        self._output_topic = rospy.get_param("~output_topic", "/mavros/odometry/out")
        self._mirror_topic = rospy.get_param("~mirror_topic", "/follow/lio/odom")
        self._frame_id_override = rospy.get_param("~frame_id_override", "odom")
        self._child_frame_id_override = rospy.get_param("~child_frame_id_override", "base_link")
        self._restamp_now = bool(rospy.get_param("~restamp_now", False))

        self._output_publisher = rospy.Publisher(self._output_topic, Odometry, queue_size=20)
        self._mirror_publisher = None
        if self._mirror_topic:
            self._mirror_publisher = rospy.Publisher(self._mirror_topic, Odometry, queue_size=20)

        self._message_count = 0

        rospy.Subscriber(self._input_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.loginfo(
            "odom_topic_relay ready input=%s output=%s mirror=%s frame_override=%s child_override=%s restamp_now=%s",
            self._input_topic,
            self._output_topic,
            self._mirror_topic or "<disabled>",
            self._frame_id_override or "<preserve>",
            self._child_frame_id_override or "<preserve>",
            self._restamp_now,
        )

    def _odom_callback(self, message):
        if rospy.is_shutdown():
            return
        relayed = copy.deepcopy(message)
        if self._frame_id_override:
            relayed.header.frame_id = self._frame_id_override
        if self._child_frame_id_override:
            relayed.child_frame_id = self._child_frame_id_override
        if self._restamp_now:
            relayed.header.stamp = rospy.Time.now()

        try:
            self._output_publisher.publish(relayed)
            if self._mirror_publisher is not None:
                self._mirror_publisher.publish(relayed)
        except rospy.ROSException:
            return

        self._message_count += 1
        if self._message_count == 1:
            rospy.loginfo(
                "odom_topic_relay forwarded first odom message from %s to %s",
                self._input_topic,
                self._output_topic,
            )
        else:
            rospy.loginfo_throttle(
                5.0,
                "odom_topic_relay forwarding %d messages from %s",
                self._message_count,
                self._input_topic,
            )


def main():
    rospy.init_node("odom_topic_relay")
    OdomTopicRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
