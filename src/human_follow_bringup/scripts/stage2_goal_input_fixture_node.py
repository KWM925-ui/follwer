#!/usr/bin/env python3
import math
import struct
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from human_follow_msgs.msg import Target3D


def quaternion_from_yaw(yaw_rad):
    half = 0.5 * float(yaw_rad)
    return 0.0, 0.0, math.sin(half), math.cos(half)


DEFAULT_PHASES = [
    {
        "label": "target_ahead",
        "duration_sec": 1.8,
        "target_xyz": [6.0, 0.0, 1.7],
        "odom_xyz": [1.0, 0.0, 1.2],
        "odom_yaw_deg": 0.0,
    },
    {
        "label": "target_right_bias",
        "duration_sec": 1.8,
        "target_xyz": [6.2, -1.1, 1.7],
        "odom_xyz": [1.4, 0.3, 1.2],
        "odom_yaw_deg": 18.0,
    },
    {
        "label": "target_left_bias",
        "duration_sec": 1.8,
        "target_xyz": [5.8, 1.2, 1.7],
        "odom_xyz": [1.8, -0.2, 1.2],
        "odom_yaw_deg": -22.0,
    },
]


class Stage2GoalInputFixtureNode:
    def __init__(self):
        self.target_world_topic = rospy.get_param("~target_world_topic", "/follow/fusion/target_world")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/lio/odom")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/follow/lidar/points")
        self.phase_label_topic = rospy.get_param("~phase_label_topic", "/follow/test/phase_label")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 15.0))
        self.loop = bool(rospy.get_param("~loop", True))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.cloud_frame_id = rospy.get_param("~cloud_frame_id", "map")
        self.phases = rospy.get_param("~phases", DEFAULT_PHASES)

        if not self.phases:
            raise RuntimeError("stage2 goal input fixture phases cannot be empty")

        self.compiled_phases = []
        self.total_duration_sec = 0.0
        for phase in self.phases:
            compiled = dict(phase)
            compiled["duration_sec"] = max(0.0, float(compiled.get("duration_sec", 0.0)))
            if compiled["duration_sec"] <= 0.0:
                raise RuntimeError("fixture phase duration must be positive")
            self.total_duration_sec += compiled["duration_sec"]
            self.compiled_phases.append(compiled)

        self.start_time = time.monotonic()
        self.last_phase_label = None

        self.target_pub = rospy.Publisher(self.target_world_topic, Target3D, queue_size=10)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=10)
        self.phase_label_pub = rospy.Publisher(self.phase_label_topic, String, queue_size=10, latch=True)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "stage2_goal_input_fixture ready target=%s odom=%s cloud=%s phase=%s phases=%d loop=%s",
            self.target_world_topic,
            self.odom_topic,
            self.cloud_topic,
            self.phase_label_topic,
            len(self.compiled_phases),
            self.loop,
        )

    def _phase_for_elapsed(self, elapsed_sec):
        if self.loop:
            elapsed_sec = elapsed_sec % self.total_duration_sec
        elif elapsed_sec >= self.total_duration_sec:
            return self.compiled_phases[-1], len(self.compiled_phases) - 1

        cursor = 0.0
        for idx, phase in enumerate(self.compiled_phases):
            cursor += phase["duration_sec"]
            if elapsed_sec <= cursor:
                return phase, idx
        return self.compiled_phases[-1], len(self.compiled_phases) - 1

    def _target_msg(self, stamp, phase):
        msg = Target3D()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.track_id = int(phase.get("track_id", 1))
        msg.valid = bool(phase.get("target_valid", True))
        msg.confidence = 0.98 if msg.valid else 0.0
        msg.frame_id = self.frame_id
        msg.support_points = 64 if msg.valid else 0
        target_xyz = phase.get("target_xyz", [0.0, 0.0, 0.0])
        msg.position.x = float(target_xyz[0])
        msg.position.y = float(target_xyz[1])
        msg.position.z = float(target_xyz[2])
        return msg

    def _odom_msg(self, stamp, phase):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = "base_link"
        odom_xyz = phase.get("odom_xyz", [0.0, 0.0, 0.0])
        yaw_deg = float(phase.get("odom_yaw_deg", 0.0))
        qx, qy, qz, qw = quaternion_from_yaw(math.radians(yaw_deg))
        msg.pose.pose.position.x = float(odom_xyz[0])
        msg.pose.pose.position.y = float(odom_xyz[1])
        msg.pose.pose.position.z = float(odom_xyz[2])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        return msg

    def _cloud_msg(self, stamp, phase):
        target_xyz = phase.get("cloud_center_xyz", phase.get("target_xyz", phase.get("odom_xyz", [0.0, 0.0, 0.0])))
        points = []
        for dx in (-0.25, 0.0, 0.25):
            for dy in (-0.35, 0.0, 0.35):
                point = (
                    float(target_xyz[0] + dx),
                    float(target_xyz[1] + dy),
                    float(target_xyz[2]),
                )
                points.append(point)
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=self.cloud_frame_id)
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", point[0], point[1], point[2]) for point in points)
        return msg

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        elapsed_sec = max(0.0, time.monotonic() - self.start_time)
        phase, idx = self._phase_for_elapsed(elapsed_sec)
        label = phase.get("label", "phase_%d" % idx)
        if label != self.last_phase_label:
            rospy.loginfo("stage2_goal_input_fixture phase=%s duration=%.2fs", label, float(phase["duration_sec"]))
            self.last_phase_label = label
        stamp = rospy.Time.now()
        try:
            if bool(phase.get("publish_target", True)):
                self.target_pub.publish(self._target_msg(stamp, phase))
            self.odom_pub.publish(self._odom_msg(stamp, phase))
            self.cloud_pub.publish(self._cloud_msg(stamp, phase))
            self.phase_label_pub.publish(String(data=label))
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("stage2_goal_input_fixture")
    Stage2GoalInputFixtureNode()
    rospy.spin()
