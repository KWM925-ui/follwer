#!/usr/bin/env python3
import copy
from collections import deque
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FUSION_SCRIPT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "human_follow_fusion", "scripts"))
if FUSION_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, FUSION_SCRIPT_DIR)

from projection_math import (  # noqa: E402
    load_camera_lidar_extrinsics_yaml,
    load_rigid_transform_yaml,
    transform_points,
)


class Stage2EgoTopicAdapterNode:
    def __init__(self):
        self.goal_input_topic = rospy.get_param("~goal_input_topic", "/follow/stage2/goal")
        self.goal_output_topic = rospy.get_param("~goal_output_topic", "/move_base_simple/goal")
        self.odom_input_topic = rospy.get_param("~odom_input_topic", "/follow/lio/odom")
        self.odom_output_topic = rospy.get_param("~odom_output_topic", "/odom_world")
        self.grid_odom_output_topic = rospy.get_param("~grid_odom_output_topic", "/grid_map/odom")
        self.cloud_input_topic = rospy.get_param("~cloud_input_topic", "/follow/lidar/points")
        self.cloud_output_topic = rospy.get_param("~cloud_output_topic", "/grid_map/cloud")
        self.frame_id_override = rospy.get_param("~frame_id_override", "")
        self.restamp_now = bool(rospy.get_param("~restamp_now", False))
        self.cloud_to_world_enabled = bool(rospy.get_param("~cloud_to_world_enabled", False))
        self.cloud_extrinsics_yaml = rospy.get_param("~cloud_extrinsics_yaml", "")
        self.cloud_body_yaml = rospy.get_param("~cloud_body_yaml", "")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.odom_history_sec = max(float(rospy.get_param("~odom_history_sec", 1.0)), 0.1)
        self.max_cloud_odom_dt_sec = max(float(rospy.get_param("~max_cloud_odom_dt_sec", 0.12)), 0.0)

        self._camera_lidar_extrinsics = None
        self._body_from_camera = None
        if self.cloud_to_world_enabled:
            if not self.cloud_extrinsics_yaml or not self.cloud_body_yaml:
                raise RuntimeError("cloud_to_world_enabled requires cloud_extrinsics_yaml and cloud_body_yaml")
            self._camera_lidar_extrinsics = load_camera_lidar_extrinsics_yaml(self.cloud_extrinsics_yaml)
            self._body_from_camera = load_rigid_transform_yaml(self.cloud_body_yaml)["target_T_source"]

        self.goal_pub = rospy.Publisher(self.goal_output_topic, PoseStamped, queue_size=10, latch=True)
        self.odom_pub = rospy.Publisher(self.odom_output_topic, Odometry, queue_size=20)
        self.grid_odom_pub = rospy.Publisher(self.grid_odom_output_topic, Odometry, queue_size=20)
        self.cloud_pub = rospy.Publisher(self.cloud_output_topic, PointCloud2, queue_size=10)

        self.last_odom = None
        self.odom_history = deque(maxlen=300)

        rospy.Subscriber(self.goal_input_topic, PoseStamped, self._goal_callback, queue_size=10)
        rospy.Subscriber(self.odom_input_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(self.cloud_input_topic, PointCloud2, self._cloud_callback, queue_size=10)

        rospy.loginfo(
            "stage2_ego_topic_adapter ready: goal %s->%s odom %s->%s grid_odom=%s cloud %s->%s",
            self.goal_input_topic,
            self.goal_output_topic,
            self.odom_input_topic,
            self.odom_output_topic,
            self.grid_odom_output_topic,
            self.cloud_input_topic,
            self.cloud_output_topic,
        )

    def _maybe_override_header(self, msg):
        if self.restamp_now:
            msg.header.stamp = rospy.Time.now()
        if self.frame_id_override:
            msg.header.frame_id = self.frame_id_override
        return msg

    def _goal_callback(self, msg):
        out = copy.deepcopy(msg)
        out = self._maybe_override_header(out)
        try:
            self.goal_pub.publish(out)
        except rospy.ROSException:
            return

    def _odom_callback(self, msg):
        self.last_odom = msg
        now_sec = rospy.Time.now().to_sec()
        stamp_sec = msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time(0) else now_sec
        self.odom_history.append((stamp_sec, copy.deepcopy(msg)))
        while self.odom_history and now_sec - self.odom_history[0][0] > self.odom_history_sec:
            self.odom_history.popleft()
        out = copy.deepcopy(msg)
        out = self._maybe_override_header(out)
        try:
            self.odom_pub.publish(out)
            self.grid_odom_pub.publish(out)
        except rospy.ROSException:
            return

    def _odom_for_cloud(self, msg):
        if self.last_odom is None:
            return None
        if msg.header.stamp == rospy.Time(0) or not self.odom_history:
            return self.last_odom

        cloud_stamp_sec = msg.header.stamp.to_sec()
        best_dt = None
        best_odom = None
        for stamp_sec, odom in self.odom_history:
            dt_sec = abs(stamp_sec - cloud_stamp_sec)
            if best_dt is None or dt_sec < best_dt:
                best_dt = dt_sec
                best_odom = odom
        if best_odom is None:
            return self.last_odom
        if best_dt is not None and best_dt > self.max_cloud_odom_dt_sec:
            rospy.logwarn_throttle(
                1.0,
                "stage2_ego_topic_adapter cloud/odom stamp gap %.3fs exceeds %.3fs; using nearest odom",
                best_dt,
                self.max_cloud_odom_dt_sec,
            )
        return best_odom

    def _cloud_to_world(self, msg):
        if self._camera_lidar_extrinsics is None:
            return None
        odom_msg = self._odom_for_cloud(msg)
        if odom_msg is None:
            return None

        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points:
            out = PointCloud2()
            out.header = Header(stamp=rospy.Time.now(), frame_id=self.world_frame_id)
            out.height = 1
            out.width = 0
            out.fields = copy.deepcopy(msg.fields)
            out.is_bigendian = msg.is_bigendian
            out.point_step = msg.point_step
            out.row_step = 0
            out.is_dense = True
            out.data = b""
            return out

        import math
        import numpy as np

        lidar_points = np.asarray(points, dtype=np.float64)
        camera_points = transform_points(lidar_points, self._camera_lidar_extrinsics["camera_T_lidar"])
        body_points = transform_points(camera_points, self._body_from_camera)

        odom = odom_msg.pose.pose
        vehicle_x = float(odom.position.x)
        vehicle_y = float(odom.position.y)
        vehicle_z = float(odom.position.z)
        q = odom.orientation
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)

        world_points = np.zeros_like(body_points)
        world_points[:, 0] = vehicle_x + cos_yaw * body_points[:, 0] - sin_yaw * body_points[:, 1]
        world_points[:, 1] = vehicle_y + sin_yaw * body_points[:, 0] + cos_yaw * body_points[:, 1]
        world_points[:, 2] = vehicle_z + body_points[:, 2]

        header = Header(stamp=msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now(), frame_id=self.world_frame_id)
        return pc2.create_cloud_xyz32(header, world_points.astype("float32"))

    def _cloud_callback(self, msg):
        out = copy.deepcopy(msg)
        if self.cloud_to_world_enabled:
            out = self._cloud_to_world(msg)
            if out is None:
                return
        else:
            out = self._maybe_override_header(out)
        try:
            self.cloud_pub.publish(out)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("stage2_ego_topic_adapter")
    Stage2EgoTopicAdapterNode()
    rospy.spin()
