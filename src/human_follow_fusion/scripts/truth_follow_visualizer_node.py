#!/usr/bin/env python3
import math
import os
import sys
import threading
from collections import deque

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA, String
from quadrotor_msgs.msg import PositionCommand
from visualization_msgs.msg import Marker, MarkerArray

from human_follow_msgs.msg import FollowCommand, FollowState, Target3D

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from projection_math import load_camera_intrinsics_yaml  # noqa: E402


def yaw_from_quaternion(x_value, y_value, z_value, w_value):
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


def set_identity_orientation(marker):
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0


class TruthFollowVisualizerNode:
    def __init__(self):
        self.truth_body_topic = rospy.get_param("~truth_body_topic", "/follow/sim/truth_target_body")
        self.truth_world_topic = rospy.get_param("~truth_world_topic", "/follow/sim/human_truth_world")
        self.fusion_topic = rospy.get_param("~fusion_topic", "/follow/fusion/target_body")
        self.command_topic = rospy.get_param("~command_topic", "/follow/control/cmd_body")
        self.odom_topic = rospy.get_param("~odom_topic", "/follow/sim/vehicle_odom")
        self.phase_topic = rospy.get_param("~phase_topic", "/follow/sim/truth_phase")
        self.follow_state_topic = rospy.get_param("~follow_state_topic", "/follow/state")
        self.stage2_state_topic = rospy.get_param("~stage2_state_topic", "/follow/stage2/state")
        self.stage2_goal_topic = rospy.get_param("~stage2_goal_topic", "/follow/stage2/goal")
        self.stage2_ego_goal_topic = rospy.get_param("~stage2_ego_goal_topic", "/follow/stage2/ego_goal")
        self.stage2_ego_cmd_topic = rospy.get_param("~stage2_ego_cmd_topic", "/follow/stage2/ego_position_cmd")
        self.marker_topic = rospy.get_param("~marker_topic", "/follow/viz/markers")
        self.truth_path_topic = rospy.get_param("~truth_path_topic", "/follow/viz/truth_path")
        self.fusion_path_topic = rospy.get_param("~fusion_path_topic", "/follow/viz/fusion_path")
        self.vehicle_path_topic = rospy.get_param("~vehicle_path_topic", "/follow/viz/vehicle_path")
        self.intrinsics_yaml = rospy.get_param("~intrinsics_yaml", "")
        self.world_frame_id = rospy.get_param("~world_frame_id", "map")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")
        self.path_length = max(20, int(rospy.get_param("~path_length", 240)))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 12.0))
        self.command_arrow_scale = float(rospy.get_param("~command_arrow_scale", 1.5))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.8))
        self.state_timeout_sec = float(rospy.get_param("~state_timeout_sec", 1.0))
        self.frustum_depth_m = float(rospy.get_param("~frustum_depth_m", 2.2))
        self.phase_text_height_m = float(rospy.get_param("~phase_text_height_m", 1.4))

        self.intrinsics = load_camera_intrinsics_yaml(self.intrinsics_yaml) if self.intrinsics_yaml else None
        default_camera_frame_id = self.intrinsics["camera_frame"] if self.intrinsics is not None else "camera"
        self.camera_frame_id = rospy.get_param("~camera_frame_id", default_camera_frame_id)
        self.camera_mesh_frame_id = rospy.get_param("~camera_mesh_frame_id", "camera")
        self.lidar_frame_id = rospy.get_param("~lidar_frame_id", "mid360")
        self.lidar_mesh_frame_id = rospy.get_param("~lidar_mesh_frame_id", "mid360")

        self.path_lock = threading.Lock()
        self.truth_path = deque(maxlen=self.path_length)
        self.fusion_path = deque(maxlen=self.path_length)
        self.vehicle_path = deque(maxlen=self.path_length)
        self.last_truth_body = None
        self.last_truth_world = None
        self.last_fusion = None
        self.last_command = None
        self.last_command_time = None
        self.last_odom = None
        self.last_phase_label = "idle_boot"
        self.last_phase_visible = True
        self.last_phase_display_visible = True
        self.last_follow_state_name = "idle"
        self.last_follow_state_detail = ""
        self.last_follow_state_time = None
        self.last_stage2_state_name = "idle"
        self.last_stage2_state_detail = ""
        self.last_stage2_state_time = None
        self.last_stage2_goal = None
        self.last_stage2_ego_goal = None
        self.last_stage2_ego_cmd = None
        self.last_stage2_ego_cmd_time = None

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=10)
        self.truth_path_pub = rospy.Publisher(self.truth_path_topic, Path, queue_size=10)
        self.fusion_path_pub = rospy.Publisher(self.fusion_path_topic, Path, queue_size=10)
        self.vehicle_path_pub = rospy.Publisher(self.vehicle_path_topic, Path, queue_size=10)

        self.truth_body_sub = rospy.Subscriber(self.truth_body_topic, Target3D, self._truth_body_callback, queue_size=10)
        self.truth_world_sub = rospy.Subscriber(self.truth_world_topic, Target3D, self._truth_world_callback, queue_size=10)
        self.fusion_sub = rospy.Subscriber(self.fusion_topic, Target3D, self._fusion_callback, queue_size=10)
        self.command_sub = rospy.Subscriber(self.command_topic, FollowCommand, self._command_callback, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=10)
        self.phase_sub = rospy.Subscriber(self.phase_topic, String, self._phase_callback, queue_size=10)
        self.follow_state_sub = rospy.Subscriber(self.follow_state_topic, FollowState, self._follow_state_callback, queue_size=10)
        self.stage2_state_sub = rospy.Subscriber(self.stage2_state_topic, FollowState, self._stage2_state_callback, queue_size=10)
        self.stage2_goal_sub = rospy.Subscriber(self.stage2_goal_topic, PoseStamped, self._stage2_goal_callback, queue_size=10)
        self.stage2_ego_goal_sub = rospy.Subscriber(self.stage2_ego_goal_topic, PoseStamped, self._stage2_ego_goal_callback, queue_size=10)
        self.stage2_ego_cmd_sub = rospy.Subscriber(self.stage2_ego_cmd_topic, PositionCommand, self._stage2_ego_cmd_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "truth_follow_visualizer ready: markers=%s truth_path=%s fusion_path=%s vehicle_path=%s",
            self.marker_topic,
            self.truth_path_topic,
            self.fusion_path_topic,
            self.vehicle_path_topic,
        )

    def _truth_body_callback(self, msg):
        self.last_truth_body = msg

    def _truth_world_callback(self, msg):
        self.last_truth_world = msg
        if msg.valid:
            with self.path_lock:
                self.truth_path.append([float(msg.position.x), float(msg.position.y), float(msg.position.z)])

    def _fusion_callback(self, msg):
        self.last_fusion = msg
        if msg.valid:
            fusion_world = self._body_point_to_world(msg.position.x, msg.position.y, msg.position.z)
            if fusion_world is not None:
                with self.path_lock:
                    self.fusion_path.append(fusion_world)

    def _command_callback(self, msg):
        self.last_command = msg
        self.last_command_time = rospy.Time.now()

    def _odom_callback(self, msg):
        self.last_odom = msg
        with self.path_lock:
            self.vehicle_path.append(
                [
                    float(msg.pose.pose.position.x),
                    float(msg.pose.pose.position.y),
                    float(msg.pose.pose.position.z),
                ]
            )

    def _phase_callback(self, msg):
        parts = [part.strip() for part in str(msg.data or "").split(";") if part.strip()]
        for part in parts:
            if part.startswith("label="):
                self.last_phase_label = part.split("=", 1)[1] or "phase"
            elif part.startswith("visible="):
                self.last_phase_visible = part.split("=", 1)[1] == "1"
                self.last_phase_display_visible = self.last_phase_visible
            elif part.startswith("display_visible="):
                self.last_phase_display_visible = part.split("=", 1)[1] == "1"

    def _follow_state_callback(self, msg):
        state_name = str(getattr(msg, "state_name", "") or "").strip()
        detail = str(getattr(msg, "detail", "") or "").strip()
        if state_name:
            self.last_follow_state_name = state_name
        self.last_follow_state_detail = detail
        self.last_follow_state_time = rospy.Time.now()

    def _stage2_state_callback(self, msg):
        state_name = str(getattr(msg, "state_name", "") or "").strip()
        detail = str(getattr(msg, "detail", "") or "").strip()
        if state_name:
            self.last_stage2_state_name = state_name
        self.last_stage2_state_detail = detail
        self.last_stage2_state_time = rospy.Time.now()

    def _stage2_goal_callback(self, msg):
        self.last_stage2_goal = msg

    def _stage2_ego_goal_callback(self, msg):
        self.last_stage2_ego_goal = msg

    def _stage2_ego_cmd_callback(self, msg):
        self.last_stage2_ego_cmd = msg
        self.last_stage2_ego_cmd_time = rospy.Time.now()

    def _body_point_to_world(self, x_value, y_value, z_value):
        if self.last_odom is None:
            return None
        odom = self.last_odom
        yaw_rad = yaw_from_quaternion(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        world_x = float(odom.pose.pose.position.x) + cos_yaw * float(x_value) - sin_yaw * float(y_value)
        world_y = float(odom.pose.pose.position.y) + sin_yaw * float(x_value) + cos_yaw * float(y_value)
        world_z = float(odom.pose.pose.position.z) + float(z_value)
        return [world_x, world_y, world_z]

    def _make_pose(self, frame_id, point_xyz):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(point_xyz[0])
        pose.pose.position.y = float(point_xyz[1])
        pose.pose.position.z = float(point_xyz[2])
        pose.pose.orientation.w = 1.0
        return pose

    def _build_path(self, frame_id, points_xyz):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame_id
        for point_xyz in points_xyz:
            path.poses.append(self._make_pose(frame_id, point_xyz))
        return path

    def _delete_marker(self, marker_id, namespace, frame_id=None):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id or self.world_frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _make_truth_marker(self):
        if self.last_truth_world is None or not self.last_truth_world.valid:
            return self._delete_marker(1, "truth", self.world_frame_id)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.world_frame_id
        marker.ns = "truth"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.last_truth_world.position.x)
        marker.pose.position.y = float(self.last_truth_world.position.y)
        marker.pose.position.z = float(self.last_truth_world.position.z + 0.85)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.55
        marker.scale.y = 0.55
        marker.scale.z = 1.70
        if self.last_phase_display_visible:
            marker.color = ColorRGBA(r=0.18, g=0.92, b=0.26, a=0.72)
        else:
            marker.color = ColorRGBA(r=0.78, g=0.82, b=0.88, a=0.24)
        return marker

    def _make_fusion_marker(self):
        if self.last_fusion is None or not self.last_fusion.valid:
            return self._delete_marker(2, "fusion", self.base_frame_id)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.base_frame_id
        marker.ns = "fusion"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.last_fusion.position.x)
        marker.pose.position.y = float(self.last_fusion.position.y)
        marker.pose.position.z = float(self.last_fusion.position.z + 0.10)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.28
        marker.scale.y = 0.28
        marker.scale.z = 0.28
        marker.color = ColorRGBA(r=1.0, g=0.46, b=0.05, a=0.95)
        return marker

    def _make_command_marker(self):
        now = rospy.Time.now()
        if (
            self.last_command is None
            or self.last_command_time is None
            or (now - self.last_command_time).to_sec() > self.command_timeout_sec
        ):
            return self._delete_marker(3, "command", self.base_frame_id)

        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.base_frame_id
        marker.ns = "command"
        marker.id = 3
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        set_identity_orientation(marker)
        marker.scale.x = 0.08
        marker.scale.y = 0.14
        marker.scale.z = 0.20
        marker.color = ColorRGBA(r=0.10, g=0.62, b=1.0, a=0.98)
        marker.points = [
            Point(x=0.0, y=0.0, z=0.25),
            Point(
                x=float(self.last_command.forward_mps) * self.command_arrow_scale,
                y=float(self.last_command.lateral_mps) * self.command_arrow_scale,
                z=0.25,
            ),
        ]
        return marker

    def _make_vehicle_outline_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.base_frame_id
        marker.ns = "vehicle_outline"
        marker.id = 4
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        set_identity_orientation(marker)
        marker.scale.x = 0.04
        marker.color = ColorRGBA(r=0.95, g=0.95, b=0.98, a=0.88)
        arm = 0.32
        skid = 0.18
        marker.points = [
            Point(x=-arm, y=-arm, z=0.02), Point(x=arm, y=arm, z=0.02),
            Point(x=-arm, y=arm, z=0.02), Point(x=arm, y=-arm, z=0.02),
            Point(x=-0.08, y=-skid, z=-0.10), Point(x=0.12, y=-skid, z=-0.10),
            Point(x=-0.08, y=skid, z=-0.10), Point(x=0.12, y=skid, z=-0.10),
        ]
        return marker

    def _make_camera_sensor_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.camera_mesh_frame_id
        marker.ns = "camera_sensor"
        marker.id = 5
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.z = 0.06
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color = ColorRGBA(r=0.95, g=0.45, b=0.20, a=0.90)
        return marker

    def _make_lidar_sensor_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.lidar_mesh_frame_id
        marker.ns = "lidar_sensor"
        marker.id = 6
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.z = 0.03
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.06
        marker.color = ColorRGBA(r=0.35, g=0.78, b=1.0, a=0.92)
        return marker

    def _make_camera_frustum_marker(self):
        if self.intrinsics is None:
            return self._delete_marker(7, "camera_frustum", self.camera_frame_id)

        depth = max(0.5, self.frustum_depth_m)
        width = float(self.intrinsics["image_width"])
        height = float(self.intrinsics["image_height"])
        fx = float(self.intrinsics["fx"])
        fy = float(self.intrinsics["fy"])
        cx = float(self.intrinsics["cx"])
        cy = float(self.intrinsics["cy"])

        corners_pixels = [
            (0.0, 0.0),
            (width, 0.0),
            (width, height),
            (0.0, height),
        ]
        corners_xyz = []
        for u_coord, v_coord in corners_pixels:
            x_coord = (u_coord - cx) / fx * depth
            y_coord = (v_coord - cy) / fy * depth
            corners_xyz.append(Point(x=x_coord, y=y_coord, z=depth))

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.camera_frame_id
        marker.ns = "camera_frustum"
        marker.id = 7
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        set_identity_orientation(marker)
        marker.scale.x = 0.02
        marker.color = ColorRGBA(r=1.0, g=0.82, b=0.18, a=0.95)
        origin = Point(x=0.0, y=0.0, z=0.0)
        for corner in corners_xyz:
            marker.points.append(origin)
            marker.points.append(corner)
        for idx in range(4):
            marker.points.append(corners_xyz[idx])
            marker.points.append(corners_xyz[(idx + 1) % 4])
        return marker

    def _make_phase_text_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.base_frame_id
        marker.ns = "phase_text"
        marker.id = 8
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = self.phase_text_height_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.26
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        marker.text = "phase: %s [%s]" % (
            self.last_phase_label,
            "shown" if self.last_phase_display_visible else "hidden",
        )
        return marker

    def _make_follow_state_text_marker(self):
        now = rospy.Time.now()
        if (
            self.last_follow_state_time is None
            or (now - self.last_follow_state_time).to_sec() > self.state_timeout_sec
        ):
            return self._delete_marker(10, "follow_state_text", self.base_frame_id)
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.base_frame_id
        marker.ns = "follow_state_text"
        marker.id = 10
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = self.phase_text_height_m - 0.26
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color = ColorRGBA(r=0.75, g=0.90, b=1.0, a=0.95)
        if self.last_follow_state_detail:
            marker.text = "stage1_state: %s | %s" % (self.last_follow_state_name, self.last_follow_state_detail)
        else:
            marker.text = "stage1_state: %s" % self.last_follow_state_name
        return marker

    def _make_stage2_state_text_marker(self):
        now = rospy.Time.now()
        if (
            self.last_stage2_state_time is None
            or (now - self.last_stage2_state_time).to_sec() > self.state_timeout_sec
        ):
            return self._delete_marker(14, "stage2_state_text", self.base_frame_id)
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.base_frame_id
        marker.ns = "stage2_state_text"
        marker.id = 14
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = self.phase_text_height_m - 0.48
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color = ColorRGBA(r=1.0, g=0.92, b=0.25, a=0.96)
        if self.last_stage2_state_detail:
            marker.text = "stage2_state: %s | %s" % (self.last_stage2_state_name, self.last_stage2_state_detail)
        else:
            marker.text = "stage2_state: %s" % self.last_stage2_state_name
        return marker

    def _make_world_sphere_marker(self, marker_id, namespace, pose_msg, color, scale_xyz):
        if pose_msg is None:
            return self._delete_marker(marker_id, namespace, self.world_frame_id)
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = pose_msg.header.frame_id or self.world_frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_msg.pose
        marker.scale.x = scale_xyz[0]
        marker.scale.y = scale_xyz[1]
        marker.scale.z = scale_xyz[2]
        marker.color = color
        return marker

    def _make_stage2_goal_marker(self):
        return self._make_world_sphere_marker(
            11,
            "stage2_goal",
            self.last_stage2_goal,
            ColorRGBA(r=1.0, g=0.92, b=0.15, a=0.90),
            (0.30, 0.30, 0.30),
        )

    def _make_stage2_ego_goal_marker(self):
        return self._make_world_sphere_marker(
            12,
            "stage2_ego_goal",
            self.last_stage2_ego_goal,
            ColorRGBA(r=1.0, g=0.10, b=0.55, a=0.92),
            (0.24, 0.24, 0.24),
        )

    def _make_stage2_ego_cmd_marker(self):
        now = rospy.Time.now()
        if (
            self.last_stage2_ego_cmd is None
            or self.last_stage2_ego_cmd_time is None
            or (now - self.last_stage2_ego_cmd_time).to_sec() > self.command_timeout_sec
        ):
            return self._delete_marker(13, "stage2_ego_cmd", self.world_frame_id)

        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.world_frame_id
        marker.ns = "stage2_ego_cmd"
        marker.id = 13
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        set_identity_orientation(marker)
        marker.scale.x = 0.08
        marker.scale.y = 0.14
        marker.scale.z = 0.16
        marker.color = ColorRGBA(r=1.0, g=0.30, b=0.95, a=0.96)
        start_point = Point(
            x=float(self.last_stage2_ego_cmd.position.x),
            y=float(self.last_stage2_ego_cmd.position.y),
            z=float(self.last_stage2_ego_cmd.position.z),
        )
        end_point = Point(
            x=float(self.last_stage2_ego_cmd.position.x + self.last_stage2_ego_cmd.velocity.x),
            y=float(self.last_stage2_ego_cmd.position.y + self.last_stage2_ego_cmd.velocity.y),
            z=float(self.last_stage2_ego_cmd.position.z + self.last_stage2_ego_cmd.velocity.z),
        )
        marker.points = [start_point, end_point]
        return marker

    def _make_vehicle_heading_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.base_frame_id
        marker.ns = "vehicle_heading"
        marker.id = 9
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        set_identity_orientation(marker)
        marker.scale.x = 0.07
        marker.scale.y = 0.13
        marker.scale.z = 0.16
        marker.color = ColorRGBA(r=1.0, g=0.20, b=0.20, a=0.96)
        marker.points = [
            Point(x=0.0, y=0.0, z=0.10),
            Point(x=0.45, y=0.0, z=0.10),
        ]
        return marker

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        with self.path_lock:
            truth_path = list(self.truth_path)
            fusion_path = list(self.fusion_path)
            vehicle_path = list(self.vehicle_path)
        markers = MarkerArray()
        markers.markers.append(self._make_truth_marker())
        markers.markers.append(self._make_fusion_marker())
        markers.markers.append(self._make_command_marker())
        markers.markers.append(self._make_vehicle_outline_marker())
        markers.markers.append(self._make_camera_sensor_marker())
        markers.markers.append(self._make_lidar_sensor_marker())
        markers.markers.append(self._make_camera_frustum_marker())
        markers.markers.append(self._make_phase_text_marker())
        markers.markers.append(self._make_vehicle_heading_marker())
        markers.markers.append(self._make_follow_state_text_marker())
        markers.markers.append(self._make_stage2_state_text_marker())
        markers.markers.append(self._make_stage2_goal_marker())
        markers.markers.append(self._make_stage2_ego_goal_marker())
        markers.markers.append(self._make_stage2_ego_cmd_marker())
        try:
            self.marker_pub.publish(markers)
            self.truth_path_pub.publish(self._build_path(self.world_frame_id, truth_path))
            self.fusion_path_pub.publish(self._build_path(self.world_frame_id, fusion_path))
            self.vehicle_path_pub.publish(self._build_path(self.world_frame_id, vehicle_path))
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("truth_follow_visualizer")
    TruthFollowVisualizerNode()
    rospy.spin()
