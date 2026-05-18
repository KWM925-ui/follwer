#!/usr/bin/env python3
import glob
import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

try:
    from runtime_mavros_import import ensure_mavros_python_path
except ImportError:
    def ensure_mavros_python_path():
        candidates = []
        env_prefix = os.environ.get("MAVROS_OVERLAY_PREFIX", "").strip()
        if env_prefix:
            candidates.append(env_prefix)
        candidates.append("/home/coco/.local/ros_noetic_overlay/opt/ros/noetic")

        for prefix in candidates:
            if not prefix:
                continue
            for dist_path in glob.glob(os.path.join(prefix, "lib", "python3*", "dist-packages")):
                if os.path.isdir(os.path.join(dist_path, "mavros_msgs")) and dist_path not in sys.path:
                    sys.path.insert(0, dist_path)

ensure_mavros_python_path()

import rospy

from human_follow_msgs.msg import FollowCommand
from mavros_msgs.msg import PositionTarget


class FollowPx4BridgeNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/control/cmd_body")
        self.output_topic = rospy.get_param("~output_topic", "/follow/offboard/setpoint")
        self.output_frame_id = rospy.get_param("~output_frame_id", "base_link")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.5))
        self.flip_lateral_sign = bool(rospy.get_param("~flip_lateral_sign", True))
        self.flip_yaw_rate_sign = bool(rospy.get_param("~flip_yaw_rate_sign", True))
        self.forward_scale = float(rospy.get_param("~forward_scale", 1.0))
        self.lateral_scale = float(rospy.get_param("~lateral_scale", 1.0))
        self.yaw_rate_scale = float(rospy.get_param("~yaw_rate_scale", 1.0))

        self.last_command = None
        self.last_command_stamp = None
        self.last_logged_mode = None

        self.publisher = rospy.Publisher(self.output_topic, PositionTarget, queue_size=20)
        self.subscriber = rospy.Subscriber(self.input_topic, FollowCommand, self._command_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._publish)

        rospy.loginfo(
            "follow_px4_bridge ready: input=%s output=%s frame=%s",
            self.input_topic,
            self.output_topic,
            self.output_frame_id,
        )

    def _command_callback(self, msg):
        self.last_command = msg
        self.last_command_stamp = rospy.Time.now()
        mode_key = (msg.mode, bool(msg.valid))
        if mode_key != self.last_logged_mode:
            rospy.loginfo(
                "follow_px4_bridge command mode=%s valid=%s forward=%.3f lateral=%.3f yaw_rate=%.3f",
                msg.mode,
                msg.valid,
                msg.forward_mps,
                msg.lateral_mps,
                msg.yaw_rate_rps,
            )
            self.last_logged_mode = mode_key

    def _command_is_fresh(self, now):
        if self.last_command is None or self.last_command_stamp is None:
            return False
        return (now - self.last_command_stamp).to_sec() <= self.command_timeout_sec

    def _make_base_setpoint(self, now):
        msg = PositionTarget()
        msg.header.stamp = now
        msg.header.frame_id = self.output_frame_id
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        msg.position.x = 0.0
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        msg.acceleration_or_force.x = 0.0
        msg.acceleration_or_force.y = 0.0
        msg.acceleration_or_force.z = 0.0
        msg.yaw = 0.0
        msg.yaw_rate = 0.0
        return msg

    def _map_lateral(self, value):
        scale = -1.0 if self.flip_lateral_sign else 1.0
        return float(value) * self.lateral_scale * scale

    def _map_yaw_rate(self, value):
        scale = -1.0 if self.flip_yaw_rate_sign else 1.0
        return float(value) * self.yaw_rate_scale * scale

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        msg = self._make_base_setpoint(now)

        if self._command_is_fresh(now):
            cmd = self.last_command
            if cmd.valid and cmd.mode == "body_velocity_yaw_rate":
                msg.velocity.x = float(cmd.forward_mps) * self.forward_scale
                msg.velocity.y = self._map_lateral(cmd.lateral_mps)
                msg.yaw_rate = self._map_yaw_rate(cmd.yaw_rate_rps)
            elif cmd.valid and cmd.mode == "search_yaw_only":
                msg.yaw_rate = self._map_yaw_rate(cmd.yaw_rate_rps)

        try:
            self.publisher.publish(msg)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("human_follow_px4_bridge")
    FollowPx4BridgeNode()
    rospy.spin()
