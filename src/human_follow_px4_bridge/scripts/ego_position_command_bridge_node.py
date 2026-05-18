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
from mavros_msgs.msg import PositionTarget


class EgoPositionCommandBridgeNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/follow/stage2/ego_position_cmd")
        self.output_topic = rospy.get_param("~output_topic", "/follow/stage2/offboard/setpoint")
        self.output_frame_id = rospy.get_param("~output_frame_id", "map")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 30.0))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.5))
        self.use_position = bool(rospy.get_param("~use_position", True))
        self.use_velocity = bool(rospy.get_param("~use_velocity", True))
        self.use_acceleration = bool(rospy.get_param("~use_acceleration", False))
        self.use_yaw = bool(rospy.get_param("~use_yaw", True))
        self.use_yaw_rate = bool(rospy.get_param("~use_yaw_rate", True))

        self.position_target_class = self._resolve_position_command_type()
        self.last_command = None
        self.last_command_stamp = None

        self.publisher = rospy.Publisher(self.output_topic, PositionTarget, queue_size=20)
        self.subscriber = rospy.Subscriber(
            self.input_topic,
            self.position_target_class,
            self._command_callback,
            queue_size=20,
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._publish)

        rospy.loginfo(
            "ego_position_command_bridge ready: input=%s output=%s",
            self.input_topic,
            self.output_topic,
        )

    def _resolve_position_command_type(self):
        try:
            from quadrotor_msgs.msg import PositionCommand
            return PositionCommand
        except ImportError as exc:
            raise RuntimeError(
                "quadrotor_msgs/PositionCommand is required for Stage2 EGO bridge"
            ) from exc

    def _command_callback(self, msg):
        self.last_command = msg
        self.last_command_stamp = rospy.Time.now()

    def _command_is_fresh(self, now):
        if self.last_command is None or self.last_command_stamp is None:
            return False
        return (now - self.last_command_stamp).to_sec() <= self.command_timeout_sec

    def _type_mask(self):
        mask = 0
        if not self.use_position:
            mask |= PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
        if not self.use_velocity:
            mask |= PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
        if not self.use_acceleration:
            mask |= PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
        if not self.use_yaw:
            mask |= PositionTarget.IGNORE_YAW
        if not self.use_yaw_rate:
            mask |= PositionTarget.IGNORE_YAW_RATE
        return mask

    def _make_base_setpoint(self, now):
        msg = PositionTarget()
        msg.header.stamp = now
        msg.header.frame_id = self.output_frame_id
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = self._type_mask()
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

    def _publish(self, _event):
        now = rospy.Time.now()
        msg = self._make_base_setpoint(now)
        if self._command_is_fresh(now):
            cmd = self.last_command
            msg.position.x = float(cmd.position.x)
            msg.position.y = float(cmd.position.y)
            msg.position.z = float(cmd.position.z)
            msg.velocity.x = float(cmd.velocity.x)
            msg.velocity.y = float(cmd.velocity.y)
            msg.velocity.z = float(cmd.velocity.z)
            msg.acceleration_or_force.x = float(cmd.acceleration.x)
            msg.acceleration_or_force.y = float(cmd.acceleration.y)
            msg.acceleration_or_force.z = float(cmd.acceleration.z)
            msg.yaw = float(cmd.yaw)
            msg.yaw_rate = float(cmd.yaw_dot)
        try:
            self.publisher.publish(msg)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("ego_position_command_bridge")
    EgoPositionCommandBridgeNode()
    rospy.spin()
