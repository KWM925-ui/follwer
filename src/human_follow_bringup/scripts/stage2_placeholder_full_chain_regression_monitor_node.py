#!/usr/bin/env python3
import glob
import math
import os
import sys
import time

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PX4_BRIDGE_SCRIPT_DIR = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", "..", "human_follow_px4_bridge", "scripts"))
for candidate in (_SCRIPT_DIR, _PX4_BRIDGE_SCRIPT_DIR):
    if candidate not in sys.path:
        sys.path.insert(0, candidate)

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
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, UInt32


class Stage2PlaceholderFullChainRegressionMonitorNode:
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/follow/stage2/goal")
        self.ego_goal_topic = rospy.get_param("~ego_goal_topic", "/move_base_simple/goal")
        self.ego_cmd_topic = rospy.get_param("~ego_cmd_topic", "/follow/stage2/ego_position_cmd")
        self.bridge_setpoint_topic = rospy.get_param("~bridge_setpoint_topic", "/follow/stage2/offboard/setpoint")
        self.mavros_setpoint_topic = rospy.get_param("~mavros_setpoint_topic", "/mavros/setpoint_raw/local")
        self.ego_odom_topic = rospy.get_param("~ego_odom_topic", "/odom_world")
        self.grid_odom_topic = rospy.get_param("~grid_odom_topic", "/grid_map/odom")
        self.grid_cloud_topic = rospy.get_param("~grid_cloud_topic", "/grid_map/cloud")
        self.mavros_state_topic = rospy.get_param("~mavros_state_topic", "/mavros/state")
        self.request_count_topic = rospy.get_param("~request_count_topic", "/mavros/fake/set_mode_request_count")
        self.last_request_topic = rospy.get_param("~last_request_topic", "/mavros/fake/last_mode_request")
        self.max_duration_sec = float(rospy.get_param("~max_duration_sec", 12.0))
        self.goal_change_threshold_m = float(rospy.get_param("~goal_change_threshold_m", 0.25))
        self.min_distinct_goals = int(rospy.get_param("~min_distinct_goals", 3))
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 0.12))
        self.velocity_tolerance = float(rospy.get_param("~velocity_tolerance", 0.12))

        self.start_time = time.monotonic()
        self.last_goal = None
        self.last_ego_goal = None
        self.last_ego_cmd = None
        self.last_bridge_setpoint = None
        self.last_mavros_setpoint = None
        self.last_ego_odom = None
        self.last_grid_odom = None
        self.last_grid_cloud = None
        self.last_mavros_state = None
        self.request_count = 0
        self.last_request = ""
        self.distinct_goal_count = 0
        self.distinct_cmd_count = 0
        self._last_counted_goal = None
        self._last_counted_cmd = None
        self.finished = False
        self.exit_code = 0

        rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_callback, queue_size=20)
        rospy.Subscriber(self.ego_goal_topic, PoseStamped, self._ego_goal_callback, queue_size=20)
        rospy.Subscriber(self.ego_cmd_topic, PositionCommand, self._ego_cmd_callback, queue_size=20)
        rospy.Subscriber(self.bridge_setpoint_topic, PositionTarget, self._bridge_setpoint_callback, queue_size=20)
        rospy.Subscriber(self.mavros_setpoint_topic, PositionTarget, self._mavros_setpoint_callback, queue_size=20)
        rospy.Subscriber(self.ego_odom_topic, Odometry, self._ego_odom_callback, queue_size=20)
        rospy.Subscriber(self.grid_odom_topic, Odometry, self._grid_odom_callback, queue_size=20)
        rospy.Subscriber(self.grid_cloud_topic, PointCloud2, self._grid_cloud_callback, queue_size=20)
        rospy.Subscriber(self.mavros_state_topic, State, self._mavros_state_callback, queue_size=20)
        rospy.Subscriber(self.request_count_topic, UInt32, self._request_count_callback, queue_size=20)
        rospy.Subscriber(self.last_request_topic, String, self._last_request_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)

        rospy.loginfo(
            "stage2_placeholder_full_chain_regression_monitor ready goal=%s cmd=%s bridge=%s mavros=%s",
            self.goal_topic,
            self.ego_cmd_topic,
            self.bridge_setpoint_topic,
            self.mavros_setpoint_topic,
        )

    def _maybe_increment(self, point_xyz, last_value, threshold):
        if last_value is None:
            return 1, point_xyz
        dx = point_xyz[0] - last_value[0]
        dy = point_xyz[1] - last_value[1]
        dz = point_xyz[2] - last_value[2]
        if math.sqrt(dx * dx + dy * dy + dz * dz) >= threshold:
            return 1, point_xyz
        return 0, last_value

    def _goal_callback(self, msg):
        self.last_goal = msg
        point_xyz = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        increment, self._last_counted_goal = self._maybe_increment(
            point_xyz, self._last_counted_goal, self.goal_change_threshold_m
        )
        self.distinct_goal_count += increment

    def _ego_goal_callback(self, msg):
        self.last_ego_goal = msg

    def _ego_cmd_callback(self, msg):
        self.last_ego_cmd = msg
        point_xyz = (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        )
        increment, self._last_counted_cmd = self._maybe_increment(
            point_xyz, self._last_counted_cmd, self.goal_change_threshold_m
        )
        self.distinct_cmd_count += increment

    def _bridge_setpoint_callback(self, msg):
        self.last_bridge_setpoint = msg

    def _mavros_setpoint_callback(self, msg):
        self.last_mavros_setpoint = msg

    def _ego_odom_callback(self, msg):
        self.last_ego_odom = msg

    def _grid_odom_callback(self, msg):
        self.last_grid_odom = msg

    def _grid_cloud_callback(self, msg):
        self.last_grid_cloud = msg

    def _mavros_state_callback(self, msg):
        self.last_mavros_state = msg

    def _request_count_callback(self, msg):
        self.request_count = int(msg.data)

    def _last_request_callback(self, msg):
        self.last_request = msg.data

    def _close(self, a, b, tol):
        return math.isclose(float(a), float(b), abs_tol=tol)

    def _fail(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 1
        rospy.logerr("stage2 placeholder full-chain regression FAIL %s", reason)
        rospy.signal_shutdown(reason)

    def _pass(self, reason):
        if self.finished:
            return
        self.finished = True
        self.exit_code = 0
        rospy.loginfo("stage2 placeholder full-chain regression PASS %s", reason)
        rospy.signal_shutdown(reason)

    def _check_contracts(self):
        if self.last_goal is None:
            return False, "waiting_stage2_goal"
        if self.last_ego_goal is None:
            return False, "waiting_ego_goal"
        if self.last_ego_cmd is None:
            return False, "waiting_ego_cmd"
        if self.last_bridge_setpoint is None:
            return False, "waiting_bridge_setpoint"
        if self.last_mavros_setpoint is None:
            return False, "waiting_mavros_setpoint"
        if self.last_ego_odom is None or self.last_grid_odom is None:
            return False, "waiting_adapter_odom"
        if self.last_grid_cloud is None or int(self.last_grid_cloud.width) <= 0:
            return False, "waiting_adapter_cloud"
        if self.last_mavros_state is None or self.last_mavros_state.mode != State.MODE_PX4_OFFBOARD:
            return False, "waiting_offboard_mode"
        if self.request_count != 1 or self.last_request != "OFFBOARD":
            return False, "waiting_offboard_request"
        if self.distinct_goal_count < self.min_distinct_goals:
            return False, "goal_changes<%d" % self.min_distinct_goals
        if self.distinct_cmd_count < self.min_distinct_goals:
            return False, "cmd_changes<%d" % self.min_distinct_goals

        goal = self.last_goal.pose.position
        ego_goal = self.last_ego_goal.pose.position
        if (
            not self._close(goal.x, ego_goal.x, self.position_tolerance)
            or not self._close(goal.y, ego_goal.y, self.position_tolerance)
            or not self._close(goal.z, ego_goal.z, self.position_tolerance)
        ):
            return False, "goal_adapter_echo_mismatch"

        cmd = self.last_ego_cmd
        setpoint = self.last_bridge_setpoint
        if (
            not self._close(setpoint.position.x, cmd.position.x, self.position_tolerance)
            or not self._close(setpoint.position.y, cmd.position.y, self.position_tolerance)
            or not self._close(setpoint.position.z, cmd.position.z, self.position_tolerance)
            or not self._close(setpoint.velocity.x, cmd.velocity.x, self.velocity_tolerance)
            or not self._close(setpoint.velocity.y, cmd.velocity.y, self.velocity_tolerance)
            or not self._close(setpoint.velocity.z, cmd.velocity.z, self.velocity_tolerance)
        ):
            return False, "bridge_echo_mismatch"

        mavros_sp = self.last_mavros_setpoint
        if (
            not self._close(mavros_sp.position.x, setpoint.position.x, self.position_tolerance)
            or not self._close(mavros_sp.position.y, setpoint.position.y, self.position_tolerance)
            or not self._close(mavros_sp.position.z, setpoint.position.z, self.position_tolerance)
        ):
            return False, "gate_forward_mismatch"

        if setpoint.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            return False, "bridge_coordinate_frame_mismatch"
        if mavros_sp.coordinate_frame != PositionTarget.FRAME_LOCAL_NED:
            return False, "mavros_coordinate_frame_mismatch"
        return True, "full_chain_ok"

    def _tick(self, _event):
        if self.finished:
            return
        ok, reason = self._check_contracts()
        if ok:
            self._pass(
                "distinct_goals=%d distinct_cmds=%d request_count=%d"
                % (self.distinct_goal_count, self.distinct_cmd_count, self.request_count)
            )
            return
        if (time.monotonic() - self.start_time) >= self.max_duration_sec:
            self._fail(
                "timeout reason=%s distinct_goals=%d distinct_cmds=%d request_count=%d last_request=%s"
                % (reason, self.distinct_goal_count, self.distinct_cmd_count, self.request_count, self.last_request)
            )


if __name__ == "__main__":
    rospy.init_node("stage2_placeholder_full_chain_regression_monitor")
    node = Stage2PlaceholderFullChainRegressionMonitorNode()
    rospy.spin()
    sys.exit(node.exit_code)
