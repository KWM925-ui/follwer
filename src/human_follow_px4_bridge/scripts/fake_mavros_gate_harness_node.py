#!/usr/bin/env python3
import glob
import os
import sys
import time

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

from mavros_msgs.msg import EstimatorStatus, PositionTarget, State
from mavros_msgs.srv import SetMode, SetModeResponse
from std_msgs.msg import String, UInt32


class FakeMavrosGateHarnessNode:
    def __init__(self):
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.estimator_topic = rospy.get_param("~estimator_topic", "/mavros/estimator_status")
        self.setpoint_topic = rospy.get_param("~setpoint_topic", "/mavros/setpoint_raw/local")
        self.set_mode_service = rospy.get_param("~set_mode_service", "/mavros/set_mode")
        self.request_count_topic = rospy.get_param("~request_count_topic", "/mavros/fake/set_mode_request_count")
        self.last_request_topic = rospy.get_param("~last_request_topic", "/mavros/fake/last_mode_request")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.connected_after_sec = float(rospy.get_param("~connected_after_sec", 0.0))
        self.armed_after_sec = float(rospy.get_param("~armed_after_sec", 0.6))
        self.estimator_valid_after_sec = float(rospy.get_param("~estimator_valid_after_sec", 0.8))
        self.initial_mode = rospy.get_param("~initial_mode", "STABILIZED")

        self.start_time = time.monotonic()
        self.current_mode = self.initial_mode
        self.request_count = 0
        self.last_request = ""
        self.forwarded_setpoint_count = 0

        self.state_pub = rospy.Publisher(self.state_topic, State, queue_size=20)
        self.estimator_pub = rospy.Publisher(self.estimator_topic, EstimatorStatus, queue_size=20)
        self.request_count_pub = rospy.Publisher(self.request_count_topic, UInt32, queue_size=20, latch=True)
        self.last_request_pub = rospy.Publisher(self.last_request_topic, String, queue_size=20, latch=True)
        self.setpoint_sub = rospy.Subscriber(self.setpoint_topic, PositionTarget, self._setpoint_callback, queue_size=20)
        self.set_mode_srv = rospy.Service(self.set_mode_service, SetMode, self._set_mode_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "fake_mavros_gate_harness ready state=%s estimator=%s set_mode=%s initial_mode=%s",
            self.state_topic,
            self.estimator_topic,
            self.set_mode_service,
            self.initial_mode,
        )

    def _elapsed(self):
        return max(0.0, time.monotonic() - self.start_time)

    def _setpoint_callback(self, _msg):
        self.forwarded_setpoint_count += 1

    def _set_mode_callback(self, request):
        self.request_count += 1
        self.last_request = request.custom_mode
        self.current_mode = request.custom_mode or self.current_mode
        try:
            self.request_count_pub.publish(UInt32(data=self.request_count))
            self.last_request_pub.publish(String(data=self.last_request))
        except rospy.ROSException:
            return SetModeResponse(mode_sent=False)
        rospy.loginfo(
            "fake_mavros_gate_harness set_mode request=%s count=%d",
            request.custom_mode,
            self.request_count,
        )
        return SetModeResponse(mode_sent=True)

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        elapsed = self._elapsed()

        state = State()
        state.header.stamp = rospy.Time.now()
        state.connected = elapsed >= self.connected_after_sec
        state.armed = elapsed >= self.armed_after_sec
        state.guided = False
        state.manual_input = True
        state.mode = self.current_mode
        try:
            self.state_pub.publish(state)
        except rospy.ROSException:
            return

        estimator = EstimatorStatus()
        estimator.header.stamp = rospy.Time.now()
        estimator_valid = elapsed >= self.estimator_valid_after_sec
        estimator.attitude_status_flag = True
        estimator.velocity_horiz_status_flag = estimator_valid
        estimator.velocity_vert_status_flag = estimator_valid
        estimator.pos_horiz_rel_status_flag = estimator_valid
        estimator.pos_horiz_abs_status_flag = False
        estimator.pos_vert_abs_status_flag = estimator_valid
        estimator.pos_vert_agl_status_flag = True
        estimator.const_pos_mode_status_flag = False
        estimator.pred_pos_horiz_rel_status_flag = False
        estimator.pred_pos_horiz_abs_status_flag = False
        estimator.gps_glitch_status_flag = False
        estimator.accel_error_status_flag = False
        try:
            self.estimator_pub.publish(estimator)
            self.request_count_pub.publish(UInt32(data=self.request_count))
            self.last_request_pub.publish(String(data=self.last_request))
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("fake_mavros_gate_harness")
    FakeMavrosGateHarnessNode()
    rospy.spin()
