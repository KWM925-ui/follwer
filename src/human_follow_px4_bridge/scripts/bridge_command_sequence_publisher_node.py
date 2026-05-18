#!/usr/bin/env python3
import copy
import time

import rospy

from human_follow_msgs.msg import FollowCommand


DEFAULT_SEQUENCE = [
    {
        "label": "track_right",
        "duration_sec": 1.5,
        "valid": True,
        "mode": "body_velocity_yaw_rate",
        "forward_mps": 1.1,
        "lateral_mps": 0.4,
        "yaw_rate_rps": 0.3,
    },
    {
        "label": "track_left",
        "duration_sec": 1.5,
        "valid": True,
        "mode": "body_velocity_yaw_rate",
        "forward_mps": 0.8,
        "lateral_mps": -0.5,
        "yaw_rate_rps": -0.25,
    },
    {
        "label": "search",
        "duration_sec": 1.2,
        "valid": True,
        "mode": "search_yaw_only",
        "forward_mps": 0.0,
        "lateral_mps": 0.0,
        "yaw_rate_rps": 0.2,
    },
    {
        "label": "hold_invalid",
        "duration_sec": 1.2,
        "valid": False,
        "mode": "hold",
        "forward_mps": 0.0,
        "lateral_mps": 0.0,
        "yaw_rate_rps": 0.0,
    },
]


class BridgeCommandSequencePublisherNode:
    def __init__(self):
        self.output_topic = rospy.get_param("~output_topic", "/follow/control/cmd_body")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.loop = bool(rospy.get_param("~loop", False))
        self.sequence = rospy.get_param("~sequence", DEFAULT_SEQUENCE)

        if not self.sequence:
            raise RuntimeError("bridge command sequence is empty")

        self.total_duration_sec = 0.0
        self.compiled_sequence = []
        for phase in self.sequence:
            compiled = copy.deepcopy(phase)
            compiled["duration_sec"] = max(0.0, float(compiled.get("duration_sec", 0.0)))
            self.total_duration_sec += compiled["duration_sec"]
            self.compiled_sequence.append(compiled)

        if self.total_duration_sec <= 0.0:
            raise RuntimeError("bridge command sequence total duration must be positive")

        self.sequence_start_time = time.monotonic()
        self.last_phase_label = None

        self.publisher = rospy.Publisher(self.output_topic, FollowCommand, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "bridge_command_sequence_publisher ready: output=%s phases=%d total_duration=%.2fs loop=%s",
            self.output_topic,
            len(self.compiled_sequence),
            self.total_duration_sec,
            self.loop,
        )

    def _phase_for_elapsed(self, elapsed_sec):
        if self.loop:
            elapsed_sec = elapsed_sec % self.total_duration_sec
        elif elapsed_sec >= self.total_duration_sec:
            return self.compiled_sequence[-1], len(self.compiled_sequence) - 1

        cursor = 0.0
        for idx, phase in enumerate(self.compiled_sequence):
            cursor += phase["duration_sec"]
            if elapsed_sec <= cursor:
                return phase, idx
        return self.compiled_sequence[-1], len(self.compiled_sequence) - 1

    def _make_msg(self, phase):
        msg = FollowCommand()
        msg.header.stamp = rospy.Time.now()
        msg.valid = bool(phase.get("valid", False))
        msg.mode = str(phase.get("mode", "hold"))
        msg.forward_mps = float(phase.get("forward_mps", 0.0))
        msg.lateral_mps = float(phase.get("lateral_mps", 0.0))
        msg.yaw_rate_rps = float(phase.get("yaw_rate_rps", 0.0))
        return msg

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        elapsed_sec = max(0.0, time.monotonic() - self.sequence_start_time)
        phase, idx = self._phase_for_elapsed(elapsed_sec)
        label = phase.get("label", "phase_%d" % idx)
        if label != self.last_phase_label:
            rospy.loginfo(
                "bridge_command_sequence_publisher phase=%s mode=%s valid=%s duration=%.2fs",
                label,
                phase.get("mode", "hold"),
                bool(phase.get("valid", False)),
                float(phase.get("duration_sec", 0.0)),
            )
            self.last_phase_label = label
        try:
            self.publisher.publish(self._make_msg(phase))
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("bridge_command_sequence_publisher")
    BridgeCommandSequencePublisherNode()
    rospy.spin()
