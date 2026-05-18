#!/usr/bin/env python3
import copy
import time

import rospy
from quadrotor_msgs.msg import PositionCommand


DEFAULT_SEQUENCE = [
    {
        "label": "approach_goal",
        "duration_sec": 1.4,
        "position": [1.8, 0.0, 1.6],
        "velocity": [0.6, 0.0, 0.0],
        "yaw": 0.0,
        "yaw_rate": 0.0,
    },
    {
        "label": "lateral_track",
        "duration_sec": 1.4,
        "position": [2.0, 0.5, 1.6],
        "velocity": [0.4, 0.2, 0.0],
        "yaw": 0.12,
        "yaw_rate": 0.05,
    },
    {
        "label": "hold_near_goal",
        "duration_sec": 1.2,
        "position": [2.1, 0.5, 1.6],
        "velocity": [0.0, 0.0, 0.0],
        "yaw": 0.12,
        "yaw_rate": 0.0,
    },
]


class PositionTargetSequencePublisherNode:
    def __init__(self):
        self.output_topic = rospy.get_param("~output_topic", "/follow/stage2/ego_position_cmd")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.loop = bool(rospy.get_param("~loop", False))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.sequence = rospy.get_param("~sequence", DEFAULT_SEQUENCE)

        if not self.sequence:
            raise RuntimeError("position target sequence is empty")

        self.total_duration_sec = 0.0
        self.compiled_sequence = []
        for phase in self.sequence:
            compiled = copy.deepcopy(phase)
            compiled["duration_sec"] = max(0.0, float(compiled.get("duration_sec", 0.0)))
            self.total_duration_sec += compiled["duration_sec"]
            self.compiled_sequence.append(compiled)

        if self.total_duration_sec <= 0.0:
            raise RuntimeError("position target sequence total duration must be positive")

        self.sequence_start_time = time.monotonic()
        self.last_phase_label = None
        self.publisher = rospy.Publisher(self.output_topic, PositionCommand, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "position_target_sequence_publisher ready: output=%s phases=%d total_duration=%.2fs loop=%s",
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
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        position = phase.get("position", [0.0, 0.0, 0.0])
        velocity = phase.get("velocity", [0.0, 0.0, 0.0])
        acceleration = phase.get("acceleration", [0.0, 0.0, 0.0])

        msg.position.x = float(position[0])
        msg.position.y = float(position[1])
        msg.position.z = float(position[2])
        msg.velocity.x = float(velocity[0])
        msg.velocity.y = float(velocity[1])
        msg.velocity.z = float(velocity[2])
        msg.acceleration.x = float(acceleration[0])
        msg.acceleration.y = float(acceleration[1])
        msg.acceleration.z = float(acceleration[2])
        msg.yaw = float(phase.get("yaw", 0.0))
        msg.yaw_dot = float(phase.get("yaw_rate", 0.0))
        msg.vel_norm = abs(msg.velocity.x) + abs(msg.velocity.y) + abs(msg.velocity.z)
        msg.acc_norm = abs(msg.acceleration.x) + abs(msg.acceleration.y) + abs(msg.acceleration.z)
        msg.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        return msg

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        elapsed_sec = max(0.0, time.monotonic() - self.sequence_start_time)
        phase, idx = self._phase_for_elapsed(elapsed_sec)
        label = phase.get("label", "phase_%d" % idx)
        if label != self.last_phase_label:
            rospy.loginfo(
                "position_target_sequence_publisher phase=%s duration=%.2fs",
                label,
                float(phase.get("duration_sec", 0.0)),
            )
            self.last_phase_label = label
        try:
            self.publisher.publish(self._make_msg(phase))
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("position_target_sequence_publisher")
    PositionTargetSequencePublisherNode()
    rospy.spin()
