#!/usr/bin/env python3
import copy

import rospy

from human_follow_msgs.msg import Target2D


DEFAULT_SEQUENCE = [
    {
        "label": "idle_boot",
        "duration_sec": 0.80,
        "valid": False,
    },
    {
        "label": "valid_center",
        "duration_sec": 2.0,
        "valid": True,
        "track_id": 1,
        "confidence": 0.92,
        "cx": 0.50,
        "cy": 0.50,
        "width": 0.25,
        "height": 0.50,
    },
    {
        "label": "brief_drop_hold",
        "duration_sec": 0.30,
        "valid": False,
    },
    {
        "label": "drop_to_timeout",
        "duration_sec": 0.70,
        "valid": False,
    },
    {
        "label": "reacquire_right",
        "duration_sec": 2.00,
        "valid": True,
        "track_id": 1,
        "confidence": 0.88,
        "cx": 0.68,
        "cy": 0.50,
        "width": 0.26,
        "height": 0.52,
    },
    {
        "label": "search_then_lost",
        "duration_sec": 5.50,
        "valid": False,
    },
    {
        "label": "reacquire_left",
        "duration_sec": 2.00,
        "valid": True,
        "track_id": 1,
        "confidence": 0.90,
        "cx": 0.32,
        "cy": 0.50,
        "width": 0.25,
        "height": 0.50,
    },
]


class TargetSequencePublisherNode:
    def __init__(self):
        self.output_topic = rospy.get_param("~output_topic", "/follow/detector/person_target")
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.loop = bool(rospy.get_param("~loop", True))
        self.sequence = rospy.get_param("~sequence", DEFAULT_SEQUENCE)

        if not self.sequence:
            raise RuntimeError("target sequence is empty")

        self.total_duration_sec = 0.0
        self.compiled_sequence = []
        for phase in self.sequence:
            compiled = copy.deepcopy(phase)
            compiled["duration_sec"] = max(0.0, float(compiled.get("duration_sec", 0.0)))
            self.total_duration_sec += compiled["duration_sec"]
            self.compiled_sequence.append(compiled)

        if self.total_duration_sec <= 0.0:
            raise RuntimeError("target sequence total duration must be positive")

        self.sequence_start_stamp = rospy.Time.now()
        self.last_phase_label = None

        self.publisher = rospy.Publisher(self.output_topic, Target2D, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "target_sequence_publisher ready: output=%s phases=%d total_duration=%.2fs loop=%s",
            self.output_topic,
            len(self.compiled_sequence),
            self.total_duration_sec,
            self.loop,
        )

    def _phase_for_elapsed(self, elapsed_sec):
        if self.loop and self.total_duration_sec > 0.0:
            elapsed_sec = elapsed_sec % self.total_duration_sec
        elif elapsed_sec >= self.total_duration_sec:
            return self.compiled_sequence[-1], len(self.compiled_sequence) - 1

        cursor = 0.0
        for idx, phase in enumerate(self.compiled_sequence):
            cursor += phase["duration_sec"]
            if elapsed_sec <= cursor:
                return phase, idx

        return self.compiled_sequence[-1], len(self.compiled_sequence) - 1

    def _build_message(self, phase):
        msg = Target2D()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        is_valid = bool(phase.get("valid", False))
        msg.valid = is_valid
        msg.track_id = int(phase.get("track_id", 1 if is_valid else -1))
        msg.confidence = float(phase.get("confidence", 0.0 if not is_valid else 0.9))
        msg.cx = float(phase.get("cx", 0.5 if is_valid else 0.0))
        msg.cy = float(phase.get("cy", 0.5 if is_valid else 0.0))
        msg.width = float(phase.get("width", 0.25 if is_valid else 0.0))
        msg.height = float(phase.get("height", 0.50 if is_valid else 0.0))
        label = phase.get("label", "phase")
        msg.source = "sequence_valid:%s" % label if is_valid else "sequence_invalid:%s" % label

        if not is_valid:
            msg.track_id = -1
            msg.confidence = 0.0
            msg.cx = 0.0
            msg.cy = 0.0
            msg.width = 0.0
            msg.height = 0.0

        return msg

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        elapsed_sec = max(0.0, (now - self.sequence_start_stamp).to_sec())
        phase, phase_idx = self._phase_for_elapsed(elapsed_sec)
        label = phase.get("label", "phase_%d" % phase_idx)

        if label != self.last_phase_label:
            rospy.loginfo(
                "target_sequence_publisher phase=%s idx=%d duration=%.2fs valid=%s",
                label,
                phase_idx,
                float(phase.get("duration_sec", 0.0)),
                bool(phase.get("valid", False)),
            )
            self.last_phase_label = label

        try:
            self.publisher.publish(self._build_message(phase))
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("target_sequence_publisher")
    TargetSequencePublisherNode()
    rospy.spin()
