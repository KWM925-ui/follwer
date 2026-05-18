#!/usr/bin/env python3
import copy

import rospy
from std_msgs.msg import String

from human_follow_msgs.msg import Target3D


DEFAULT_SEQUENCE = [
    {
        "label": "idle_boot",
        "duration_sec": 1.0,
        "truth_valid": False,
        "visible": False,
    },
    {
        "label": "approach_center",
        "duration_sec": 1.8,
        "truth_valid": True,
        "visible": True,
        "start_xyz": [5.4, 0.0, 0.0],
        "end_xyz": [4.2, 0.0, 0.0],
    },
    {
        "label": "stop_go_pause",
        "duration_sec": 1.0,
        "truth_valid": True,
        "visible": True,
        "position_xyz": [4.2, 0.0, 0.0],
    },
    {
        "label": "short_occlusion",
        "duration_sec": 0.4,
        "truth_valid": True,
        "visible": False,
        "position_xyz": [4.1, -0.1, 0.0],
    },
    {
        "label": "reacquire_right",
        "duration_sec": 1.8,
        "truth_valid": True,
        "visible": True,
        "start_xyz": [4.1, -0.8, 0.0],
        "end_xyz": [4.0, -1.2, 0.0],
    },
    {
        "label": "retreat_center",
        "duration_sec": 1.6,
        "truth_valid": True,
        "visible": True,
        "start_xyz": [4.0, -1.2, 0.0],
        "end_xyz": [4.8, -0.1, 0.0],
    },
    {
        "label": "long_loss",
        "duration_sec": 12.0,
        "truth_valid": True,
        "visible": False,
        "start_xyz": [4.8, -0.1, 0.0],
        "end_xyz": [4.4, 0.8, 0.0],
    },
    {
        "label": "reacquire_left",
        "duration_sec": 2.0,
        "truth_valid": True,
        "visible": True,
        "start_xyz": [4.4, 1.0, 0.0],
        "end_xyz": [3.9, 0.6, 0.0],
    },
]


PRESET_SEQUENCES = {
    "stage2_visual_ego_autoplay": [
        {
            "label": "idle_boot",
            "duration_sec": 0.8,
            "truth_valid": False,
            "visible": False,
        },
        {
            "label": "ego_follow_right_bias",
            "duration_sec": 8.0,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [10.5, 1.8, 0.5],
            "end_xyz": [10.5, 1.2, 0.5],
        },
        {
            "label": "ego_follow_center_obstacle",
            "duration_sec": 10.0,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [10.8, 0.2, 0.5],
            "end_xyz": [11.2, -0.2, 0.5],
        },
        {
            "label": "ego_follow_left_bias",
            "duration_sec": 8.0,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [10.5, -1.6, 0.5],
            "end_xyz": [10.0, -2.2, 0.5],
        },
    ],
    "stage2_visual_ego_replan_showcase": [
        {
            "label": "idle_boot",
            "duration_sec": 0.8,
            "truth_valid": False,
            "visible": False,
        },
        {
            "label": "slalom_prime_right",
            "duration_sec": 3.0,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [8.8, 1.4, 0.5],
            "end_xyz": [9.2, 1.8, 0.5],
        },
        {
            "label": "slalom_cross_left",
            "duration_sec": 5.5,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [9.2, 1.8, 0.5],
            "end_xyz": [8.8, -1.8, 0.5],
        },
        {
            "label": "slalom_cross_right",
            "duration_sec": 5.5,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [8.8, -1.8, 0.5],
            "end_xyz": [9.0, 1.6, 0.5],
        },
        {
            "label": "slalom_cross_left_again",
            "duration_sec": 5.5,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [9.0, 1.6, 0.5],
            "end_xyz": [8.7, -1.7, 0.5],
        },
        {
            "label": "slalom_settle_center",
            "duration_sec": 3.5,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [8.7, -0.4, 0.5],
            "end_xyz": [9.4, 0.1, 0.5],
        },
    ],
    "acquire_center": [
        {
            "label": "idle_boot",
            "duration_sec": 0.8,
            "truth_valid": False,
            "visible": False,
        },
        {
            "label": "acquire_center",
            "duration_sec": 1.8,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [4.8, 0.0, 0.0],
            "end_xyz": [4.2, 0.0, 0.0],
        },
        {
            "label": "settle_center",
            "duration_sec": 1.6,
            "truth_valid": True,
            "visible": True,
            "position_xyz": [4.2, 0.0, 0.0],
        },
    ],
    "search_reacquire_right": [
        {
            "label": "idle_boot",
            "duration_sec": 0.8,
            "truth_valid": False,
            "visible": False,
        },
        {
            "label": "prime_right",
            "duration_sec": 1.2,
            "truth_valid": True,
            "visible": True,
            "position_xyz": [4.1, -0.9, 0.0],
        },
        {
            "label": "search_loss_right",
            "duration_sec": 2.4,
            "truth_valid": True,
            "visible": False,
            "position_xyz": [4.1, -0.9, 0.0],
        },
        {
            "label": "reacquire_right",
            "duration_sec": 1.8,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [4.1, -0.9, 0.0],
            "end_xyz": [4.0, -0.4, 0.0],
        },
        {
            "label": "settle_center",
            "duration_sec": 1.4,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [4.0, -0.4, 0.0],
            "end_xyz": [4.0, 0.0, 0.0],
        },
    ],
    "search_reacquire_left": [
        {
            "label": "idle_boot",
            "duration_sec": 0.8,
            "truth_valid": False,
            "visible": False,
        },
        {
            "label": "prime_left",
            "duration_sec": 1.2,
            "truth_valid": True,
            "visible": True,
            "position_xyz": [4.1, 0.9, 0.0],
        },
        {
            "label": "search_loss_left",
            "duration_sec": 2.4,
            "truth_valid": True,
            "visible": False,
            "position_xyz": [4.1, 0.9, 0.0],
        },
        {
            "label": "reacquire_left",
            "duration_sec": 1.8,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [4.1, 0.9, 0.0],
            "end_xyz": [4.0, 0.4, 0.0],
        },
        {
            "label": "settle_center",
            "duration_sec": 1.4,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [4.0, 0.4, 0.0],
            "end_xyz": [4.0, 0.0, 0.0],
        },
    ],
    "person_approach_retreat": [
        {
            "label": "idle_boot",
            "duration_sec": 0.8,
            "truth_valid": False,
            "visible": False,
        },
        {
            "label": "approach_retreat_center",
            "duration_sec": 4.0,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [3.6, 0.0, 0.0],
            "end_xyz": [2.2, 0.0, 0.0],
        },
        {
            "label": "retreat_hold_center",
            "duration_sec": 1.6,
            "truth_valid": True,
            "visible": True,
            "position_xyz": [2.2, 0.0, 0.0],
        },
    ],
    "person_depart_follow": [
        {
            "label": "idle_boot",
            "duration_sec": 0.8,
            "truth_valid": False,
            "visible": False,
        },
        {
            "label": "depart_follow_center",
            "duration_sec": 4.0,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [4.4, 0.0, 0.0],
            "end_xyz": [6.2, 0.0, 0.0],
        },
        {
            "label": "depart_hold_center",
            "duration_sec": 1.6,
            "truth_valid": True,
            "visible": True,
            "position_xyz": [6.2, 0.0, 0.0],
        },
    ],
    "lateral_left_track": [
        {
            "label": "idle_boot",
            "duration_sec": 0.8,
            "truth_valid": False,
            "visible": False,
        },
        {
            "label": "lateral_left_track",
            "duration_sec": 3.6,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [4.1, 0.0, 0.0],
            "end_xyz": [4.1, 1.2, 0.0],
        },
        {
            "label": "settle_left",
            "duration_sec": 1.6,
            "truth_valid": True,
            "visible": True,
            "position_xyz": [4.1, 1.2, 0.0],
        },
    ],
    "lateral_right_track": [
        {
            "label": "idle_boot",
            "duration_sec": 0.8,
            "truth_valid": False,
            "visible": False,
        },
        {
            "label": "lateral_right_track",
            "duration_sec": 3.6,
            "truth_valid": True,
            "visible": True,
            "start_xyz": [4.1, 0.0, 0.0],
            "end_xyz": [4.1, -1.2, 0.0],
        },
        {
            "label": "settle_right",
            "duration_sec": 1.6,
            "truth_valid": True,
            "visible": True,
            "position_xyz": [4.1, -1.2, 0.0],
        },
    ],
}


MOTION_MODE_ALIASES = {
    "scripted": "scripted_standard",
    "standard": "scripted_standard",
    "search_right": "search_reacquire_right",
    "search_left": "search_reacquire_left",
    "approach_retreat": "person_approach_retreat",
    "retreat_follow": "person_depart_follow",
    "depart_follow": "person_depart_follow",
    "follow_far": "person_depart_follow",
    "follow_near": "person_approach_retreat",
    "left_track": "lateral_left_track",
    "right_track": "lateral_right_track",
    "stage2_ego_demo": "stage2_visual_ego_autoplay",
}


def _phase_descriptor(label, visible, truth_valid, phase_index):
    return "label=%s;visible=%d;truth_valid=%d;phase_index=%d" % (
        label,
        1 if visible else 0,
        1 if truth_valid else 0,
        int(phase_index),
    )


def _as_xyz_list(value):
    xyz = [float(v) for v in value]
    if len(xyz) != 3:
        raise ValueError("xyz must have exactly 3 values")
    return xyz


class HumanTruthSequencePublisherNode:
    def __init__(self):
        self.truth_topic = rospy.get_param("~truth_topic", "/follow/sim/truth_target_body")
        self.phase_topic = rospy.get_param("~phase_topic", "/follow/sim/truth_phase")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.track_id = int(rospy.get_param("~track_id", 1))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.loop = bool(rospy.get_param("~loop", True))
        requested_motion_mode = str(rospy.get_param("~motion_mode", "scripted_standard")).strip().lower()
        self.motion_mode = MOTION_MODE_ALIASES.get(requested_motion_mode, requested_motion_mode)
        self.sequence = rospy.get_param("~sequence", DEFAULT_SEQUENCE)
        self.random_seed = int(rospy.get_param("~random_seed", 13))

        if self.motion_mode in PRESET_SEQUENCES:
            self.sequence = copy.deepcopy(PRESET_SEQUENCES[self.motion_mode])
        elif self.motion_mode != "scripted_standard":
            supported_modes = sorted(["scripted_standard"] + list(PRESET_SEQUENCES.keys()))
            raise RuntimeError("unsupported motion_mode=%s supported=%s" % (self.motion_mode, ",".join(supported_modes)))

        if not self.sequence:
            raise RuntimeError("truth sequence is empty")

        self.compiled_sequence = []
        self.total_duration_sec = 0.0
        for phase in self.sequence:
            compiled = copy.deepcopy(phase)
            compiled["duration_sec"] = max(0.0, float(compiled.get("duration_sec", 0.0)))
            compiled["label"] = str(compiled.get("label", "phase"))
            compiled["truth_valid"] = bool(compiled.get("truth_valid", compiled.get("valid", False)))
            compiled["visible"] = bool(compiled.get("visible", compiled["truth_valid"]))

            if "position_xyz" in compiled:
                compiled["start_xyz"] = _as_xyz_list(compiled["position_xyz"])
                compiled["end_xyz"] = _as_xyz_list(compiled["position_xyz"])
            else:
                compiled["start_xyz"] = _as_xyz_list(compiled.get("start_xyz", [0.0, 0.0, 0.0]))
                compiled["end_xyz"] = _as_xyz_list(compiled.get("end_xyz", compiled["start_xyz"]))

            self.total_duration_sec += compiled["duration_sec"]
            self.compiled_sequence.append(compiled)

        if self.total_duration_sec <= 0.0:
            raise RuntimeError("truth sequence total duration must be positive")

        self.sequence_start_stamp = rospy.Time.now()
        self.last_phase_label = None

        self.truth_publisher = rospy.Publisher(self.truth_topic, Target3D, queue_size=10)
        self.phase_publisher = rospy.Publisher(self.phase_topic, String, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-3)), self._tick)

        rospy.loginfo(
            "human_truth_sequence_publisher ready: truth=%s phase=%s phases=%d total_duration=%.2fs loop=%s mode=%s",
            self.truth_topic,
            self.phase_topic,
            len(self.compiled_sequence),
            self.total_duration_sec,
            self.loop,
            self.motion_mode,
        )

    def _phase_for_elapsed(self, elapsed_sec):
        if self.loop and self.total_duration_sec > 0.0:
            elapsed_sec = elapsed_sec % self.total_duration_sec
        elif elapsed_sec >= self.total_duration_sec:
            return self.compiled_sequence[-1], len(self.compiled_sequence) - 1, 1.0

        cursor = 0.0
        for idx, phase in enumerate(self.compiled_sequence):
            duration = phase["duration_sec"]
            next_cursor = cursor + duration
            if elapsed_sec <= next_cursor or idx == len(self.compiled_sequence) - 1:
                if duration <= 1e-6:
                    alpha = 1.0
                else:
                    alpha = min(max((elapsed_sec - cursor) / duration, 0.0), 1.0)
                return phase, idx, alpha
            cursor = next_cursor

        return self.compiled_sequence[-1], len(self.compiled_sequence) - 1, 1.0

    def _interpolate_position(self, phase, alpha):
        start_xyz = phase["start_xyz"]
        end_xyz = phase["end_xyz"]
        return [
            float(start_xyz[i]) + (float(end_xyz[i]) - float(start_xyz[i])) * float(alpha)
            for i in range(3)
        ]

    def _build_truth_message(self, phase, phase_index, alpha):
        msg = Target3D()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.track_id = self.track_id
        msg.frame_id = self.frame_id

        truth_valid = bool(phase["truth_valid"])
        msg.valid = truth_valid
        msg.confidence = 1.0 if truth_valid else 0.0
        msg.support_points = 0

        xyz = self._interpolate_position(phase, alpha) if truth_valid else [0.0, 0.0, 0.0]
        msg.position.x = float(xyz[0])
        msg.position.y = float(xyz[1])
        msg.position.z = float(xyz[2])

        descriptor = String()
        descriptor.data = _phase_descriptor(
            phase["label"],
            bool(phase["visible"]),
            truth_valid,
            phase_index,
        )
        return msg, descriptor

    def _tick(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        elapsed_sec = max(0.0, (now - self.sequence_start_stamp).to_sec())
        phase, phase_index, alpha = self._phase_for_elapsed(elapsed_sec)
        label = phase["label"]

        if label != self.last_phase_label:
            rospy.loginfo(
                "human_truth_sequence phase=%s idx=%d duration=%.2fs visible=%s truth_valid=%s",
                label,
                phase_index,
                phase["duration_sec"],
                phase["visible"],
                phase["truth_valid"],
            )
            self.last_phase_label = label

        truth_msg, phase_msg = self._build_truth_message(phase, phase_index, alpha)
        try:
            self.truth_publisher.publish(truth_msg)
            self.phase_publisher.publish(phase_msg)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("human_truth_sequence_publisher")
    HumanTruthSequencePublisherNode()
    rospy.spin()
