#!/usr/bin/env python3
"""Extract paper-line Stage2 metrics from a ROS1 bag.

Run this on Ubuntu 20.04 + ROS1 after recording the validation topics. The
script intentionally avoids project-specific plotting dependencies and writes a
small JSON/CSV/Markdown bundle for paper-line evidence triage.
"""

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path


DEFAULT_TOPICS = {
    "target": "/follow/fusion/target_world",
    "odom": "/follow/lio/odom",
    "state": "/follow/stage2/state",
    "goal": "/follow/stage2/goal",
    "ego_goal": "/move_base_simple/goal",
    "ego_cmd": "/follow/stage2/ego_position_cmd",
    "bridge_setpoint": "/follow/stage2/offboard/setpoint",
}


def _import_rosbag():
    try:
        import rosbag  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "rosbag is not importable. Run this on Ubuntu 20.04 after sourcing ROS1/catkin setup."
        ) from exc
    return rosbag


def _stamp_sec(msg, fallback_time):
    try:
        stamp = msg.header.stamp
        value = float(stamp.to_sec())
        if value > 0.0:
            return value
    except AttributeError:
        pass
    return float(fallback_time.to_sec())


def _point_from_pose(msg):
    point = msg.pose.position
    return float(point.x), float(point.y), float(point.z)


def _point_from_position(msg):
    point = msg.position
    return float(point.x), float(point.y), float(point.z)


def _distance(left, right):
    return math.sqrt((left[0] - right[0]) ** 2 + (left[1] - right[1]) ** 2 + (left[2] - right[2]) ** 2)


def _mean(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return 0.0
    return sum(finite) / len(finite)


def _max_gap(stamps):
    if len(stamps) < 2:
        return 0.0
    return max(b - a for a, b in zip(stamps[:-1], stamps[1:]))


def _rate_hz(stamps):
    if len(stamps) < 2:
        return 0.0
    duration = stamps[-1] - stamps[0]
    if duration <= 0.0:
        return 0.0
    return float(len(stamps) - 1) / duration


def _nearest_sample(samples, stamp):
    if not samples:
        return None
    best = min(samples, key=lambda item: abs(item["stamp"] - stamp))
    return best


def _state_durations(states, bag_start, bag_end):
    if not states:
        return {}
    durations = {}
    sorted_states = sorted(states, key=lambda item: item["stamp"])
    for idx, item in enumerate(sorted_states):
        start = item["stamp"]
        end = sorted_states[idx + 1]["stamp"] if idx + 1 < len(sorted_states) else bag_end
        duration = max(0.0, end - start)
        durations[item["state_name"]] = durations.get(item["state_name"], 0.0) + duration
    if sorted_states[0]["stamp"] > bag_start:
        durations["pre_state"] = sorted_states[0]["stamp"] - bag_start
    return durations


def _read_bag(path, topics):
    rosbag = _import_rosbag()
    data = {key: [] for key in topics}
    topic_counts = {topic: 0 for topic in topics.values()}
    bag_start = None
    bag_end = None

    with rosbag.Bag(str(path), "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=list(topics.values())):
            stamp_sec = _stamp_sec(msg, stamp)
            bag_start = stamp_sec if bag_start is None else min(bag_start, stamp_sec)
            bag_end = stamp_sec if bag_end is None else max(bag_end, stamp_sec)
            topic_counts[topic] = topic_counts.get(topic, 0) + 1

            if topic == topics["target"]:
                try:
                    data["target"].append(
                        {
                            "stamp": stamp_sec,
                            "valid": bool(msg.valid),
                            "point": (float(msg.position.x), float(msg.position.y), float(msg.position.z)),
                        }
                    )
                except AttributeError:
                    continue
            elif topic == topics["odom"]:
                try:
                    data["odom"].append({"stamp": stamp_sec, "point": _point_from_pose(msg.pose)})
                except AttributeError:
                    continue
            elif topic == topics["state"]:
                data["state"].append(
                    {
                        "stamp": stamp_sec,
                        "state_name": str(getattr(msg, "state_name", "")),
                        "detail": str(getattr(msg, "detail", "")),
                    }
                )
            elif topic in (topics["goal"], topics["ego_goal"]):
                try:
                    key = "goal" if topic == topics["goal"] else "ego_goal"
                    data[key].append({"stamp": stamp_sec, "point": _point_from_pose(msg)})
                except AttributeError:
                    continue
            elif topic == topics["ego_cmd"]:
                try:
                    data["ego_cmd"].append(
                        {
                            "stamp": stamp_sec,
                            "point": _point_from_position(msg),
                            "velocity": (
                                float(msg.velocity.x),
                                float(msg.velocity.y),
                                float(msg.velocity.z),
                            ),
                            "trajectory_flag": int(getattr(msg, "trajectory_flag", 0)),
                        }
                    )
                except AttributeError:
                    continue
            elif topic == topics["bridge_setpoint"]:
                try:
                    data["bridge_setpoint"].append({"stamp": stamp_sec, "point": _point_from_position(msg)})
                except AttributeError:
                    continue

    return data, topic_counts, (bag_start or 0.0), (bag_end or 0.0)


def _compute_metrics(data, bag_start, bag_end):
    goal_stamps = [item["stamp"] for item in data["goal"]]
    ego_cmd_stamps = [item["stamp"] for item in data["ego_cmd"]]
    state_names = [item["state_name"] for item in data["state"]]

    goal_to_ego_latencies = []
    goal_to_cmd_distances = []
    for goal in data["goal"]:
        nearest_ego_goal = _nearest_sample(data["ego_goal"], goal["stamp"])
        if nearest_ego_goal is not None:
            goal_to_ego_latencies.append(abs(nearest_ego_goal["stamp"] - goal["stamp"]))
        nearest_cmd = _nearest_sample(data["ego_cmd"], goal["stamp"])
        if nearest_cmd is not None:
            goal_to_cmd_distances.append(_distance(goal["point"], nearest_cmd["point"]))

    cmd_steps = [
        _distance(left["point"], right["point"])
        for left, right in zip(data["ego_cmd"][:-1], data["ego_cmd"][1:])
    ]
    bridge_echo_distances = []
    for cmd in data["ego_cmd"]:
        bridge = _nearest_sample(data["bridge_setpoint"], cmd["stamp"])
        if bridge is not None:
            bridge_echo_distances.append(_distance(cmd["point"], bridge["point"]))

    return {
        "bag_duration_sec": max(0.0, bag_end - bag_start),
        "goal_count": len(data["goal"]),
        "ego_cmd_count": len(data["ego_cmd"]),
        "target_count": len(data["target"]),
        "state_count": len(data["state"]),
        "goal_rate_hz": _rate_hz(goal_stamps),
        "ego_cmd_rate_hz": _rate_hz(ego_cmd_stamps),
        "goal_max_gap_sec": _max_gap(goal_stamps),
        "ego_cmd_max_gap_sec": _max_gap(ego_cmd_stamps),
        "goal_to_ego_goal_latency_mean_sec": _mean(goal_to_ego_latencies),
        "goal_to_ego_cmd_distance_mean_m": _mean(goal_to_cmd_distances),
        "ego_cmd_step_mean_m": _mean(cmd_steps),
        "bridge_echo_distance_mean_m": _mean(bridge_echo_distances),
        "state_durations_sec": _state_durations(data["state"], bag_start, bag_end),
        "state_names_seen": sorted(set(name for name in state_names if name)),
        "failsafe_count": sum(1 for name in state_names if name == "failsafe"),
        "hold_safe_count": sum(1 for name in state_names if name == "hold_safe"),
        "search_count": sum(1 for name in state_names if name == "search_safe_viewpoint"),
    }


def _write_topic_counts(path, topic_counts):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["topic", "count"])
        writer.writeheader()
        for topic, count in sorted(topic_counts.items()):
            writer.writerow({"topic": topic, "count": count})


def _write_summary(path, metrics, bag_path):
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# Stage2 ROS Bag Metrics\n\n")
        handle.write("- `bag`: `%s`\n" % bag_path)
        handle.write("- `created_at`: `%s`\n\n" % datetime.now().isoformat(timespec="seconds"))
        handle.write("## Core Metrics\n\n")
        for key in [
            "bag_duration_sec",
            "goal_count",
            "ego_cmd_count",
            "goal_rate_hz",
            "ego_cmd_rate_hz",
            "goal_max_gap_sec",
            "ego_cmd_max_gap_sec",
            "goal_to_ego_goal_latency_mean_sec",
            "goal_to_ego_cmd_distance_mean_m",
            "bridge_echo_distance_mean_m",
            "failsafe_count",
            "hold_safe_count",
            "search_count",
        ]:
            handle.write("- `%s`: %s\n" % (key, metrics[key]))
        handle.write("\n## State Durations\n\n")
        for state_name, duration in sorted(metrics["state_durations_sec"].items()):
            handle.write("- `%s`: %.3f sec\n" % (state_name, duration))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Path to ROS1 bag")
    parser.add_argument("--output-dir", default="", help="Output directory; default is next to bag")
    args = parser.parse_args()

    bag_path = Path(args.bag)
    output_dir = Path(args.output_dir) if args.output_dir else bag_path.with_suffix("").parent / (bag_path.stem + "_metrics")
    output_dir.mkdir(parents=True, exist_ok=True)

    data, topic_counts, bag_start, bag_end = _read_bag(bag_path, DEFAULT_TOPICS)
    metrics = _compute_metrics(data, bag_start, bag_end)

    with (output_dir / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2, sort_keys=True)
    _write_topic_counts(output_dir / "topic_counts.csv", topic_counts)
    _write_summary(output_dir / "summary.md", metrics, str(bag_path))
    print("stage2 rosbag metrics PASS output=%s goal_count=%d ego_cmd_count=%d" % (
        output_dir,
        metrics["goal_count"],
        metrics["ego_cmd_count"],
    ))


if __name__ == "__main__":
    main()
