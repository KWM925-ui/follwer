#!/usr/bin/env python3
"""Run a small local 2-D paper-line simulation.

This is intentionally lightweight: it does not require ROS, MATLAB, numpy, or
matplotlib. The goal is to exercise the paper-line decision ideas on Ubuntu
before moving to the Windows/MATLAB figure and batch workflow.
"""

import argparse
import csv
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float


@dataclass
class Candidate:
    name: str
    x: float
    y: float
    score: float = 0.0


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def mul(a, scale):
    return (a[0] * scale, a[1] * scale)


def norm(a):
    return math.hypot(a[0], a[1])


def unit(a, fallback=(1.0, 0.0)):
    length = norm(a)
    if length < 1e-9:
        return fallback
    return (a[0] / length, a[1] / length)


def distance(a, b):
    return norm(sub(a, b))


def min_point_clearance(point, obstacles):
    if not obstacles:
        return 99.0
    return min(distance(point, (obs.x, obs.y)) - obs.radius for obs in obstacles)


def segment_clearance(start, end, obstacles):
    if not obstacles:
        return 99.0
    best = 99.0
    seg = sub(end, start)
    seg_len2 = seg[0] * seg[0] + seg[1] * seg[1]
    for obs in obstacles:
        center = (obs.x, obs.y)
        if seg_len2 < 1e-9:
            closest = start
        else:
            ratio = ((center[0] - start[0]) * seg[0] + (center[1] - start[1]) * seg[1]) / seg_len2
            ratio = clamp(ratio, 0.0, 1.0)
            closest = (start[0] + ratio * seg[0], start[1] + ratio * seg[1])
        best = min(best, distance(closest, center) - obs.radius)
    return best


def scenario_at(name, t):
    if name == "target_loss":
        target = (0.45 * t, 0.55 * math.sin(0.55 * t))
        dropout = (6.0 <= t <= 6.8) or (10.0 <= t <= 12.2)
        obstacles = [
            Obstacle(4.0, 1.2, 0.55),
            Obstacle(6.6, -1.0, 0.60),
        ]
        return target, dropout, obstacles

    if name == "safety_margin":
        target = (0.50 * t, 0.18 * math.sin(0.35 * t))
        dropout = False
        obstacles = [
            Obstacle(1.35, -0.15, 0.58),
            Obstacle(2.80, 0.18, 0.58),
            Obstacle(4.25, -0.15, 0.58),
            Obstacle(5.70, 0.18, 0.58),
            Obstacle(7.15, -0.15, 0.58),
        ]
        return target, dropout, obstacles

    if name == "planner_feedback":
        target = (0.44 * t, -0.15 + 0.25 * math.sin(0.45 * t))
        dropout = False
        obstacles = [
            Obstacle(4.0, 0.85, 0.55),
            Obstacle(6.0, -0.90, 0.55),
        ]
        return target, dropout, obstacles

    raise ValueError("unknown scenario: %s" % name)


def generate_candidates(target, velocity, uav, state, condition):
    desired = 3.0
    forward = unit(velocity, unit(sub(target, uav), (1.0, 0.0)))
    behind = (-forward[0], -forward[1])
    left = (-forward[1], forward[0])

    if state == "search":
        angles = [-55.0, -25.0, 0.0, 25.0, 55.0]
        base = math.atan2(uav[1] - target[1], uav[0] - target[0])
        out = []
        for index, deg in enumerate(angles):
            heading = base + math.radians(deg)
            out.append(Candidate("search_%d" % (index + 1), target[0] + 3.8 * math.cos(heading), target[1] + 3.8 * math.sin(heading)))
        return out

    candidates = [
        Candidate("behind", target[0] + behind[0] * desired, target[1] + behind[1] * desired),
        Candidate("left", target[0] - forward[0] * 1.0 + left[0] * 2.3, target[1] - forward[1] * 1.0 + left[1] * 2.3),
        Candidate("right", target[0] - forward[0] * 1.0 - left[0] * 2.3, target[1] - forward[1] * 1.0 - left[1] * 2.3),
        Candidate("far_safe", target[0] + behind[0] * 4.8, target[1] + behind[1] * 4.8),
    ]
    if condition == "behind_only":
        return [candidates[0]]
    return candidates


def dynamic_margin(condition, uav_speed, uncertainty):
    base = 0.35 + 0.45
    if condition == "fixed_margin":
        return base
    return base + 0.30 * max(uav_speed, 0.0) + 1.20 * math.sqrt(max(uncertainty, 0.0))


def score_candidate(candidate, target, velocity, uav, obstacles, margin, last_name):
    desired = 3.0
    follow_distance = distance((candidate.x, candidate.y), target)
    distance_score = clamp(1.0 - abs(follow_distance - desired) / desired, 0.0, 1.0)
    clearance = min_point_clearance((candidate.x, candidate.y), obstacles)
    clearance_score = clamp((clearance - margin) / 2.5, 0.0, 1.0)
    motion_score = 1.0 / (1.0 + distance(uav, (candidate.x, candidate.y)))

    forward = unit(velocity, (1.0, 0.0))
    preferred_view = (-forward[0], -forward[1])
    actual_view = unit(sub((candidate.x, candidate.y), target), preferred_view)
    angle_score = clamp((actual_view[0] * preferred_view[0] + actual_view[1] * preferred_view[1] + 1.0) * 0.5, 0.0, 1.0)
    switch_score = 1.0 if candidate.name == last_name else 0.65
    candidate.score = (
        0.26 * distance_score
        + 0.26 * clearance_score
        + 0.24 * angle_score
        + 0.14 * motion_score
        + 0.10 * switch_score
    )
    return candidate.score


def planner_accepts(scenario, t, candidate):
    if scenario != "planner_feedback":
        return True
    if 6.0 <= t <= 12.0 and candidate.name == "behind":
        return False
    return True


def state_from_loss(target_seen, loss_time):
    if target_seen:
        return "follow"
    if loss_time <= 1.2:
        return "predict_hold"
    if loss_time <= 5.0:
        return "search"
    return "hold"


def run_one(scenario, condition, seed):
    rng = random.Random(seed)
    dt = 0.1
    duration = 18.0
    steps = int(duration / dt) + 1

    first_target, _dropout, _obstacles = scenario_at(scenario, 0.0)
    uav = (first_target[0] - 3.2, first_target[1] - 0.3)
    previous_uav = uav
    estimate = first_target
    previous_estimate = estimate
    velocity = (0.45, 0.0)
    uncertainty = 0.20
    loss_time = 0.0
    cooldown = {}
    last_candidate_name = ""

    rows = []
    planner_failure_burst = 0
    planner_failure_burst_max = 0
    planner_failure_count = 0
    near_miss_count = 0
    loss_duration = 0.0
    candidate_switch_count = 0
    state_switch_count = 0
    last_state = ""
    selected_names = []
    clearances = []
    distance_errors = []

    for step in range(steps):
        t = step * dt
        target, dropout, obstacles = scenario_at(scenario, t)
        visible = not dropout
        if visible:
            noisy = (target[0] + rng.gauss(0.0, 0.04), target[1] + rng.gauss(0.0, 0.04))
            previous_estimate = estimate
            estimate = noisy
            velocity = mul(sub(estimate, previous_estimate), 1.0 / dt)
            uncertainty = max(0.04, uncertainty * 0.72)
            loss_time = 0.0
        else:
            estimate = add(estimate, mul(velocity, dt))
            uncertainty = min(8.0, uncertainty + 0.18)
            loss_time += dt
            loss_duration += dt

        state = state_from_loss(visible, loss_time)
        if condition == "no_recovery" and state in ("predict_hold", "search"):
            state = "hold"

        if state == "hold":
            selected = Candidate("hold", uav[0], uav[1])
            planner_ok = True
        else:
            decision_target = estimate
            if condition == "no_prediction":
                decision_target = previous_estimate if visible else estimate
            elif state in ("follow", "predict_hold", "search"):
                decision_target = add(estimate, mul(velocity, 0.6))

            uav_speed = distance(uav, previous_uav) / dt if step > 0 else 0.0
            margin = dynamic_margin(condition, uav_speed, uncertainty)
            candidates = generate_candidates(decision_target, velocity, uav, "search" if state == "search" else "follow", condition)

            now = t
            candidates = [
                item for item in candidates
                if cooldown.get(item.name, -1.0) <= now
            ]

            safe = []
            for item in candidates:
                point = (item.x, item.y)
                if min_point_clearance(point, obstacles) < margin:
                    continue
                if segment_clearance(uav, point, obstacles) < margin:
                    continue
                score_candidate(item, decision_target, velocity, uav, obstacles, margin, last_candidate_name)
                safe.append(item)

            if safe:
                safe.sort(key=lambda item: item.score, reverse=True)
                selected = safe[0]
                planner_ok = planner_accepts(scenario, t, selected)
                if not planner_ok and condition != "no_planner_feedback":
                    cooldown[selected.name] = t + 1.8
            else:
                selected = Candidate("hold", uav[0], uav[1])
                planner_ok = True

        if not planner_ok:
            planner_failure_count += 1
            planner_failure_burst += 1
            planner_failure_burst_max = max(planner_failure_burst_max, planner_failure_burst)
        else:
            planner_failure_burst = 0

        selected_names.append(selected.name)
        if last_candidate_name and selected.name != last_candidate_name:
            candidate_switch_count += 1
        if last_state and state != last_state:
            state_switch_count += 1

        previous_uav = uav
        if planner_ok and selected.name != "hold":
            goal = (selected.x, selected.y)
            delta = sub(goal, uav)
            max_step = 1.2 * dt
            move = mul(unit(delta, (0.0, 0.0)), min(norm(delta), max_step))
            uav = add(uav, move)

        clearance = min_point_clearance(uav, obstacles)
        clearances.append(clearance)
        if clearance < 1.25:
            near_miss_count += 1
        distance_errors.append(abs(distance(uav, target) - 3.0))
        last_candidate_name = selected.name
        last_state = state
        rows.append(
            {
                "time": round(t, 3),
                "target_x": target[0],
                "target_y": target[1],
                "uav_x": uav[0],
                "uav_y": uav[1],
                "state": state,
                "candidate": selected.name,
                "planner_ok": planner_ok,
                "clearance": clearance,
            }
        )

    task_success = int(near_miss_count == 0 and planner_failure_burst_max <= 4 and loss_duration <= 5.5)
    return {
        "scenario": scenario,
        "condition": condition,
        "seed": seed,
        "mean_distance_error": sum(distance_errors) / len(distance_errors),
        "min_clearance": min(clearances),
        "near_miss_count": near_miss_count,
        "loss_duration": loss_duration,
        "planner_failure_count": planner_failure_count,
        "planner_failure_burst_max": planner_failure_burst_max,
        "candidate_switch_count": candidate_switch_count,
        "state_switch_count": state_switch_count,
        "task_success": task_success,
        "state_names": sorted(set(row["state"] for row in rows)),
        "candidate_names": sorted(set(selected_names)),
        "trace": rows,
    }


def mean(values):
    return sum(values) / len(values) if values else 0.0


def summarize(runs):
    groups = {}
    for run in runs:
        key = (run["scenario"], run["condition"])
        groups.setdefault(key, []).append(run)

    out = []
    for (scenario, condition), items in sorted(groups.items()):
        out.append(
            {
                "scenario": scenario,
                "condition": condition,
                "n": len(items),
                "mean_distance_error": mean([item["mean_distance_error"] for item in items]),
                "min_clearance": min(item["min_clearance"] for item in items),
                "near_miss_count": mean([item["near_miss_count"] for item in items]),
                "loss_duration": mean([item["loss_duration"] for item in items]),
                "planner_failure_count": mean([item["planner_failure_count"] for item in items]),
                "planner_failure_burst_max": mean([item["planner_failure_burst_max"] for item in items]),
                "candidate_switch_count": mean([item["candidate_switch_count"] for item in items]),
                "task_success": mean([item["task_success"] for item in items]),
            }
        )
    return out


def write_csv(path, rows, fieldnames):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, "") for name in fieldnames})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", default="1,2,3,4,5")
    parser.add_argument("--output-dir", default="research/outputs/local_sim")
    args = parser.parse_args()

    seeds = [int(item.strip()) for item in args.seeds.split(",") if item.strip()]
    scenarios = ["target_loss", "safety_margin", "planner_feedback"]
    conditions = ["proposed", "fixed_margin", "no_planner_feedback", "behind_only", "no_recovery"]

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    runs = []
    traces = {}
    for scenario in scenarios:
        for condition in conditions:
            for seed in seeds:
                result = run_one(scenario, condition, seed)
                traces["%s__%s__seed_%d" % (scenario, condition, seed)] = result.pop("trace")
                runs.append(result)

    summary = summarize(runs)
    write_csv(
        output_dir / "local_sim_runs.csv",
        runs,
        [
            "scenario",
            "condition",
            "seed",
            "mean_distance_error",
            "min_clearance",
            "near_miss_count",
            "loss_duration",
            "planner_failure_count",
            "planner_failure_burst_max",
            "candidate_switch_count",
            "state_switch_count",
            "task_success",
            "state_names",
            "candidate_names",
        ],
    )
    write_csv(
        output_dir / "local_sim_summary.csv",
        summary,
        [
            "scenario",
            "condition",
            "n",
            "mean_distance_error",
            "min_clearance",
            "near_miss_count",
            "loss_duration",
            "planner_failure_count",
            "planner_failure_burst_max",
            "candidate_switch_count",
            "task_success",
        ],
    )
    (output_dir / "local_sim_summary.json").write_text(
        json.dumps({"runs": runs, "summary": summary}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    interesting_traces = {
        key: value
        for key, value in traces.items()
        if key in (
            "target_loss__proposed__seed_1",
            "safety_margin__fixed_margin__seed_1",
            "safety_margin__proposed__seed_1",
            "planner_feedback__no_planner_feedback__seed_1",
            "planner_feedback__proposed__seed_1",
        )
    }
    (output_dir / "local_sim_traces_seed1.json").write_text(
        json.dumps(interesting_traces, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    lines = ["# Local Paper-Line Simulation", ""]
    lines.append("This is a small Ubuntu-side 2-D simulation, not the final MATLAB batch.")
    lines.append("")
    lines.append("| scenario | condition | near_miss | planner_failures | max_failure_burst | task_success |")
    lines.append("|---|---|---:|---:|---:|---:|")
    for row in summary:
        lines.append(
            "| {scenario} | {condition} | {near_miss_count:.2f} | {planner_failure_count:.2f} | {planner_failure_burst_max:.2f} | {task_success:.2f} |".format(
                **row
            )
        )
    (output_dir / "local_sim_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("local sim PASS output=%s runs=%d" % (output_dir, len(runs)))
    for row in summary:
        if row["scenario"] in ("safety_margin", "planner_feedback") and row["condition"] in (
            "proposed",
            "fixed_margin",
            "no_planner_feedback",
        ):
            print(
                "%s %-20s near_miss=%.2f planner_fail=%.2f burst=%.2f success=%.2f"
                % (
                    row["scenario"],
                    row["condition"],
                    row["near_miss_count"],
                    row["planner_failure_count"],
                    row["planner_failure_burst_max"],
                    row["task_success"],
                )
            )


if __name__ == "__main__":
    main()
