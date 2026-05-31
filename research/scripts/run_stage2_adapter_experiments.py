#!/usr/bin/env python3
"""Offline baseline/ablation experiments for the paper-line Stage2 adapter.

The runner uses the ROS1 adapter's pure Python decision core without starting
ROS. Generated CSV/JSON/Markdown outputs go under research/runs by default and
are intentionally ignored by git.
"""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import random
import statistics
import time

from smoke_ros1_stage2_adapter_core import install_stubs, load_adapter, make_uninitialized_node


DT_SEC = 0.1
RUN_DURATION_SEC = 18.0
TARGET_Z = 1.7
UAV_Z = 1.2
FOLLOW_DISTANCE_M = 3.5
MAX_UAV_SPEED_MPS = 1.3
RESPONSE_GAIN = 0.85
SAFETY_SHELL_M = 0.45


@dataclass
class Obstacle:
    x: float
    y: float
    z: float
    radius: float


@dataclass
class Scenario:
    name: str
    obstacles: list
    loss_windows: list
    planner_block_windows: list


class _Point:
    def __init__(self, x_value=0.0, y_value=0.0, z_value=0.0):
        self.x = float(x_value)
        self.y = float(y_value)
        self.z = float(z_value)


class _Target:
    def __init__(self, x_value, y_value, z_value=TARGET_Z):
        self.position = _Point(x_value, y_value, z_value)


def _target_xy(scenario_name, t_sec):
    if scenario_name == "open_straight":
        return 0.46 * t_sec, 0.22 * math.sin(0.35 * t_sec)
    if scenario_name == "sharp_turn":
        if t_sec < 6.0:
            return 0.52 * t_sec, 0.10 * math.sin(0.4 * t_sec)
        if t_sec < 12.0:
            return 3.12 + 0.10 * (t_sec - 6.0), 0.62 * (t_sec - 6.0)
        return 3.72 + 0.48 * (t_sec - 12.0), 3.72 - 0.10 * (t_sec - 12.0)
    if scenario_name == "occlusion_corridor":
        return 0.45 * t_sec, 0.72 * math.sin(0.48 * t_sec)
    if scenario_name == "target_loss_reacquire":
        return 0.43 * t_sec, 0.50 * math.sin(0.58 * t_sec)
    if scenario_name == "planner_blocked":
        return 0.42 * t_sec, -0.15 + 0.30 * math.sin(0.45 * t_sec)
    raise ValueError("unknown scenario %s" % scenario_name)


def _make_scenarios():
    return [
        Scenario("open_straight", [Obstacle(6.5, 3.0, UAV_Z, 0.55)], [], []),
        Scenario(
            "sharp_turn",
            [Obstacle(3.6, 1.5, UAV_Z, 0.70), Obstacle(4.4, 3.2, UAV_Z, 0.65)],
            [],
            [],
        ),
        Scenario(
            "occlusion_corridor",
            [
                Obstacle(2.8, -0.55, UAV_Z, 0.70),
                Obstacle(4.5, 0.45, UAV_Z, 0.75),
                Obstacle(6.2, -0.65, UAV_Z, 0.70),
                Obstacle(7.8, 0.55, UAV_Z, 0.75),
            ],
            [],
            [],
        ),
        Scenario(
            "target_loss_reacquire",
            [Obstacle(4.2, 1.0, UAV_Z, 0.65), Obstacle(6.8, -0.9, UAV_Z, 0.60)],
            [(6.8, 8.2), (13.0, 14.1)],
            [],
        ),
        Scenario(
            "planner_blocked",
            [
                Obstacle(3.2, -0.75, UAV_Z, 0.65),
                Obstacle(4.9, 0.55, UAV_Z, 0.70),
                Obstacle(6.8, -0.65, UAV_Z, 0.70),
            ],
            [],
            [(7.2, 11.8)],
        ),
    ]


def _conditions():
    return [
        "fixed_behind",
        "side_only",
        "no_far_safe",
        "nearest_feasible",
        "paper_line_full",
        "no_prediction",
        "fixed_safety_margin",
        "no_hard_filter",
        "no_planner_feedback",
    ]


def _inside_any_window(t_sec, windows):
    return any(start <= t_sec <= end for start, end in windows)


def _distance_xy(left_xy, right_xy):
    return math.hypot(left_xy[0] - right_xy[0], left_xy[1] - right_xy[1])


def _min_obstacle_clearance(point_xyz, obstacles):
    if not obstacles:
        return 99.0
    return min(
        math.sqrt((point_xyz[0] - obs.x) ** 2 + (point_xyz[1] - obs.y) ** 2 + (point_xyz[2] - obs.z) ** 2)
        - obs.radius
        for obs in obstacles
    )


def _segment_clearance(start_xyz, end_xyz, obstacles):
    samples = max(int(math.ceil(_distance_xy(start_xyz, end_xyz) / 0.15)), 1)
    best = 99.0
    for idx in range(samples + 1):
        ratio = float(idx) / float(samples)
        point = (
            start_xyz[0] + (end_xyz[0] - start_xyz[0]) * ratio,
            start_xyz[1] + (end_xyz[1] - start_xyz[1]) * ratio,
            start_xyz[2] + (end_xyz[2] - start_xyz[2]) * ratio,
        )
        best = min(best, _min_obstacle_clearance(point, obstacles))
    return best


def _occupied_cells_for(node, obstacles):
    occupied = set()
    for obs in obstacles:
        span = int(math.ceil(obs.radius / node.occupancy_resolution_m))
        center = node._cell_key(obs.x, obs.y, obs.z)
        for ix in range(center[0] - span, center[0] + span + 1):
            for iy in range(center[1] - span, center[1] + span + 1):
                x_value = ix * node.occupancy_resolution_m
                y_value = iy * node.occupancy_resolution_m
                if math.hypot(x_value - obs.x, y_value - obs.y) <= obs.radius:
                    occupied.add((ix, iy, center[2]))
    return occupied


def _configure_node(module, condition):
    node = make_uninitialized_node(module)
    node.follow_distance_m = FOLLOW_DISTANCE_M
    node.occupancy_resolution_m = 0.35
    node.segment_sample_step_m = 0.45
    node.enable_obstacle_safety_filter = condition != "no_hard_filter"
    node.base_safety_margin_m = 0.28
    node.obstacle_margin_m = 0.18
    node.speed_margin_gain = 0.35 if condition != "fixed_safety_margin" else 0.0
    node.covariance_margin_gain = 1.0 if condition != "fixed_safety_margin" else 0.0
    node.failed_candidate_cooldown_sec = 2.5
    node.failed_candidate_radius_m = 0.7
    node.last_valid_target = _Target(0.0, 0.0)
    return node


def _prediction_for_step(module, condition, kf, measurement, t_sec, last_measurement):
    visible, meas_x, meas_y = measurement
    if visible:
        kf.update(t_sec, meas_x, meas_y)
        last_measurement = (meas_x, meas_y)

    if condition == "no_prediction":
        if visible:
            return [meas_x, meas_y, 0.0, 0.0], module.mat_eye(4, 0.05), last_measurement
        if last_measurement is not None:
            return [last_measurement[0], last_measurement[1], 0.0, 0.0], module.mat_eye(4, 0.9), last_measurement

    state, covariance = kf.predict(0.6)
    if state is None and last_measurement is not None:
        state = [last_measurement[0], last_measurement[1], 0.0, 0.0]
        covariance = module.mat_eye(4, 1.0)
    return state, covariance, last_measurement


def _select_candidate(node, module, condition, predicted, vehicle_xyz, vehicle_speed, covariance, occupied, now_sec):
    candidates = node._generate_follow_candidates(predicted, vehicle_xyz)
    if condition == "fixed_behind":
        candidates = [candidate for candidate in candidates if candidate.name == "behind"]
    elif condition == "side_only":
        candidates = [candidate for candidate in candidates if candidate.name in ("left", "right")]
    elif condition == "no_far_safe":
        candidates = [candidate for candidate in candidates if candidate.name != "far_safe"]

    safe, margin = node._filter_and_score_candidates(
        candidates,
        vehicle_xyz,
        vehicle_speed,
        covariance,
        occupied,
        now_sec,
    )
    if not safe:
        return None, margin, 0
    if condition == "nearest_feasible":
        chosen = min(safe, key=lambda item: _distance_xy((item.x, item.y), vehicle_xyz))
    else:
        chosen = safe[0]
    return chosen, margin, len(safe)


def _update_vehicle(vehicle_xyz, goal_xyz):
    dx = goal_xyz[0] - vehicle_xyz[0]
    dy = goal_xyz[1] - vehicle_xyz[1]
    dz = goal_xyz[2] - vehicle_xyz[2]
    step = (RESPONSE_GAIN * dx, RESPONSE_GAIN * dy, RESPONSE_GAIN * dz)
    max_step = MAX_UAV_SPEED_MPS * DT_SEC
    norm = math.sqrt(step[0] ** 2 + step[1] ** 2 + step[2] ** 2)
    if norm > max_step:
        step = tuple(value / norm * max_step for value in step)
    return vehicle_xyz[0] + step[0], vehicle_xyz[1] + step[1], vehicle_xyz[2] + step[2]


def _run_one(module, scenario, condition, seed):
    rng = random.Random(seed)
    node = _configure_node(module, condition)
    occupied = _occupied_cells_for(node, scenario.obstacles) if node.enable_obstacle_safety_filter else None
    kf = module.ConstantVelocityKalmanFilter(1.2, 0.09, 1.0)
    last_measurement = None
    vehicle = (-FOLLOW_DISTANCE_M, -0.3, UAV_Z)
    last_vehicle = vehicle
    last_candidate = ""
    goal_points = []
    distance_errors = []
    view_errors = []
    clearances = []
    safety_margins = []
    safe_candidate_counts = []
    planner_failures = []
    state_names = []
    candidate_names = []
    decision_latencies_ms = []
    command_step_changes = []
    goal_accel_changes = []
    previous_goal = None
    previous_goal_delta = None
    target_loss_active = False
    loss_start = None
    recovery_times = []

    for idx in range(int(RUN_DURATION_SEC / DT_SEC) + 1):
        t_sec = idx * DT_SEC
        target_x, target_y = _target_xy(scenario.name, t_sec)
        visible = not _inside_any_window(t_sec, scenario.loss_windows)
        noise_x = rng.gauss(0.0, 0.04)
        noise_y = rng.gauss(0.0, 0.04)
        measurement = (visible, target_x + noise_x, target_y + noise_y)
        node.last_valid_target = _Target(target_x, target_y)

        if not visible and not target_loss_active:
            target_loss_active = True
            loss_start = t_sec
        if visible and target_loss_active:
            recovery_times.append(t_sec - loss_start)
            target_loss_active = False
            loss_start = None

        predicted, covariance, last_measurement = _prediction_for_step(
            module,
            condition,
            kf,
            measurement,
            t_sec,
            last_measurement,
        )
        if predicted is None:
            selected = None
            margin = 0.0
            safe_count = 0
        else:
            vehicle_speed = _distance_xy(vehicle, last_vehicle) / DT_SEC if idx > 0 else 0.0
            start_decision = time.perf_counter()
            selected, margin, safe_count = _select_candidate(
                node,
                module,
                condition,
                predicted,
                vehicle,
                vehicle_speed,
                covariance,
                occupied,
                t_sec,
            )
            decision_latencies_ms.append((time.perf_counter() - start_decision) * 1000.0)

        if selected is None:
            state_name = "hold_safe"
            goal = vehicle
            selected_name = "hold"
        else:
            state_name = "follow" if visible else "predict_hold"
            goal = (selected.x, selected.y, selected.z)
            selected_name = selected.name

        segment_clearance = _segment_clearance(vehicle, goal, scenario.obstacles)
        planner_blocked = _planner_rejects_candidate(
            scenario,
            t_sec,
            segment_clearance,
            selected_name,
        )
        planner_failures.append(planner_blocked)
        if planner_blocked:
            if condition != "no_planner_feedback" and selected is not None:
                node.failed_candidates.append(
                    {"name": selected.name, "x": selected.x, "y": selected.y, "time_sec": t_sec}
                )
            next_vehicle = vehicle
        else:
            next_vehicle = _update_vehicle(vehicle, goal)

        distance_errors.append(abs(_distance_xy(vehicle, (target_x, target_y, TARGET_Z)) - FOLLOW_DISTANCE_M))
        target_velocity = (
            _target_xy(scenario.name, min(t_sec + DT_SEC, RUN_DURATION_SEC))[0] - target_x,
            _target_xy(scenario.name, min(t_sec + DT_SEC, RUN_DURATION_SEC))[1] - target_y,
        )
        view_vector = (target_x - vehicle[0], target_y - vehicle[1])
        if math.hypot(*target_velocity) > 1e-6 and math.hypot(*view_vector) > 1e-6:
            preferred = (-target_velocity[0], -target_velocity[1])
            preferred_norm = math.hypot(*preferred)
            actual_norm = math.hypot(*view_vector)
            dot = (preferred[0] / preferred_norm) * (view_vector[0] / actual_norm) + (
                preferred[1] / preferred_norm
            ) * (view_vector[1] / actual_norm)
            view_errors.append(math.acos(max(-1.0, min(1.0, dot))))
        clearances.append(_min_obstacle_clearance(vehicle, scenario.obstacles))
        safety_margins.append(margin)
        safe_candidate_counts.append(safe_count)
        state_names.append(state_name)
        candidate_names.append(selected_name)
        goal_points.append(goal)
        if previous_goal is not None:
            delta = _distance_xy(goal, previous_goal)
            command_step_changes.append(delta)
            if previous_goal_delta is not None:
                goal_accel_changes.append(abs(delta - previous_goal_delta))
            previous_goal_delta = delta
        previous_goal = goal

        last_vehicle = vehicle
        vehicle = next_vehicle
        node.last_candidate_name = selected_name
        last_candidate = selected_name

    candidate_switch_count = sum(
        1 for left, right in zip(candidate_names[1:], candidate_names[:-1]) if left != right
    )
    state_switch_count = sum(1 for left, right in zip(state_names[1:], state_names[:-1]) if left != right)
    planner_failure_count = sum(1 for item in planner_failures if item)
    planner_failure_burst = _max_true_burst(planner_failures)
    return {
        "scenario": scenario.name,
        "condition": condition,
        "seed": seed,
        "mean_distance_error": _mean(distance_errors),
        "mean_view_angle_error": _mean(view_errors),
        "min_obstacle_clearance": min(clearances),
        "safety_shell_violations": sum(1 for value in clearances if value < SAFETY_SHELL_M),
        "mean_safety_margin": _mean(safety_margins),
        "safe_candidate_ratio": _mean([1.0 if count > 0 else 0.0 for count in safe_candidate_counts]),
        "target_loss_recovery_time": _mean(recovery_times) if recovery_times else 0.0,
        "planner_failure_count": planner_failure_count,
        "planner_failure_burst_max": planner_failure_burst,
        "candidate_switch_count": candidate_switch_count,
        "state_switch_count": state_switch_count,
        "goal_update_latency_ms": _mean(decision_latencies_ms),
        "ego_command_continuity": _mean(command_step_changes) if command_step_changes else 0.0,
        "trajectory_smoothness": _mean(goal_accel_changes) if goal_accel_changes else 0.0,
        "task_success": int(
            min(clearances) >= SAFETY_SHELL_M
            and planner_failure_count <= 4
            and (not recovery_times or _mean(recovery_times) <= 2.0)
        ),
    }


def _mean(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return 0.0
    return sum(finite) / len(finite)


def _planner_rejects_candidate(scenario, t_sec, segment_clearance, selected_name):
    if not _inside_any_window(t_sec, scenario.planner_block_windows):
        return False
    if scenario.name == "planner_blocked":
        return selected_name in {"behind", "far_safe"} or segment_clearance < 1.05
    return segment_clearance < 1.05


def _std(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if len(finite) <= 1:
        return 0.0
    return statistics.stdev(finite)


def _max_true_burst(values):
    best = 0
    current = 0
    for value in values:
        if value:
            current += 1
            best = max(best, current)
        else:
            current = 0
    return best


def _summarize(rows, keys):
    metric_names = [
        "mean_distance_error",
        "mean_view_angle_error",
        "min_obstacle_clearance",
        "safety_shell_violations",
        "safe_candidate_ratio",
        "target_loss_recovery_time",
        "planner_failure_count",
        "planner_failure_burst_max",
        "candidate_switch_count",
        "goal_update_latency_ms",
        "ego_command_continuity",
        "trajectory_smoothness",
        "task_success",
    ]
    groups = {}
    for row in rows:
        key = tuple(row[item] for item in keys)
        groups.setdefault(key, []).append(row)
    out = []
    for key, group_rows in sorted(groups.items()):
        summary = {keys[idx]: key[idx] for idx in range(len(keys))}
        summary["n"] = len(group_rows)
        for metric in metric_names:
            values = [row[metric] for row in group_rows]
            summary[metric + "_mean"] = _mean(values)
            summary[metric + "_std"] = _std(values)
        out.append(summary)
    return out


def _write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _write_markdown(path, by_condition, manifest):
    ordered = sorted(
        by_condition,
        key=lambda row: (
            -row["task_success_mean"],
            row["safety_shell_violations_mean"],
            row["planner_failure_count_mean"],
            row["mean_distance_error_mean"],
        ),
    )
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# Stage2 Adapter Offline Experiment Summary\n\n")
        handle.write("This is a diagnostic software-only experiment. It is not hardware validation.\n\n")
        handle.write("## Manifest\n\n")
        for key in ["created_at", "seeds", "scenarios", "conditions"]:
            handle.write("- `%s`: %s\n" % (key, manifest[key]))
        handle.write("\n## By Condition\n\n")
        handle.write(
            "| condition | n | success | safety violations | planner failures | distance error | view error | latency ms |\n"
        )
        handle.write("|---|---:|---:|---:|---:|---:|---:|---:|\n")
        for row in ordered:
            handle.write(
                "| {condition} | {n} | {success:.3f} | {viol:.3f} | {pf:.3f} | {dist:.3f} | {view:.3f} | {lat:.3f} |\n".format(
                    condition=row["condition"],
                    n=row["n"],
                    success=row["task_success_mean"],
                    viol=row["safety_shell_violations_mean"],
                    pf=row["planner_failure_count_mean"],
                    dist=row["mean_distance_error_mean"],
                    view=row["mean_view_angle_error_mean"],
                    lat=row["goal_update_latency_ms_mean"],
                )
            )


def _parse_seeds(raw):
    seeds = []
    for chunk in raw.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ":" in chunk:
            start, end = [int(item) for item in chunk.split(":", 1)]
            seeds.extend(range(start, end + 1))
        else:
            seeds.append(int(chunk))
    return sorted(set(seeds))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", default="1:5", help="Comma/range list, for example 1:5 or 1,2,7")
    parser.add_argument("--output-dir", default="", help="Output directory; default is timestamped research/runs path")
    args = parser.parse_args()

    install_stubs()
    module = load_adapter()
    seeds = _parse_seeds(args.seeds)
    scenarios = _make_scenarios()
    conditions = _conditions()
    rows = []
    for scenario in scenarios:
        for condition in conditions:
            for seed in seeds:
                rows.append(_run_one(module, scenario, condition, seed))

    repo_root = Path(__file__).resolve().parents[2]
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = repo_root / "research" / "runs" / "stage2_adapter_experiments" / stamp
    output_dir.mkdir(parents=True, exist_ok=True)

    by_condition = _summarize(rows, ["condition"])
    by_scenario_condition = _summarize(rows, ["scenario", "condition"])
    manifest = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "seeds": seeds,
        "scenarios": [scenario.name for scenario in scenarios],
        "conditions": conditions,
        "dt_sec": DT_SEC,
        "duration_sec": RUN_DURATION_SEC,
        "notes": "Software-only diagnostic run using the paper-line ROS1 adapter decision core.",
    }

    _write_csv(output_dir / "runs.csv", rows)
    _write_csv(output_dir / "by_condition.csv", by_condition)
    _write_csv(output_dir / "by_scenario_condition.csv", by_scenario_condition)
    with (output_dir / "manifest.json").open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
    _write_markdown(output_dir / "summary.md", by_condition, manifest)

    best = max(by_condition, key=lambda row: (row["task_success_mean"], -row["safety_shell_violations_mean"]))
    print(
        "stage2 adapter experiments PASS runs=%d output=%s best=%s success=%.3f"
        % (len(rows), output_dir, best["condition"], best["task_success_mean"])
    )


if __name__ == "__main__":
    main()
