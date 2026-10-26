#!/usr/bin/env python3
"""Analyze Stage-4 raw RM75 cycles without inventing physical evidence.

Only CSV rows labelled real_robot and base_link_to_Link7 are accepted. Delay is
estimated from trajectory response alignment; feedback receipt age is reported
separately and is never presented as controller delay.
"""

import argparse
import csv
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

SCENARIOS = ("x", "y", "z", "rx", "ry", "rz", "6d")
JOINTS = range(1, 8)


def percentile(values, probability):
    if not values:
        return None
    ordered = sorted(values)
    index = probability * (len(ordered) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    fraction = index - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def stats(values):
    values = list(values)
    if not values:
        return {key: None for key in
                ("samples", "mean", "rms", "max", "p95", "p99", "stddev")}
    if not all(math.isfinite(value) for value in values):
        raise ValueError("metric contains a non-finite value")
    mean = statistics.fmean(values)
    return {
        "samples": len(values),
        "mean": mean,
        "rms": math.sqrt(statistics.fmean(value * value for value in values)),
        "max": max(values),
        "p95": percentile(values, .95),
        "p99": percentile(values, .99),
        "stddev": math.sqrt(statistics.fmean((value - mean) ** 2
                                              for value in values)),
    }


def number(row, field):
    try:
        value = float(row[field])
    except (KeyError, ValueError) as exc:
        raise ValueError(f"invalid {field!r} in run {row.get('run_id', '?')}") from exc
    if not math.isfinite(value):
        raise ValueError(f"non-finite {field!r} in run {row.get('run_id', '?')}")
    return value


def vector(row, prefix):
    return [number(row, f"{prefix}{joint}") for joint in JOINTS]


def pose(row, prefix):
    return ([number(row, f"{prefix}_{axis}_m") for axis in "xyz"],
            [number(row, f"{prefix}_q{axis}") for axis in "xyzw"])


def quaternion_error(a, b):
    an = math.sqrt(sum(value * value for value in a))
    bn = math.sqrt(sum(value * value for value in b))
    if an < 1e-12 or bn < 1e-12:
        raise ValueError("zero quaternion in raw data")
    dot = abs(sum(x * y for x, y in zip(a, b)) / (an * bn))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def pose_error(a, b):
    return (math.sqrt(sum((x - y) ** 2 for x, y in zip(a[0], b[0]))),
            quaternion_error(a[1], b[1]))


def pearson(a, b):
    if len(a) != len(b) or len(a) < 4:
        return None
    am, bm = statistics.fmean(a), statistics.fmean(b)
    ac = [value - am for value in a]
    bc = [value - bm for value in b]
    denominator = math.sqrt(sum(value * value for value in ac) *
                            sum(value * value for value in bc))
    return None if denominator < 1e-15 else sum(x * y for x, y in zip(ac, bc)) / denominator


def interpolate(times, values, target):
    if target < times[0] or target > times[-1]:
        return None
    low, high = 0, len(times) - 1
    while high - low > 1:
        middle = (low + high) // 2
        if times[middle] <= target:
            low = middle
        else:
            high = middle
    if times[high] == times[low]:
        return values[low]
    ratio = (target - times[low]) / (times[high] - times[low])
    return values[low] + ratio * (values[high] - values[low])


def speed_profile(times, vectors):
    output_times, speeds = [], []
    for index in range(1, len(times)):
        dt = times[index] - times[index - 1]
        if dt <= 1e-6:
            continue
        output_times.append((times[index] + times[index - 1]) * .5)
        speeds.append(math.sqrt(sum((a - b) ** 2 for a, b in
                                    zip(vectors[index], vectors[index - 1]))) / dt)
    return output_times, speeds


def sampled_joint_dynamics(rows):
    times = [number(row, "feedback_time_sec") for row in rows]
    joints = [vector(row, "actual_q") for row in rows]
    velocity_times, velocities, velocity_max = [], [], []
    for index in range(1, len(rows)):
        dt = times[index] - times[index - 1]
        if dt <= 1e-6:
            continue
        value = [(a - b) / dt for a, b in
                 zip(joints[index], joints[index - 1])]
        velocity_times.append((times[index] + times[index - 1]) * .5)
        velocities.append(value)
        velocity_max.append(max(abs(item) for item in value))
    acceleration_max = []
    for index in range(1, len(velocities)):
        dt = velocity_times[index] - velocity_times[index - 1]
        if dt <= 1e-6:
            continue
        value = [(a - b) / dt for a, b in
                 zip(velocities[index], velocities[index - 1])]
        acceleration_max.append(max(abs(item) for item in value))
    return {
        "max_joint_velocity_rad_s": stats(velocity_max),
        "max_joint_acceleration_rad_s2": stats(acceleration_max),
        "estimator":
            "finite difference at recorder sample rate; encoder quantization sensitive",
    }


def estimate_delay(rows):
    """Return response delay models, not timestamp subtraction."""
    published = [row for row in rows if row.get("command_published") == "1"]
    if len(published) < 8:
        return {"cross_correlation_sec": None, "peak_alignment_sec": None,
                "correlation": None, "method": "insufficient_motion_samples"}
    command_times = [number(row, "command_time_sec") for row in published]
    feedback_times = [number(row, "feedback_time_sec") for row in published]
    command_q = [vector(row, "target_q") for row in published]
    actual_q = [vector(row, "actual_q") for row in published]
    ct, cs = speed_profile(command_times, command_q)
    ft, fs = speed_profile(feedback_times, actual_q)
    if len(ct) < 4 or len(ft) < 4 or max(cs, default=0.0) < 1e-8 or max(fs, default=0.0) < 1e-8:
        return {"cross_correlation_sec": None, "peak_alignment_sec": None,
                "correlation": None, "method": "insufficient_excitation"}
    periods = [ct[index] - ct[index - 1] for index in range(1, len(ct))]
    step = min(.005, max(.001, statistics.median(periods) / 4.0))
    best_delay, best_correlation = None, -2.0
    candidates = int(.5 / step) + 1
    for candidate in range(candidates):
        delay = candidate * step
        expected, observed = [], []
        for time, measured_speed in zip(ft, fs):
            commanded_speed = interpolate(ct, cs, time - delay)
            if commanded_speed is not None:
                expected.append(commanded_speed)
                observed.append(measured_speed)
        correlation = pearson(expected, observed)
        if correlation is not None and correlation > best_correlation:
            best_delay, best_correlation = delay, correlation
    peak_delay = ft[max(range(len(fs)), key=fs.__getitem__)] - \
        ct[max(range(len(cs)), key=cs.__getitem__)]
    return {
        "cross_correlation_sec": best_delay,
        "peak_alignment_sec": peak_delay if peak_delay >= 0.0 else None,
        "correlation": best_correlation if best_delay is not None else None,
        "method": "joint_speed_cross_correlation_and_peak_alignment",
    }


def validate_and_group(paths):
    required = {"data_source", "reference_frame", "run_id", "scenario",
                "repetition", "sample_time_sec", "command_time_sec",
                "feedback_time_sec", "feedback_source_time_sec",
                "feedback_source_time_valid", "command_published", "ideal_x_m",
                "command_x_m", "actual_x_m", "guard_result", "collision",
                "stop_reason", "force_n", "torque_nm",
                "requested_linear_speed_mps", "requested_angular_speed_rad_s"}
    grouped = defaultdict(list)
    for path in paths:
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            missing = required - set(reader.fieldnames or ())
            if missing:
                raise ValueError(f"{path}: missing columns {sorted(missing)}")
            for row in reader:
                if row["data_source"] != "real_robot" or \
                        row["reference_frame"] != "base_link_to_Link7":
                    raise ValueError(f"{path}: non-real or non-Link7 row rejected")
                if row["scenario"] not in SCENARIOS:
                    raise ValueError(f"{path}: unknown scenario {row['scenario']!r}")
                grouped[row["run_id"]].append(row)
    if not grouped:
        raise ValueError("no real-robot rows found")
    seen_repetitions = set()
    for run_id, rows in grouped.items():
        scenario = {row["scenario"] for row in rows}
        repetition = {int(row["repetition"]) for row in rows}
        if len(scenario) != 1 or len(repetition) != 1 or min(repetition) < 1:
            raise ValueError(f"run {run_id}: inconsistent metadata")
        identity = (next(iter(scenario)), next(iter(repetition)))
        if identity in seen_repetitions:
            raise ValueError(f"duplicate scenario/repetition {identity}")
        seen_repetitions.add(identity)
        rows.sort(key=lambda row: number(row, "sample_time_sec"))
        for field in ("sample_time_sec", "command_time_sec", "feedback_time_sec"):
            values = [number(row, field) for row in rows]
            if any(b < a for a, b in zip(values, values[1:])):
                raise ValueError(f"run {run_id}: non-monotonic {field}")
        source_rows = []
        for row in rows:
            if row["feedback_source_time_valid"] not in ("0", "1"):
                raise ValueError(
                    f"run {run_id}: invalid feedback_source_time_valid")
            if row["feedback_source_time_valid"] == "1":
                source_rows.append(number(row, "feedback_source_time_sec"))
        if any(stamp <= 0.0 for stamp in source_rows):
            raise ValueError(f"run {run_id}: invalid feedback source timestamp")
        if any(b < a for a, b in zip(source_rows, source_rows[1:])):
            raise ValueError(f"run {run_id}: non-monotonic feedback source timestamp")
        if any(number(row, "command_time_sec") > number(row, "sample_time_sec") + 1e-6 or
               number(row, "feedback_time_sec") > number(row, "sample_time_sec") + 1e-6
               for row in rows):
            raise ValueError(f"run {run_id}: future timestamp")
    return grouped


def direction_for(rows):
    scenario = rows[0]["scenario"]
    if scenario == "6d":
        return "combined"
    ideal_start, ideal_end = pose(rows[0], "ideal"), pose(rows[-1], "ideal")
    if scenario in ("x", "y", "z"):
        component = "xyz".index(scenario)
        delta = ideal_end[0][component] - ideal_start[0][component]
    else:
        # The exact Euler decomposition is unnecessary for sign balance: use
        # the vector part of q_end * conjugate(q_start), with q stored xyzw.
        axis = {"rx": 0, "ry": 1, "rz": 2}[scenario]
        start, end = ideal_start[1], ideal_end[1]
        sign = 1.0 if sum(x * y for x, y in zip(start, end)) >= 0 else -1.0
        end = [sign * value for value in end]
        w0, w1 = start[3], end[3]
        v0, v1 = start[:3], end[:3]
        delta = w0 * v1[axis] - w1 * v0[axis]
        cross = (v0[(axis + 1) % 3] * v1[(axis + 2) % 3] -
                 v0[(axis + 2) % 3] * v1[(axis + 1) % 3])
        delta -= cross
    return "+" if delta >= 0.0 else "-"


def analyze_run(run_id, rows):
    ik_position, ik_rotation = [], []
    discretization_position, discretization_rotation = [], []
    driver_position, driver_rotation = [], []
    total_position, total_rotation, joint_errors = [], [], [[] for _ in JOINTS]
    feedback_ages, intervals, source_receipt_offsets, forces, torques = [], [], [], [], []
    for index, row in enumerate(rows):
        ideal, command, actual = pose(row, "ideal"), pose(row, "command"), pose(row, "actual")
        ep, er = pose_error(ideal, command)
        ik_position.append(ep)  # solver/FK residual at the sampled joint target
        ik_rotation.append(er)
        if index:
            # Zero-order-hold distance from the continuous ideal path is the
            # recorded trajectory discretization contribution.
            previous_command = pose(rows[index - 1], "command")
            dp, dr = pose_error(ideal, previous_command)
        else:
            dp, dr = ep, er
        discretization_position.append(dp)
        discretization_rotation.append(dr)
        ep, er = pose_error(command, actual)
        driver_position.append(ep)
        driver_rotation.append(er)
        ep, er = pose_error(ideal, actual)
        total_position.append(ep)
        total_rotation.append(er)
        for joint, (target, actual_q) in enumerate(zip(vector(row, "target_q"),
                                                       vector(row, "actual_q"))):
            joint_errors[joint].append(target - actual_q)
        feedback_ages.append(number(row, "sample_time_sec") -
                             number(row, "feedback_time_sec"))
        if row["feedback_source_time_valid"] == "1":
            source_receipt_offsets.append(
                number(row, "feedback_time_sec") -
                number(row, "feedback_source_time_sec"))
        if index:
            intervals.append(number(row, "feedback_time_sec") -
                             number(rows[index - 1], "feedback_time_sec"))
        forces.append(number(row, "force_n"))
        torques.append(number(row, "torque_nm"))
    stopped = any(row["stop_reason"] for row in rows)
    delay = estimate_delay(rows)
    dynamics = sampled_joint_dynamics(rows)
    published_times = [number(row, "command_time_sec") for row in rows
                       if row["command_published"] == "1"]
    command_intervals = [b - a for a, b in
                         zip(published_times, published_times[1:]) if b > a]
    stop_rows = [row for row in rows if row["stop_reason"]]
    stop_latency, stop_distance = None, None
    if stop_rows:
        request_time = number(stop_rows[0], "sample_time_sec")
        start_q = vector(stop_rows[0], "actual_q")
        stop_distance = max(max(abs(value - origin) for value, origin in
                                zip(vector(row, "actual_q"), start_q))
                            for row in stop_rows)
        last_motion_time = request_time
        previous = start_q
        for row in stop_rows[1:]:
            current = vector(row, "actual_q")
            if max(abs(value - old) for value, old in zip(current, previous)) > 1e-5:
                last_motion_time = number(row, "sample_time_sec")
            previous = current
        stop_latency = last_motion_time - request_time
    return {
        "run_id": run_id, "scenario": rows[0]["scenario"],
        "repetition": int(rows[0]["repetition"]), "direction": direction_for(rows),
        "status": "STOP" if stopped else "PASS", "samples": len(rows),
        "position_error_m": stats(total_position),
        "orientation_error_rad": stats(total_rotation),
        "joint_error_rad": [stats(values) for values in joint_errors],
        "error_decomposition": {
            "ik_position_m": stats(ik_position), "ik_orientation_rad": stats(ik_rotation),
            "trajectory_discretization_position_m": stats(discretization_position),
            "trajectory_discretization_orientation_rad": stats(discretization_rotation),
            "driver_tracking_position_m": stats(driver_position),
            "driver_tracking_orientation_rad": stats(driver_rotation),
        },
        "delay": delay, "feedback_age_sec": stats(feedback_ages),
        "command_interval_sec": stats(command_intervals),
        "sampled_joint_dynamics": dynamics,
        "timestamp_sync": {
            "source_stamp_samples": len(source_receipt_offsets),
            "source_stamp_coverage": len(source_receipt_offsets) / len(rows),
            "source_to_receipt_offset_sec": stats(source_receipt_offsets),
            "interpretation": "clock/transport diagnostic; not controller delay",
        },
        "feedback_interval_sec": stats(intervals), "force_n": stats(forces),
        "torque_nm": stats(torques),
        "collision_samples": sum(row["collision"] == "1" for row in rows),
        "stop_latency_sec": stop_latency,
        "stop_distance_max_joint_rad": stop_distance,
        "stop_reason": next((row["stop_reason"] for row in rows
                             if row["stop_reason"]), ""),
        "requested_linear_speed_mps": number(rows[0], "requested_linear_speed_mps"),
        "requested_angular_speed_rad_s": number(rows[0], "requested_angular_speed_rad_s"),
    }


def aggregate_runs(runs):
    scenarios = []
    for scenario in SCENARIOS:
        selected = [run for run in runs if run["scenario"] == scenario]
        directions = sorted({run["direction"] for run in selected})
        direction_ok = scenario == "6d" or directions == ["+", "-"]
        scenarios.append({
            "scenario": scenario, "attempted_runs": len(selected),
            "passed_runs": sum(run["status"] == "PASS" for run in selected),
            "stopped_runs": sum(run["status"] == "STOP" for run in selected),
            "directions": directions,
            "requirement_met": len(selected) >= 100 and direction_ok,
            "position_error_m": stats(run["position_error_m"]["rms"] for run in selected),
            "orientation_error_rad": stats(run["orientation_error_rad"]["rms"] for run in selected),
            "delay_cross_correlation_sec": stats(
                run["delay"]["cross_correlation_sec"] for run in selected
                if run["delay"]["cross_correlation_sec"] is not None),
        })
    return scenarios


def guard_recommendation(grouped, runs, complete):
    normal_ids = {run["run_id"] for run in runs if run["status"] == "PASS"}
    tracking, intervals, forces, torques = [], [], [], []
    for run_id in normal_ids:
        rows = grouped[run_id]
        previous_feedback = None
        for row in rows:
            tracking.append(max(abs(a - b) for a, b in
                                zip(vector(row, "target_q"), vector(row, "actual_q"))))
            stamp = number(row, "feedback_time_sec")
            if previous_feedback is not None and stamp > previous_feedback:
                intervals.append(stamp - previous_feedback)
            previous_feedback = stamp
            forces.append(number(row, "force_n"))
            torques.append(number(row, "torque_nm"))
    def limits(values):
        summary = stats(values)
        if not values:
            return None, None, summary
        warning = max(summary["p99"], summary["mean"] + 3.0 * summary["stddev"])
        stop = max(summary["max"] * 1.5,
                   summary["mean"] + 6.0 * summary["stddev"])
        return warning, stop, summary
    track_warning, track_stop, track_stats = limits(tracking)
    force_warning, force_stop, force_stats = limits(forces)
    torque_warning, torque_stop, torque_stats = limits(torques)
    interval_stats = stats(intervals)
    timeout = None if not intervals else max(interval_stats["p99"] * 2.0,
                                              interval_stats["max"] * 1.5)
    pass_coverage = {}
    for scenario in SCENARIOS:
        selected = [run for run in runs
                    if run["scenario"] == scenario and run["status"] == "PASS"]
        directions = sorted({run["direction"] for run in selected})
        direction_ok = scenario == "6d" or directions == ["+", "-"]
        pass_coverage[scenario] = {
            "pass_runs": len(selected), "directions": directions,
            "requirement_met": len(selected) >= 100 and direction_ok,
        }
    normal_coverage_complete = all(
        item["requirement_met"] for item in pass_coverage.values())
    eligible = complete and normal_coverage_complete
    return {
        "status": "eligible" if eligible else "provisional_do_not_deploy",
        "normal_pass_runs": len(normal_ids),
        "normal_coverage_complete": normal_coverage_complete,
        "normal_coverage_rule":
            "100 PASS runs per scenario; +/- represented for each single-axis scenario",
        "normal_pass_coverage": pass_coverage,
        "method": "normal PASS samples; warning=max(P99,mean+3sigma); stop=max(1.5*observed_max,mean+6sigma)",
        "max_tracking_error_warning_rad": track_warning,
        "max_tracking_error_stop_rad": track_stop,
        "feedback_timeout_stop_sec": timeout,
        "force_warning_n": force_warning, "force_stop_n": force_stop,
        "torque_warning_nm": torque_warning, "torque_stop_nm": torque_stop,
        "evidence": {"tracking": track_stats, "feedback_interval": interval_stats,
                     "force": force_stats, "torque": torque_stats},
    }


def fmt(value, scale=1.0, digits=4):
    return "N/A" if value is None else f"{value * scale:.{digits}g}"


def write_summary_csv(path, scenarios):
    fields = ("scenario", "attempted_runs", "passed_runs", "stopped_runs",
              "directions", "requirement_met", "position_rms_mean_m",
              "position_rms_p95_m", "orientation_rms_mean_rad",
              "orientation_rms_p95_rad", "delay_mean_sec", "delay_p95_sec",
              "delay_p99_sec")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in scenarios:
            writer.writerow({
                "scenario": item["scenario"], "attempted_runs": item["attempted_runs"],
                "passed_runs": item["passed_runs"], "stopped_runs": item["stopped_runs"],
                "directions": "".join(item["directions"]),
                "requirement_met": item["requirement_met"],
                "position_rms_mean_m": item["position_error_m"]["mean"],
                "position_rms_p95_m": item["position_error_m"]["p95"],
                "orientation_rms_mean_rad": item["orientation_error_rad"]["mean"],
                "orientation_rms_p95_rad": item["orientation_error_rad"]["p95"],
                "delay_mean_sec": item["delay_cross_correlation_sec"]["mean"],
                "delay_p95_sec": item["delay_cross_correlation_sec"]["p95"],
                "delay_p99_sec": item["delay_cross_correlation_sec"]["p99"],
            })


def write_report(path, sources, payload):
    lines = [
        "# RM75-6FB real robot benchmark report", "",
        f"Verdict: **{'COMPLETE' if payload['complete'] else 'INCOMPLETE'}**.", "",
        "This report contains only supervised `real_robot` records. Missing experiments are shown as N/A; no physical result is inferred from simulation or synthetic tests.", "",
        "## Robot configuration", "",
        "- Robot: RM75-6FB", "- ROS: ROS1 Noetic",
        "- Endpoint: `base_link -> Link7` (no TCP)",
        "- Nominal command period: 20 ms / 50 Hz", "",
        "## Accuracy and timing", "",
        "| Scenario | Runs | PASS | STOP | Directions | Position RMS mean (mm) | Position RMS P95 (mm) | Orientation RMS mean (deg) | Delay mean/P95/P99 (ms) |",
        "|---|---:|---:|---:|---|---:|---:|---:|---:|",
    ]
    for item in payload["scenarios"]:
        delay = item["delay_cross_correlation_sec"]
        lines.append(
            f"| {item['scenario']} | {item['attempted_runs']} | {item['passed_runs']} | "
            f"{item['stopped_runs']} | {''.join(item['directions']) or 'N/A'} | "
            f"{fmt(item['position_error_m']['mean'], 1000)} | "
            f"{fmt(item['position_error_m']['p95'], 1000)} | "
            f"{fmt(item['orientation_error_rad']['mean'], 180 / math.pi)} | "
            f"{fmt(delay['mean'], 1000)}/{fmt(delay['p95'], 1000)}/{fmt(delay['p99'], 1000)} |")
    calibration = payload["guard_recommendation"]
    stop = payload["stop_performance"]
    clock = payload["timestamp_sync"]["source_to_receipt_offset_sec"]
    robot_state = payload["robot_state"]
    timing = payload["timing"]
    lines += [
        "", "Delay is identified using joint-speed cross-correlation and independently checked by peak alignment. Sample receipt age is retained separately and is not called control delay.",
        f"ROS source-stamp to local-receipt offset mean/P95 (ms): {fmt(clock['mean'], 1000)} / {fmt(clock['p95'], 1000)}. This is a clock/transport diagnostic, not controller delay.",
        f"Command frequency mean/P95 (Hz): {fmt(timing['command_frequency_hz']['mean'])} / {fmt(timing['command_frequency_hz']['p95'])}.",
        f"Sampled maximum joint velocity/acceleration (rad/s, rad/s^2): {fmt(robot_state['max_joint_velocity_rad_s']['max'])} / {fmt(robot_state['max_joint_acceleration_rad_s2']['max'])}.",
        "Joint velocity/acceleration use finite differences at the recorder sample rate and are sensitive to encoder quantization; acceleration is not a controller-internal measurement.",
        "", "## Error model", "",
        "Each run stores three distinct terms: continuous ideal pose → FK(command joint) (IK residual), ideal pose → previous held command (trajectory discretization), and FK(command joint) → FK(feedback joint) (driver tracking). They are not algebraically merged into one cause.",
        "", "Velocity-stratified 0.01/0.03 m/s results are stored under `velocity_profile` in the JSON; a speed is absent unless supervised runs at that requested speed exist.",
        "", "## Safety", "",
        f"- Guard calibration: **{calibration['status']}**",
        f"- Normal PASS coverage complete: {calibration['normal_coverage_complete']} ({calibration['normal_coverage_rule']})",
        f"- Tracking warning/stop recommendation (rad): {fmt(calibration['max_tracking_error_warning_rad'])} / {fmt(calibration['max_tracking_error_stop_rad'])}",
        f"- Feedback timeout recommendation (s): {fmt(calibration['feedback_timeout_stop_sec'])}",
        f"- Force warning/stop recommendation (N): {fmt(calibration['force_warning_n'])} / {fmt(calibration['force_stop_n'])}",
        f"- Torque warning/stop recommendation (N m): {fmt(calibration['torque_warning_nm'])} / {fmt(calibration['torque_stop_nm'])}",
        f"- STOP runs: {stop['stop_runs']}; latency mean/P95 (s): {fmt(stop['stop_latency_sec']['mean'])} / {fmt(stop['stop_latency_sec']['p95'])}",
        f"- STOP distance mean/max (rad): {fmt(stop['stop_distance_max_joint_rad']['mean'])} / {fmt(stop['stop_distance_max_joint_rad']['max'])}",
        f"- STOP/failure reasons: {', '.join(stop['failure_reasons']) or 'N/A'}",
        "- Collision and STOP details remain in each run entry in the JSON.",
        "", "## Limitations", "",
        "- Link7 is the current endpoint; there is no calibrated TCP in this benchmark.",
        "- Only low-speed, manually supervised motion is permitted.",
        "- Touch teleoperation and force control are disabled.",
        "- No contact, soil insertion, or unattended repetition is permitted.",
        "- A recommendation marked provisional must not replace the deployed Guard configuration.",
        "", "## Sources", "",
    ] + [f"- `{source}`" for source in sources]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_yaml(path, recommendation):
    def yaml_value(value):
        return "null" if value is None else f"{value:.12g}"
    path.write_text(
        "# Generated from supervised real-robot data. Do not deploy unless status=eligible.\n"
        f"calibration_status: {recommendation['status']}\n"
        f"normal_pass_runs: {recommendation['normal_pass_runs']}\n"
        f"normal_coverage_complete: {'true' if recommendation['normal_coverage_complete'] else 'false'}\n"
        f"max_tracking_error_warning_rad: {yaml_value(recommendation['max_tracking_error_warning_rad'])}\n"
        f"max_tracking_error_stop_rad: {yaml_value(recommendation['max_tracking_error_stop_rad'])}\n"
        f"feedback_timeout_stop_sec: {yaml_value(recommendation['feedback_timeout_stop_sec'])}\n"
        f"force_warning_n: {yaml_value(recommendation['force_warning_n'])}\n"
        f"force_stop_n: {yaml_value(recommendation['force_stop_n'])}\n"
        f"torque_warning_nm: {yaml_value(recommendation['torque_warning_nm'])}\n"
        f"torque_stop_nm: {yaml_value(recommendation['torque_stop_nm'])}\n",
        encoding="utf-8")


def write_svg(path, scenarios):
    width, height, margin = 980, 470, 70
    values = [item["position_error_m"]["mean"] or 0.0 for item in scenarios]
    maximum = max(values, default=0.0) or 1.0
    spacing = (width - 2 * margin) / len(scenarios)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<text x="490" y="30" text-anchor="middle" font-family="sans-serif" font-size="18">RM75 Link7 position RMS by scenario</text>',
             f'<line x1="{margin}" y1="400" x2="{width-margin}" y2="400" stroke="black"/>']
    for index, (item, value) in enumerate(zip(scenarios, values)):
        x = margin + index * spacing + spacing * .2
        bar_width = spacing * .6
        bar_height = 300 * value / maximum
        parts += [f'<rect x="{x:.1f}" y="{400-bar_height:.1f}" width="{bar_width:.1f}" height="{bar_height:.1f}" fill="#2878b5"/>',
                  f'<text x="{x+bar_width/2:.1f}" y="420" text-anchor="middle" font-family="sans-serif" font-size="12">{item["scenario"]}</text>',
                  f'<text x="{x+bar_width/2:.1f}" y="{395-bar_height:.1f}" text-anchor="middle" font-family="sans-serif" font-size="10">{value*1000:.3g} mm</text>']
    parts.append('</svg>')
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def write_tracking_svg(path, grouped):
    width, height, margin = 980, 470, 70
    series = []
    for scenario in SCENARIOS:
        match = next(((run_id, rows) for run_id, rows in sorted(grouped.items())
                      if rows[0]["scenario"] == scenario), None)
        if match:
            rows = match[1]
            start = number(rows[0], "sample_time_sec")
            series.append((scenario,
                           [number(row, "sample_time_sec") - start for row in rows],
                           [pose_error(pose(row, "ideal"), pose(row, "actual"))[0] * 1000
                            for row in rows]))
    xmax = max((times[-1] for _, times, _ in series), default=1.0) or 1.0
    ymax = max((max(values, default=0.0) for _, _, values in series), default=1.0) or 1.0
    colors = ("#2878b5", "#d95f02", "#1b9e77", "#7570b3", "#e7298a", "#66a61e", "#e6ab02")
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<text x="490" y="28" text-anchor="middle" font-family="sans-serif" font-size="18">Link7 tracking error (first recorded run per scenario)</text>',
             f'<line x1="{margin}" y1="400" x2="{width-margin}" y2="400" stroke="black"/>',
             f'<line x1="{margin}" y1="60" x2="{margin}" y2="400" stroke="black"/>',
             '<text x="490" y="450" text-anchor="middle" font-family="sans-serif" font-size="12">elapsed time (s)</text>',
             '<text x="18" y="230" text-anchor="middle" transform="rotate(-90 18 230)" font-family="sans-serif" font-size="12">position error (mm)</text>']
    for index, (name, times, values) in enumerate(series):
        points = " ".join(f"{margin + value_t / xmax * (width - 2 * margin):.1f},"
                          f"{400 - value_y / ymax * 330:.1f}"
                          for value_t, value_y in zip(times, values))
        color = colors[index % len(colors)]
        parts += [f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.5"/>',
                  f'<text x="{margin + 80 * index}" y="50" font-family="sans-serif" font-size="11" fill="{color}">{name}</text>']
    parts.append('</svg>')
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def write_delay_svg(path, runs):
    values = [run["delay"]["cross_correlation_sec"] * 1000 for run in runs
              if run["delay"]["cross_correlation_sec"] is not None]
    width, height, margin, bins = 900, 440, 70, 10
    maximum = max(values, default=1.0)
    minimum = min(values, default=0.0)
    span = maximum - minimum
    counts = [0] * bins
    for value in values:
        index = 0 if span <= 1e-12 else min(bins - 1, int((value - minimum) / span * bins))
        counts[index] += 1
    peak = max(counts, default=1) or 1
    spacing = (width - 2 * margin) / bins
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
             '<rect width="100%" height="100%" fill="white"/>',
             '<text x="450" y="28" text-anchor="middle" font-family="sans-serif" font-size="18">Command-to-feedback response delay distribution</text>',
             f'<line x1="{margin}" y1="370" x2="{width-margin}" y2="370" stroke="black"/>',
             '<text x="450" y="420" text-anchor="middle" font-family="sans-serif" font-size="12">cross-correlation delay (ms)</text>']
    for index, count in enumerate(counts):
        x = margin + index * spacing + 1
        bar_height = 280 * count / peak
        label = minimum + (index + .5) * (span / bins if span > 1e-12 else 0.0)
        parts += [f'<rect x="{x:.1f}" y="{370-bar_height:.1f}" width="{spacing-2:.1f}" height="{bar_height:.1f}" fill="#6a51a3"/>',
                  f'<text x="{x+(spacing-2)/2:.1f}" y="390" text-anchor="middle" font-family="sans-serif" font-size="9">{label:.2g}</text>']
    if not values:
        parts.append('<text x="450" y="220" text-anchor="middle" font-family="sans-serif" font-size="16">N/A — insufficient motion data</text>')
    parts.append('</svg>')
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path, help="raw per-cycle CSV files")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--plots-dir", type=Path,
        help="optional plot directory (defaults to --output-dir)")
    args = parser.parse_args(argv)
    try:
        grouped = validate_and_group(args.inputs)
        runs = [analyze_run(run_id, rows) for run_id, rows in sorted(grouped.items())]
    except (OSError, ValueError) as error:
        print(f"analysis failed: {error}", file=sys.stderr)
        return 2
    scenarios = aggregate_runs(runs)
    complete = all(item["requirement_met"] for item in scenarios)
    recommendation = guard_recommendation(grouped, runs, complete)
    velocity_profile = {}
    for speed in sorted({run["requested_linear_speed_mps"] for run in runs
                         if run["scenario"] in ("x", "y", "z")}):
        selected = [run for run in runs if run["scenario"] in ("x", "y", "z")
                    and abs(run["requested_linear_speed_mps"] - speed) < 1e-9]
        velocity_profile[f"{speed:.6g}"] = {
            "runs": len(selected),
            "position_error_rms_m": stats(run["position_error_m"]["rms"]
                                           for run in selected),
            "driver_tracking_rms_m": stats(
                run["error_decomposition"]["driver_tracking_position_m"]["rms"]
                for run in selected),
        }
    stop_performance = {
        "stop_runs": sum(run["status"] == "STOP" for run in runs),
        "stop_latency_sec": stats(run["stop_latency_sec"] for run in runs
                                  if run["stop_latency_sec"] is not None),
        "stop_distance_max_joint_rad": stats(
            run["stop_distance_max_joint_rad"] for run in runs
            if run["stop_distance_max_joint_rad"] is not None),
        "failure_reasons": sorted({run["stop_reason"] for run in runs
                                   if run["stop_reason"]}),
    }
    robot_state = {
        "max_joint_velocity_rad_s": stats(
            run["sampled_joint_dynamics"]["max_joint_velocity_rad_s"]["max"]
            for run in runs),
        "max_joint_acceleration_rad_s2": stats(
            run["sampled_joint_dynamics"]["max_joint_acceleration_rad_s2"]["max"]
            for run in runs),
        "estimator":
            "finite difference at recorder sample rate; encoder quantization sensitive",
    }
    timing = {
        "command_frequency_hz": stats(
            1.0 / run["command_interval_sec"]["mean"] for run in runs
            if run["command_interval_sec"]["mean"]),
        "feedback_age_sec": stats(
            run["feedback_age_sec"]["mean"] for run in runs),
    }
    payload = {
        "data_source": "real_robot", "reference_frame": "base_link_to_Link7",
        "complete": complete, "completion_rule": "100 attempts per scenario; +/- represented for each single-axis scenario",
        "sources": [str(path) for path in args.inputs], "runs": runs,
        "scenarios": scenarios, "velocity_profile": velocity_profile,
        "stop_performance": stop_performance,
        "robot_state": robot_state, "timing": timing,
        "guard_recommendation": recommendation,
        "timestamp_sync": {
            "source_stamp_rows": sum(
                row["feedback_source_time_valid"] == "1"
                for rows in grouped.values() for row in rows),
            "source_to_receipt_offset_sec": stats(
                number(row, "feedback_time_sec") -
                number(row, "feedback_source_time_sec")
                for rows in grouped.values() for row in rows
                if row["feedback_source_time_valid"] == "1"),
            "interpretation":
                "ROS source stamp to local receipt diagnostic; not controller delay",
        },
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plots_dir = args.plots_dir or args.output_dir
    plots_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "real_robot_benchmark.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write_summary_csv(args.output_dir / "real_robot_benchmark.csv", scenarios)
    write_report(args.output_dir / "real_robot_benchmark_report.md", args.inputs, payload)
    write_yaml(args.output_dir / "runtime_guard.recommended.yaml", recommendation)
    write_svg(plots_dir / "real_robot_benchmark.svg", scenarios)
    write_tracking_svg(plots_dir / "tracking_error_curves.svg", grouped)
    write_delay_svg(plots_dir / "delay_distribution.svg", runs)
    return 0 if complete else 3


if __name__ == "__main__":
    raise SystemExit(main())
