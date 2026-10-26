#!/usr/bin/env python3
"""Append one supervised Cartesian executor log to the real benchmark CSV."""

import argparse
import csv
import datetime
import fcntl
from pathlib import Path

SCENARIOS = {
    "joint_small",
    "cartesian_line",
    "cartesian_orientation",
    "random_small_6d",
    "near_singular",
}

FIELDS = (
    "run_id", "utc_timestamp", "robot_serial", "scenario", "status",
    "reference_frame", "final_link7_position_error_m",
    "final_link7_orientation_error_rad", "max_link7_position_error_m",
    "rms_link7_position_error_m", "max_link7_orientation_error_rad",
    "rms_link7_orientation_error_rad", "max_joint_error_rad",
    "rms_joint_error_rad", "max_joint_change_rad",
    "actual_control_frequency_hz", "mean_command_period_ms",
    "max_command_period_ms", "actual_feedback_frequency_hz",
    "mean_feedback_latency_ms", "max_feedback_latency_ms", "dropped_frames",
    "max_velocity_rad_s", "max_acceleration_rad_s2", "fcl_collision_count",
    "peak_force_n", "peak_torque_nm", "guard_warn_count", "estop_count",
    "control_failure_count", "stop_time_sec",
    "stop_distance_max_joint_rad", "ik_dls_calls", "ik_refinements",
    "ik_fallback_calls", "notes",
)

KEYS = {
    "final_link7_position_error_m": "final_Link7_position_error_m",
    "final_link7_orientation_error_rad": "final_Link7_orientation_error_rad",
    "max_link7_position_error_m": "max_Link7_tracking_position_error_m",
    "rms_link7_position_error_m": "rms_Link7_tracking_position_error_m",
    "max_link7_orientation_error_rad": "max_Link7_tracking_orientation_error_rad",
    "rms_link7_orientation_error_rad": "rms_Link7_tracking_orientation_error_rad",
    "max_joint_error_rad": "max_joint_tracking_error_rad",
    "rms_joint_error_rad": "rms_joint_tracking_error_rad",
    "max_joint_change_rad": "max_joint_change_rad",
    "actual_control_frequency_hz": "actual_control_frequency_hz",
    "mean_command_period_ms": "mean_command_period_ms",
    "max_command_period_ms": "max_command_period_ms",
    "actual_feedback_frequency_hz": "actual_feedback_frequency_hz",
    "mean_feedback_latency_ms": "mean_feedback_age_ms",
    "max_feedback_latency_ms": "max_feedback_age_ms",
    "dropped_frames": "estimated_dropped_feedback_frames",
    "max_velocity_rad_s": "max_measured_velocity_rad_s",
    "max_acceleration_rad_s2": "max_measured_acceleration_rad_s2",
    "fcl_collision_count": "fcl_collision_count",
    "peak_force_n": "peak_force_n",
    "peak_torque_nm": "peak_torque_nm",
    "guard_warn_count": "guard_warn_samples",
    "estop_count": "estop_count",
    "control_failure_count": "control_failure_count",
    "stop_time_sec": "stop_observation_sec",
    "stop_distance_max_joint_rad": "stop_distance_max_joint_rad",
    "ik_dls_calls": "ik_dls_calls",
    "ik_refinements": "ik_refinements",
    "ik_fallback_calls": "ik_fallback_calls",
}


def parse_log(path):
    values = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        if key.replace("_", "").isalnum():
            values[key] = value.strip()
    return values


def append_row(path, row):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+", newline="", encoding="utf-8") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        stream.seek(0)
        existing = list(csv.DictReader(stream))
        if any(item.get("run_id") == row["run_id"] for item in existing):
            raise ValueError(f"duplicate run_id {row['run_id']!r}")
        stream.seek(0, 2)
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        if not existing and stream.tell() == 0:
            writer.writeheader()
        writer.writerow(row)
        stream.flush()
        fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


def main():
    parser = argparse.ArgumentParser(
        description="Record exactly one already-completed, supervised run; this tool never moves hardware.")
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--robot-serial", required=True)
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), required=True)
    parser.add_argument("--notes", default="")
    args = parser.parse_args()

    values = parse_log(args.log)
    if values.get("data_source") != "real_robot":
        raise SystemExit("log is not labelled data_source=real_robot")
    if values.get("reference_frame") != "base_link_to_Link7":
        raise SystemExit("log is not a base_link -> Link7 execution record")
    execution = values.get("execution")
    if execution not in {"PASS", "STOP"}:
        raise SystemExit("log does not contain a completed PASS or STOP execution")

    row = {field: "" for field in FIELDS}
    row.update({
        "run_id": args.run_id,
        "utc_timestamp": datetime.datetime.now(
            datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
        "robot_serial": args.robot_serial,
        "scenario": args.scenario,
        "status": execution,
        "reference_frame": values["reference_frame"],
        "notes": args.notes,
    })
    for field, source_key in KEYS.items():
        row[field] = values.get(source_key, "")
    append_row(args.csv, row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
