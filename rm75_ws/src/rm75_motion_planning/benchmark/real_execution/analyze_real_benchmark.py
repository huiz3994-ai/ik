#!/usr/bin/env python3
"""Aggregate Link7 real-execution trials without manufacturing measurements."""

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

SCENARIOS = (
    "joint_small",
    "cartesian_line",
    "cartesian_orientation",
    "random_small_6d",
    "near_singular",
)

METRICS = (
    "final_link7_position_error_m",
    "final_link7_orientation_error_rad",
    "max_link7_position_error_m",
    "rms_link7_position_error_m",
    "max_link7_orientation_error_rad",
    "rms_link7_orientation_error_rad",
    "max_joint_error_rad",
    "rms_joint_error_rad",
    "max_joint_change_rad",
    "actual_control_frequency_hz",
    "mean_command_period_ms",
    "max_command_period_ms",
    "actual_feedback_frequency_hz",
    "mean_feedback_latency_ms",
    "max_feedback_latency_ms",
    "dropped_frames",
    "max_velocity_rad_s",
    "max_acceleration_rad_s2",
    "fcl_collision_count",
    "peak_force_n",
    "peak_torque_nm",
    "guard_warn_count",
    "estop_count",
    "control_failure_count",
    "stop_time_sec",
    "stop_distance_max_joint_rad",
    "ik_dls_calls",
    "ik_refinements",
    "ik_fallback_calls",
)


def optional_float(row, field):
    raw = row.get(field, "").strip()
    if not raw:
        return None
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(
            f"invalid {field!r} in row {row.get('run_id', '?')}"
        ) from exc
    if not math.isfinite(value):
        raise ValueError(f"non-finite {field!r} in row {row.get('run_id', '?')}")
    return value


def summarize(rows):
    grouped = defaultdict(list)
    seen = set()
    for row in rows:
        run_id = row.get("run_id", "")
        if not run_id or run_id in seen:
            raise ValueError(f"missing or duplicate run_id {run_id!r}")
        seen.add(run_id)
        scenario = row.get("scenario", "")
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario {scenario!r}")
        if row.get("reference_frame") != "base_link_to_Link7":
            raise ValueError(f"row {run_id} is not a Link7 real-execution record")
        if row.get("status") not in {"PASS", "STOP", "FAIL"}:
            raise ValueError(f"invalid status in row {run_id}")
        grouped[scenario].append(row)

    output = []
    for scenario in SCENARIOS:
        trials = grouped[scenario]
        passed = [row for row in trials if row["status"] == "PASS"]
        stopped = [row for row in trials if row["status"] == "STOP"]
        item = {
            "scenario": scenario,
            "attempted_runs": len(trials),
            "passed_runs": len(passed),
            "stopped_runs": len(stopped),
            "failed_runs": len(trials) - len(passed) - len(stopped),
            # The protocol is 100 attempts, not 100 cherry-picked successes.
            "requirement_met": len(trials) >= 100,
            "success_rate": len(passed) / len(trials) if trials else None,
            "metrics": {},
        }
        for metric in METRICS:
            values = []
            for row in trials:
                value = optional_float(row, metric)
                if value is not None:
                    values.append(value)
            item["metrics"][metric] = {
                "samples": len(values),
                "mean": sum(values) / len(values) if values else None,
                "max": max(values) if values else None,
                "min": min(values) if values else None,
            }
        output.append(item)
    return output


def write_csv(path, summary):
    fields = ["scenario", "attempted_runs", "passed_runs", "stopped_runs",
              "failed_runs", "success_rate", "requirement_met"]
    for metric in METRICS:
        fields += [f"{metric}_samples", f"{metric}_mean", f"{metric}_max",
                   f"{metric}_min"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for item in summary:
            row = {key: item[key] for key in fields if key in item}
            for metric in METRICS:
                for aggregate in ("samples", "mean", "max", "min"):
                    row[f"{metric}_{aggregate}"] = item["metrics"][metric][aggregate]
            writer.writerow(row)


def display(value, precision=6):
    return "N/A" if value is None else f"{value:.{precision}g}"


def write_markdown(path, source, summary):
    complete = all(item["requirement_met"] for item in summary)
    lines = [
        "# RM75-6FB Link7 real-execution benchmark",
        "",
        f"Source: `{source}`",
        "",
        "Verdict: **{}**. Completion requires 100 recorded attempts per scenario; "
        "STOP and FAIL attempts remain part of the denominator.".format(
            "COMPLETE" if complete else "INCOMPLETE"),
        "",
        "| Scenario | Attempts | PASS | STOP | FAIL | Success | Max Link7 pos (m) | Max Link7 rot (rad) | Max joint (rad) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in summary:
        metrics = item["metrics"]
        rate = item["success_rate"]
        lines.append(
            f"| {item['scenario']} | {item['attempted_runs']} | "
            f"{item['passed_runs']} | {item['stopped_runs']} | "
            f"{item['failed_runs']} | "
            f"{'N/A' if rate is None else f'{100.0 * rate:.1f}%'} | "
            f"{display(metrics['max_link7_position_error_m']['max'])} | "
            f"{display(metrics['max_link7_orientation_error_rad']['max'])} | "
            f"{display(metrics['max_joint_error_rad']['max'])} |"
        )
    lines += [
        "",
        "Only logs emitted by the real Cartesian executor with "
        "`data_source=real_robot` and `reference_frame=base_link_to_Link7` are "
        "accepted by the recorder. Gazebo data has a separate workflow.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_svg(path, summary):
    width, height, margin = 1000, 500, 70
    position = [item["metrics"]["max_link7_position_error_m"]["max"] or 0.0
                for item in summary]
    rotation = [item["metrics"]["max_link7_orientation_error_rad"]["max"] or 0.0
                for item in summary]
    pmax = max(position, default=0.0) or 1.0
    rmax = max(rotation, default=0.0) or 1.0
    group = (width - 2 * margin) / len(SCENARIOS)
    bar = group * .28
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="500" y="30" text-anchor="middle" font-family="sans-serif" font-size="18">Link7 real-execution maximum errors</text>',
        f'<line x1="{margin}" y1="410" x2="{width-margin}" y2="410" stroke="black"/>',
        '<rect x="70" y="50" width="14" height="14" fill="#2474b5"/><text x="90" y="62" font-family="sans-serif" font-size="12">position (independent scale)</text>',
        '<rect x="290" y="50" width="14" height="14" fill="#e07b39"/><text x="310" y="62" font-family="sans-serif" font-size="12">orientation (independent scale)</text>',
    ]
    for index, name in enumerate(SCENARIOS):
        center = margin + (index + .5) * group
        ph = 300 * position[index] / pmax
        rh = 300 * rotation[index] / rmax
        parts += [
            f'<rect x="{center-bar-2:.1f}" y="{410-ph:.1f}" width="{bar:.1f}" height="{ph:.1f}" fill="#2474b5"/>',
            f'<rect x="{center+2:.1f}" y="{410-rh:.1f}" width="{bar:.1f}" height="{rh:.1f}" fill="#e07b39"/>',
            f'<text x="{center:.1f}" y="430" text-anchor="middle" font-family="sans-serif" font-size="11">{name}</text>',
            f'<text x="{center-bar/2-2:.1f}" y="{max(405-ph, 82):.1f}" text-anchor="middle" font-family="sans-serif" font-size="10">{position[index]:.3g}m</text>',
            f'<text x="{center+bar/2+2:.1f}" y="{max(405-rh, 82):.1f}" text-anchor="middle" font-family="sans-serif" font-size="10">{rotation[index]:.3g}rad</text>',
        ]
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()
    with args.input_csv.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("refusing to create an empty real-execution report")
    missing = {"run_id", "scenario", "status", "reference_frame", *METRICS} - set(rows[0])
    if missing:
        raise SystemExit("missing CSV columns: " + ", ".join(sorted(missing)))
    summary = summarize(rows)
    prefix = args.output_prefix
    prefix.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "data_source": "real_robot",
        "reference_frame": "base_link_to_Link7",
        "input_csv": str(args.input_csv),
        "complete": all(item["requirement_met"] for item in summary),
        "scenarios": summary,
    }
    prefix.with_suffix(".json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write_csv(prefix.with_suffix(".csv"), summary)
    write_markdown(prefix.with_suffix(".md"), args.input_csv, summary)
    write_svg(prefix.with_suffix(".svg"), summary)
    return 0 if payload["complete"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
