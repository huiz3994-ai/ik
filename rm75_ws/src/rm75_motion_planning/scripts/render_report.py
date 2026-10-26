#!/usr/bin/env python3
"""Render the machine-readable motion benchmark JSON as a compact Markdown table."""
import argparse
import json
import math


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("json")
    parser.add_argument("markdown")
    args = parser.parse_args()
    with open(args.json, encoding="utf-8") as stream:
        data = json.load(stream)
    if "solvers" in data:
        solvers = data["solvers"]
    else:
        solvers = {data.get("solver", "ipopt"): data["scenarios"]}
    lines = [
        "# RM75-6FB offline motion-planning benchmark",
        "",
        f"Fixed seed: `{data['seed']}`; repetitions per scenario: "
        f"`{data['count_per_scenario']}`; output: `{data['control_rate_hz']} Hz`.",
        "",
        "This is an offline numerical benchmark. No robot command was published.",
        "",
        "| Solver | Scenario | Success | Mean ms | P95 ms | P99 ms | Max pos mm | Max rot deg | Max v | Max a | Max jerk |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for solver, scenarios in solvers.items():
        for scenario, row in scenarios.items():
            lines.append(
                f"| {solver} | {scenario} | {100*row['success_rate']:.2f}% | "
                f"{row['mean_planning_ms']:.3f} | {row['p95_planning_ms']:.3f} | "
                f"{row['p99_planning_ms']:.3f} | {1000*row['max_position_error_m']:.3f} | "
                f"{math.degrees(row['max_orientation_error_rad']):.3f} | "
                f"{row['max_velocity_rad_s']:.3f} | {row['max_acceleration_rad_s2']:.3f} | "
                f"{row['max_jerk_rad_s3']:.3f} |"
            )
    lines += ["", "## Failure counts", ""]
    for solver, scenarios in solvers.items():
        for scenario, row in scenarios.items():
            failures = sum(row["failures"].values())
            if failures:
                lines.append(f"- `{solver}/{scenario}`: {failures} failed paths. "
                             f"Full reasons are retained in the JSON and CSV.")
    with open(args.markdown, "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
