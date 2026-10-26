#!/usr/bin/env python3
"""Offline trajectory-level CasADi/Ipopt benchmark for RM75-6FB.

This intentionally lives outside the ROS runtime and imports the phase-one
symbolic URDF solver. It never creates a ROS publisher or connects to hardware.
"""
import argparse
import csv
import importlib.util
import json
import math
import os
import time

import numpy as np


SCENARIOS = ("short_line", "large_cartesian", "random_6d",
             "near_singular", "near_joint_limit")
LOWER = np.array([-3.1, -2.268, -3.1, -2.355, -3.1, -2.233, -6.28])
UPPER = np.array([3.1, 2.268, 3.1, 2.355, 3.1, 2.233, 6.28])
VMAX = np.ones(7)
AMAX = np.full(7, .5)
JMAX = np.full(7, 2.0)


def so3_log(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    theta = math.acos(cosine)
    vee = np.array([rotation[2, 1] - rotation[1, 2],
                    rotation[0, 2] - rotation[2, 0],
                    rotation[1, 0] - rotation[0, 1]])
    if theta < 1e-8:
        return .5 * vee
    if math.pi - theta < 1e-6:
        values, vectors = np.linalg.eigh((rotation + np.eye(3)) / 2.0)
        axis = vectors[:, np.argmax(values)]
        return theta * axis
    return theta / (2.0 * math.sin(theta)) * vee


def so3_exp(vector):
    theta = np.linalg.norm(vector)
    if theta < 1e-12:
        return np.eye(3)
    axis = vector / theta
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(theta) * skew + (1 - math.cos(theta)) * skew @ skew


def interpolate(start, goal):
    distance = np.linalg.norm(goal[:3, 3] - start[:3, 3])
    rotation_vector = so3_log(start[:3, :3].T @ goal[:3, :3])
    segments = max(1, math.ceil(distance / .005),
                   math.ceil(np.linalg.norm(rotation_vector) / .05))
    output = []
    for index in range(segments + 1):
        alpha = index / segments
        pose = np.eye(4)
        pose[:3, 3] = (1 - alpha) * start[:3, 3] + alpha * goal[:3, 3]
        pose[:3, :3] = start[:3, :3] @ so3_exp(alpha * rotation_vector)
        output.append(pose)
    return output


def sample_q(rng, fraction):
    center = .5 * (LOWER + UPPER)
    return center + rng.uniform(-fraction, fraction, 7) * .5 * (UPPER - LOWER)


def dynamic_extrema(path):
    max_v = max_a = max_j = 0.0
    for q0, q1 in zip(path, path[1:]):
        delta = np.abs(q1 - q0)
        duration = max(np.max(1.875 * delta / VMAX),
                       np.max(np.sqrt((10 / math.sqrt(3)) * delta / AMAX)),
                       np.max(np.cbrt(60 * delta / JMAX)))
        if duration <= 1e-12:
            continue
        duration = math.ceil(duration / .02) * .02
        max_v = max(max_v, np.max(1.875 * delta / duration))
        max_a = max(max_a, np.max((10 / math.sqrt(3)) * delta / duration**2))
        max_j = max(max_j, np.max(60 * delta / duration**3))
    return max_v, max_a, max_j


def percentile(values, p):
    return float(np.percentile(values, 100 * p)) if values else 0.0


def main():
    global LOWER, UPPER
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--optimization-ik", required=True)
    parser.add_argument("--count-per-scenario", type=int, default=100)
    parser.add_argument("--seed", type=int, default=7506)
    parser.add_argument("--cases-csv", help="Reuse C++ benchmark start/goal configurations")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.count_per_scenario <= 0:
        raise SystemExit("count must be positive")
    spec = importlib.util.spec_from_file_location("optimization_ik", args.optimization_ik)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    solver = module.OptimizationIk(args.urdf, max_iterations=150, max_cpu_time=.08)
    # Derive position limits from the same parsed URDF as the symbolic solver.
    LOWER, UPPER = solver.lower.copy(), solver.upper.copy()
    rng = np.random.default_rng(args.seed)
    shared_cases = {}
    if args.cases_csv:
        with open(args.cases_csv, newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                if row["solver"] != "fixed_dls":
                    continue
                key = (row["scenario"], int(row["sample"]))
                if key[1] >= args.count_per_scenario:
                    continue
                shared_cases[key] = (
                    np.array([float(row[f"start_q{i}"]) for i in range(1, 8)]),
                    np.array([float(row[f"goal_q{i}"]) for i in range(1, 8)]))
        expected = len(SCENARIOS) * args.count_per_scenario
        if len(shared_cases) != expected:
            raise SystemExit(f"expected {expected} shared cases, found {len(shared_cases)}")
    rows = []
    for scenario in SCENARIOS:
        for sample in range(args.count_per_scenario):
            if shared_cases:
                start, goal = shared_cases[(scenario, sample)]
            else:
                start = sample_q(rng, .45)
                goal = start.copy()
                if scenario == "short_line":
                    goal += rng.normal(0, .01, 7)
                elif scenario == "large_cartesian":
                    goal = sample_q(rng, .55)
                elif scenario == "random_6d":
                    start, goal = sample_q(rng, .6), sample_q(rng, .6)
                elif scenario == "near_singular":
                    start = rng.normal(0, 1e-4, 7)
                    goal = start + rng.normal(0, .01, 7)
                else:
                    joint = sample % 7
                    start[joint] = UPPER[joint] - .06
                    goal = start + rng.normal(0, .003, 7)
                    goal[joint] -= .03
                goal = np.clip(goal, LOWER + 1e-5, UPPER - 1e-5)
            poses = interpolate(solver.forward(start), solver.forward(goal))
            q = start.copy()
            joints = [q.copy()]
            max_pe = max_re = max_jump = 0.0
            min_margin = float(np.min(np.minimum(q - LOWER, UPPER - q)))
            failure = ""
            started = time.perf_counter()
            for index, pose in enumerate(poses[1:], 1):
                result = solver.solve(pose, q, position_tolerance=.002,
                                      orientation_tolerance=math.radians(2),
                                      warm_start=True)
                if result["status"] != "SUCCESS":
                    failure = f"{result['status']} at pose {index}: {result.get('detail', '')}"
                    break
                solved = np.asarray(result["q"])
                jump = float(np.linalg.norm(solved - q))
                if jump > .35:
                    failure = f"JOINT_JUMP at pose {index}: {jump}"
                    break
                q = solved
                joints.append(q.copy())
                max_pe = max(max_pe, result["position_error_m"])
                max_re = max(max_re, result["orientation_error_rad"])
                max_jump = max(max_jump, jump)
                min_margin = min(min_margin, result["min_limit_margin_rad"])
            planning_ms = (time.perf_counter() - started) * 1000
            success = not failure
            max_v, max_a, max_j = dynamic_extrema(joints) if success else (0, 0, 0)
            rows.append(dict(solver="ipopt", scenario=scenario, sample=sample,
                             success=success, planning_ms=planning_ms,
                             pose_count=len(poses), max_position_error_m=max_pe,
                             max_orientation_error_rad=max_re,
                             max_velocity_rad_s=max_v,
                             max_acceleration_rad_s2=max_a,
                             max_jerk_rad_s3=max_j,
                             max_joint_jump_rad=max_jump,
                             min_joint_margin_rad=min_margin,
                             failure=failure))
        print(f"Completed {scenario} ({args.count_per_scenario} Ipopt paths)", flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output + ".csv", "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    summary = {"model": "RM75-6FB", "seed": args.seed,
               "count_per_scenario": args.count_per_scenario,
               "control_rate_hz": 50, "solver": "CasADi 3.7.2 / Ipopt",
               "shared_cpp_cases": bool(args.cases_csv),
               "scenarios": {}}
    for scenario in SCENARIOS:
        selected = [row for row in rows if row["scenario"] == scenario]
        passed = [row for row in selected if row["success"]]
        failures = {}
        for row in selected:
            if not row["success"]:
                key = row["failure"].split(" at pose")[0]
                failures[key] = failures.get(key, 0) + 1
        maximum = lambda key: max((row[key] for row in passed), default=0.0)
        summary["scenarios"][scenario] = {
            "success_rate": len(passed) / len(selected),
            "mean_planning_ms": float(np.mean([r["planning_ms"] for r in selected])),
            "p95_planning_ms": percentile([r["planning_ms"] for r in selected], .95),
            "p99_planning_ms": percentile([r["planning_ms"] for r in selected], .99),
            "max_position_error_m": maximum("max_position_error_m"),
            "max_orientation_error_rad": maximum("max_orientation_error_rad"),
            "max_velocity_rad_s": maximum("max_velocity_rad_s"),
            "max_acceleration_rad_s2": maximum("max_acceleration_rad_s2"),
            "max_jerk_rad_s3": maximum("max_jerk_rad_s3"),
            "max_joint_jump_rad": maximum("max_joint_jump_rad"),
            "min_joint_margin_rad": min((r["min_joint_margin_rad"] for r in passed), default=0.0),
            "failures": failures,
        }
    with open(args.output + ".json", "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)
    print(f"Wrote {args.output}.csv and {args.output}.json ({len(rows)} paths)")


if __name__ == "__main__":
    main()
