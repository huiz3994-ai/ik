#!/usr/bin/env python3
"""Persistent line-protocol wrapper around phase-one CasADi/Ipopt IK."""
import argparse
import importlib.util
import math
import sys

import numpy as np


def clean(text):
    return " ".join(str(text).replace("\r", " ").replace("\n", " ").split())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--optimization-ik", required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("optimization_ik", args.optimization_ik)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    solver = module.OptimizationIk(args.urdf, max_iterations=200, max_cpu_time=.2)
    # Prime the plugin before announcing protocol readiness. Ipopt prints its
    # license banner on the first solve; keeping that output before READY avoids
    # corrupting the line protocol on the first real fallback request.
    zero = np.zeros(7)
    solver.solve(solver.forward(zero), zero, warm_start=False)
    print("READY", flush=True)
    for line in sys.stdin:
        if line.strip() == "QUIT":
            return 0
        try:
            values = [float(value) for value in line.split()]
            if len(values) != 24:
                raise ValueError(f"expected 24 values, got {len(values)}")
            target = np.asarray(values[:16]).reshape(4, 4)
            seed = np.asarray(values[16:23])
            result = solver.solve(target, seed, warm_start=True,
                                  orientation_enabled=bool(round(values[23])))
            q = result.get("q", seed.tolist())
            fields = [result.get("status", "NUMERICAL_ERROR"),
                      result.get("solve_time_ms", 0.0), result.get("iterations", 0),
                      result.get("position_error_m", math.inf),
                      result.get("orientation_error_rad", math.inf),
                      result.get("min_limit_margin_rad", -math.inf), *q,
                      clean(result.get("detail", "no_detail"))]
            print(" ".join(map(str, fields)), flush=True)
        except Exception as error:
            print("NUMERICAL_ERROR 0 0 inf inf -inf 0 0 0 0 0 0 0 " +
                  clean(error), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
