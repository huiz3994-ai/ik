#!/usr/bin/env python3

import csv
import importlib.util
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class Stage4CharacterizationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = Path(__file__).resolve().parents[1]
        cls.script = cls.root / "benchmark/real_execution/analyze_characterization.py"
        spec = importlib.util.spec_from_file_location("characterization", cls.script)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    @staticmethod
    def fields():
        output = ["data_source", "reference_frame", "run_id", "scenario",
                  "repetition", "sample_time_sec", "command_time_sec",
                  "feedback_time_sec", "feedback_source_time_sec",
                  "feedback_source_time_valid", "command_published",
                  "requested_linear_speed_mps", "requested_angular_speed_rad_s"]
        for pose in ("ideal", "command", "actual"):
            output += [f"{pose}_{field}" for field in
                       ("x_m", "y_m", "z_m", "qx", "qy", "qz", "qw")]
        for prefix in ("target_q", "target_dq", "target_ddq", "actual_q"):
            output += [f"{prefix}{joint}" for joint in range(1, 8)]
        return output + ["force_n", "torque_nm", "controller_status",
                         "guard_result", "collision", "stop_reason"]

    def write_run(self, path, scenario="x", repetition=1, sign=1.0, samples=80):
        fields = self.fields()
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index in range(samples):
                time = 10.0 + .02 * index
                phase = math.pi * index / (samples - 1)
                target = sign * .01 * (1.0 - math.cos(phase)) * .5
                delayed = max(0, index - 2)
                delayed_phase = math.pi * delayed / (samples - 1)
                actual = sign * .01 * (1.0 - math.cos(delayed_phase)) * .5
                ideal_xyz = [target if scenario == axis else 0.0 for axis in "xyz"]
                actual_xyz = [actual if scenario == axis else 0.0 for axis in "xyz"]
                angle = target if scenario in ("rx", "ry", "rz") else 0.0
                axis_index = {"rx": 0, "ry": 1, "rz": 2}.get(scenario, 2)
                q = [0.0, 0.0, 0.0, math.cos(angle / 2)]
                q[axis_index] = math.sin(angle / 2)
                actual_angle = actual if scenario in ("rx", "ry", "rz") else 0.0
                qa = [0.0, 0.0, 0.0, math.cos(actual_angle / 2)]
                qa[axis_index] = math.sin(actual_angle / 2)
                row = {field: "" for field in fields}
                row.update({
                    "data_source": "real_robot", "reference_frame": "base_link_to_Link7",
                    "run_id": f"{scenario}-{repetition}", "scenario": scenario,
                    "repetition": repetition, "sample_time_sec": time + .003,
                    "requested_linear_speed_mps": .01,
                    "requested_angular_speed_rad_s": math.radians(5.0),
                    "command_time_sec": time, "feedback_time_sec": time + .001,
                    "feedback_source_time_sec": time,
                    "feedback_source_time_valid": 1,
                    "command_published": 1, "force_n": 1.0 + .01 * math.sin(phase),
                    "torque_nm": .05, "controller_status": "OK",
                    "guard_result": "ALLOW", "collision": 0, "stop_reason": "",
                })
                for prefix, xyz, quat in (("ideal", ideal_xyz, q),
                                          ("command", ideal_xyz, q),
                                          ("actual", actual_xyz, qa)):
                    for name, value in zip("xyz", xyz):
                        row[f"{prefix}_{name}_m"] = value
                    for name, value in zip("xyzw", quat):
                        row[f"{prefix}_q{name}"] = value
                speed = sign * .01 * math.sin(phase) * math.pi / (2 * (samples - 1) * .02)
                for joint in range(1, 8):
                    row[f"target_q{joint}"] = target if joint == 1 else 0.0
                    row[f"target_dq{joint}"] = speed if joint == 1 else 0.0
                    row[f"target_ddq{joint}"] = 0.0
                    row[f"actual_q{joint}"] = actual if joint == 1 else 0.0
                writer.writerow(row)

    def test_distribution_has_requested_statistics(self):
        result = self.module.stats([1.0, 2.0, 3.0, 4.0])
        self.assertAlmostEqual(result["mean"], 2.5)
        self.assertAlmostEqual(result["rms"], math.sqrt(7.5))
        self.assertAlmostEqual(result["max"], 4.0)
        self.assertAlmostEqual(result["p95"], 3.85)

    def test_partial_real_dataset_generates_all_outputs_and_stays_incomplete(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            inputs = []
            repetition = 1
            for scenario in ("x", "y", "z", "rx", "ry", "rz"):
                for sign in (1.0, -1.0):
                    source = directory / f"{scenario}-{repetition}.csv"
                    self.write_run(source, scenario, repetition, sign)
                    inputs.append(source)
                    repetition += 1
            source = directory / "6d.csv"
            self.write_run(source, "6d", repetition, 1.0)
            inputs.append(source)
            output = directory / "results"
            result = subprocess.run(
                [sys.executable, str(self.script), *map(str, inputs),
                 "--output-dir", str(output)], check=False,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertEqual(result.returncode, 3, result.stderr)
            for name in ("real_robot_benchmark.csv", "real_robot_benchmark.json",
                         "real_robot_benchmark_report.md",
                         "runtime_guard.recommended.yaml", "real_robot_benchmark.svg",
                         "tracking_error_curves.svg", "delay_distribution.svg"):
                self.assertTrue((output / name).is_file(), name)
            payload = json.loads((output / "real_robot_benchmark.json").read_text())
            self.assertFalse(payload["complete"])
            self.assertEqual(payload["guard_recommendation"]["status"],
                             "provisional_do_not_deploy")
            self.assertFalse(payload["guard_recommendation"]
                             ["normal_coverage_complete"])
            self.assertGreater(payload["timestamp_sync"]["source_stamp_rows"], 0)
            self.assertAlmostEqual(
                payload["timestamp_sync"]["source_to_receipt_offset_sec"]["mean"],
                .001, places=9)
            self.assertIsNotNone(
                payload["robot_state"]["max_joint_velocity_rad_s"]["max"])
            self.assertIsNotNone(
                payload["robot_state"]["max_joint_acceleration_rad_s2"]["max"])
            self.assertEqual(payload["runs"][0]["error_decomposition"]
                             ["ik_position_m"]["max"], 0.0)

    def test_rejects_simulation_rows(self):
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / "bad.csv"
            self.write_run(source)
            text = source.read_text(encoding="utf-8").replace(
                "real_robot", "gazebo_shadow")
            source.write_text(text, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-real"):
                self.module.validate_and_group([source])


if __name__ == "__main__":
    unittest.main()
