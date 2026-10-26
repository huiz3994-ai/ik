#!/usr/bin/env python3

import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


class RealBenchmarkReportTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[1]
        self.script = self.root / "benchmark/real_execution/analyze_real_benchmark.py"
        self.recorder = self.root / "benchmark/real_execution/record_real_trial.py"
        self.template = self.root / "benchmark/real_execution/trials.template.csv"

    def make_input(self, directory, count):
        target = directory / "trials.csv"
        with self.template.open(newline="", encoding="utf-8") as stream:
            fields = next(csv.reader(stream))
        with target.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for scenario in ("joint_small", "cartesian_line",
                             "cartesian_orientation", "random_small_6d",
                             "near_singular"):
                for index in range(count):
                    row = {field: "" for field in fields}
                    row.update({
                        "run_id": f"{scenario}-{index}",
                        "scenario": scenario,
                        "status": "PASS" if index else "STOP",
                        "reference_frame": "base_link_to_Link7",
                        "max_link7_position_error_m": "0.001",
                        "max_link7_orientation_error_rad": "0.002",
                        "max_joint_error_rad": "0.01",
                    })
                    writer.writerow(row)
        return target

    def test_complete_report_generates_all_formats_and_keeps_stops(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = self.make_input(directory, 100)
            prefix = directory / "report"
            result = subprocess.run(
                [sys.executable, str(self.script), str(source),
                 "--output-prefix", str(prefix)], check=False)
            self.assertEqual(result.returncode, 0)
            for suffix in (".csv", ".json", ".md", ".svg"):
                self.assertTrue(prefix.with_suffix(suffix).is_file())
            payload = json.loads(prefix.with_suffix(".json").read_text())
            self.assertTrue(payload["complete"])
            self.assertEqual(payload["scenarios"][0]["attempted_runs"], 100)
            self.assertEqual(payload["scenarios"][0]["stopped_runs"], 1)

    def test_partial_report_is_explicitly_incomplete(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            source = self.make_input(directory, 2)
            result = subprocess.run(
                [sys.executable, str(self.script), str(source),
                 "--output-prefix", str(directory / "report")], check=False)
            self.assertEqual(result.returncode, 3)

    def test_recorder_appends_one_real_link7_log(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            log = directory / "run.log"
            log.write_text(
                "data_source=real_robot\n"
                "reference_frame=base_link_to_Link7\n"
                "execution=PASS\n"
                "final_Link7_position_error_m=0.001\n"
                "estop_count=0\n", encoding="utf-8")
            target = directory / "trials.csv"
            result = subprocess.run([
                sys.executable, str(self.recorder), "--log", str(log),
                "--csv", str(target), "--run-id", "line-001",
                "--robot-serial", "test-robot", "--scenario",
                "cartesian_line"], check=False)
            self.assertEqual(result.returncode, 0)
            with target.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["reference_frame"], "base_link_to_Link7")
            self.assertEqual(rows[0]["final_link7_position_error_m"], "0.001")

    def test_recorder_rejects_gazebo_shadow_log(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            log = directory / "shadow.log"
            log.write_text(
                "data_source=gazebo_shadow\n"
                "reference_frame=base_link_to_Link7\n"
                "execution=PASS\n", encoding="utf-8")
            result = subprocess.run([
                sys.executable, str(self.recorder), "--log", str(log),
                "--csv", str(directory / "trials.csv"), "--run-id",
                "shadow-001", "--robot-serial", "sim", "--scenario",
                "cartesian_line"], check=False,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((directory / "trials.csv").exists())


if __name__ == "__main__":
    unittest.main()
