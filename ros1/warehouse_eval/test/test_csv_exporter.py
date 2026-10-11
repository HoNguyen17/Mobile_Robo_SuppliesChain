import unittest
import os
import tempfile
import shutil
import csv

from warehouse_mission.models import RunSummary, OUTCOME_SUCCESS, OUTCOME_FAIL
from warehouse_eval.csv_exporter import CsvExporter


class TestCsvExporter(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.exporter = CsvExporter(output_dir=self.temp_dir)

    def tearDown(self):
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_export_runs_csv(self):
        runs = [
            RunSummary(
                task_id=1,
                scenario_id="S-00",
                planner="astar",
                outcome=OUTCOME_SUCCESS,
                collisions=0,
                min_clearance_m=0.25,
                path_length_m=10.0,
                baseline_m=10.0,
                duration_s=30.0,
            ),
            RunSummary(
                task_id=2,
                scenario_id="S-00",
                planner="astar",
                outcome=OUTCOME_FAIL,
                fail_reason="timeout",
                collisions=1,
                min_clearance_m=0.10,
                path_length_m=5.0,
                baseline_m=10.0,
                duration_s=90.0,
            ),
        ]
        csv_path = self.exporter.export_runs("S-00", runs)
        self.assertTrue(os.path.exists(csv_path))

        with open(csv_path, "r", encoding="utf-8") as f:
            reader = list(csv.reader(f))
            self.assertEqual(len(reader), 3)  # Header + 2 rows
            self.assertEqual(reader[1][5], "success")
            self.assertEqual(reader[2][5], "fail")

    def test_export_summary_csv(self):
        runs = [
            RunSummary(
                task_id=i,
                scenario_id="S-00",
                planner="astar",
                outcome=OUTCOME_SUCCESS,
                collisions=0,
                min_clearance_m=0.20,
                path_length_m=10.0,
                baseline_m=10.0,
                duration_s=30.0,
            )
            for i in range(1, 21)
        ]
        criteria = {
            "success_rate": 0.90,
            "max_collisions": 0,
            "max_path_ratio": 1.3,
        }
        sum_path = self.exporter.export_summary("S-00", runs, criteria)
        self.assertTrue(os.path.exists(sum_path))

        with open(sum_path, "r", encoding="utf-8") as f:
            reader = list(csv.reader(f))
            self.assertEqual(len(reader), 2)  # Header + 1 summary row
            self.assertEqual(reader[1][14], "PASS")  # Overall status


if __name__ == "__main__":
    unittest.main()
