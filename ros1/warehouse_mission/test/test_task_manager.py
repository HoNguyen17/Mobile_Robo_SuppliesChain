import unittest
import tempfile
import shutil
import os
from warehouse_mission.task_manager import TaskManager
from warehouse_mission.models import RunSummary, OUTCOME_SUCCESS


class TestTaskManager(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.mgr = TaskManager(buffer_dir=self.temp_dir)

    def tearDown(self):
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_fallback_task_retrieval(self):
        # Database is not reachable, should return fallback default task
        task, from_fallback = self.mgr.get_next_task(scenario_id="S-00")
        self.assertTrue(from_fallback)
        self.assertEqual(task.task_id, 1)
        self.assertEqual(task.shelf_pose.x, -3.75)
        self.assertEqual(task.shelf_pose.y, -3.13)
        self.assertEqual(task.dropoff_pose.x, 6.32)
        self.assertEqual(task.dropoff_pose.y, 5.87)

    def test_fallback_buffered_reporting(self):
        summary = RunSummary(
            task_id=1,
            scenario_id="S-00",
            planner="astar",
            outcome=OUTCOME_SUCCESS,
            path_length_m=10.0,
            baseline_m=10.0,
            duration_s=25.0,
        )
        res = self.mgr.report_run(summary)
        self.assertTrue(res)

        files = os.listdir(self.temp_dir)
        self.assertEqual(len(files), 1)
        self.assertTrue(files[0].startswith("run_1_"))


if __name__ == "__main__":
    unittest.main()
