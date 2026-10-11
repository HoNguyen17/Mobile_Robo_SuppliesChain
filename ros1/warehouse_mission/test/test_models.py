import unittest
import math
from warehouse_mission.models import (
    Pose2D,
    Task,
    RunEvent,
    RunSummary,
    CATEGORY_STANDARD,
    CATEGORY_HEAVY,
    CATEGORY_FRAGILE,
    OUTCOME_SUCCESS,
    OUTCOME_FAIL,
    FAIL_TIMEOUT,
    EVENT_COLLISION,
)


class TestModels(unittest.TestCase):
    def test_pose2d_distance(self):
        p1 = Pose2D(0.0, 0.0)
        p2 = Pose2D(3.0, 4.0)
        self.assertAlmostEqual(p1.distance_to(p2), 5.0)

    def test_task_valid_and_invalid_category(self):
        task = Task(
            task_id=1,
            item_id=1,
            category=CATEGORY_STANDARD,
            shelf_pose=Pose2D(1.0, 2.0),
            dropoff_pose=Pose2D(3.0, 4.0),
        )
        self.assertEqual(task.category, "Standard")

        with self.assertRaises(ValueError):
            Task(
                task_id=2,
                item_id=2,
                category="NonExistent",
                shelf_pose=Pose2D(0, 0),
                dropoff_pose=Pose2D(1, 1),
            )

    def test_run_event_valid_and_invalid(self):
        ev = RunEvent(event_type=EVENT_COLLISION, sim_time_s=12.5, payload={"other_tag": "Wall"})
        self.assertEqual(ev.event_type, "collision")

        with self.assertRaises(ValueError):
            RunEvent(event_type="invalid_event", sim_time_s=1.0)

    def test_run_summary_path_ratio(self):
        summary = RunSummary(
            task_id=1,
            scenario_id="S-00",
            planner="astar",
            outcome=OUTCOME_SUCCESS,
            path_length_m=12.0,
            baseline_m=10.0,
        )
        self.assertAlmostEqual(summary.path_ratio, 1.2)


if __name__ == "__main__":
    unittest.main()
