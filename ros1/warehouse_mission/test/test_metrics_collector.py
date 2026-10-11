import unittest
from warehouse_mission.metrics_collector import MetricsCollector, ROBOT_FOOTPRINT_RADIUS_M
from warehouse_mission.models import (
    Task,
    Pose2D,
    CATEGORY_STANDARD,
    LEG_SHELF,
    LEG_DROPOFF,
    OUTCOME_SUCCESS,
)


class TestMetricsCollector(unittest.TestCase):
    def setUp(self):
        self.collector = MetricsCollector(planner_name="astar")
        self.task = Task(
            task_id=10,
            item_id=5,
            category=CATEGORY_STANDARD,
            shelf_pose=Pose2D(0.0, 10.0),
            dropoff_pose=Pose2D(10.0, 10.0),
            scenario_id="S-00",
        )

    def test_complete_metrics_flow(self):
        # Start at (0, 0)
        self.collector.start_run(self.task, start_x=0.0, start_y=0.0, sim_time_s=100.0)
        self.collector.mark_leg(LEG_SHELF)

        # First path plan of shelf leg (does NOT count as replan)
        self.collector.on_planned_path(101.0)
        self.assertEqual(self.collector.replans, 0)

        # Second path plan of shelf leg (DOES count as replan)
        self.collector.on_planned_path(105.0)
        self.assertEqual(self.collector.replans, 1)

        # Recovery event
        self.collector.on_nav_event("recovery", 106.0)
        self.assertEqual(self.collector.recoveries, 1)

        # Lidar scan (clearance)
        self.collector.on_scan([1.5, 0.8, 2.0])
        self.assertAlmostEqual(self.collector.min_clearance_m, 0.8 - ROBOT_FOOTPRINT_RADIUS_M)

        # Movement to shelf (0, 10)
        self.collector.on_pose(0.0, 5.0, 110.0)
        self.collector.on_pose(0.0, 10.0, 120.0)

        # Mark drop-off leg
        self.collector.mark_leg(LEG_DROPOFF)
        self.collector.on_planned_path(125.0)  # First plan of drop-off leg
        self.assertEqual(self.collector.replans, 1)

        # Collision with shelf
        self.collector.on_collision("Shelf", 126.0, 0.0, 10.0)
        self.assertEqual(self.collector.collisions, 1)

        # Movement to drop-off (10, 10)
        self.collector.on_pose(5.0, 10.0, 140.0)
        self.collector.on_pose(10.0, 10.0, 160.0)

        summary = self.collector.finish_run(OUTCOME_SUCCESS, sim_time_s=165.0)

        self.assertEqual(summary.outcome, OUTCOME_SUCCESS)
        self.assertEqual(summary.collisions, 1)
        self.assertEqual(summary.replans, 1)
        self.assertEqual(summary.recoveries, 1)
        self.assertAlmostEqual(summary.path_length_m, 20.0)
        self.assertAlmostEqual(summary.baseline_m, 20.0)
        self.assertAlmostEqual(summary.path_ratio, 1.0)
        self.assertAlmostEqual(summary.duration_s, 65.0)
        # Drop-off leg traveled 10m over 40s (from 120s to 160s) -> 0.25 m/s
        self.assertAlmostEqual(summary.dropoff_mean_speed_mps, 0.25)


if __name__ == "__main__":
    unittest.main()
