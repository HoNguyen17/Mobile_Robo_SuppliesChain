import unittest
import tempfile
import shutil

from warehouse_mission.models import (
    OUTCOME_SUCCESS,
    OUTCOME_FAIL,
    FAIL_TIMEOUT,
    FAIL_ITEM_ERROR,
    CATEGORY_FRAGILE,
)
from warehouse_mission.task_manager import TaskManager
from warehouse_mission.motion_profile import MotionProfileManager
from warehouse_mission.metrics_collector import MetricsCollector
from warehouse_mission.mission_orchestrator import (
    MissionOrchestrator,
    STATE_IDLE,
    STATE_TO_SHELF,
    STATE_AT_SHELF,
    STATE_TO_DROPOFF,
    STATE_AT_DROPOFF,
)


class TestMissionOrchestrator(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.task_mgr = TaskManager(buffer_dir=self.temp_dir)
        self.motion_mgr = MotionProfileManager()
        self.metrics_coll = MetricsCollector()

        self.goals_sent = []
        self.cancelled = False
        self.attach_called = False
        self.release_called = False

        self.orchestrator = MissionOrchestrator(
            task_manager=self.task_mgr,
            motion_profile=self.motion_mgr,
            metrics_collector=self.metrics_coll,
            send_goal_fn=lambda p: self.goals_sent.append(p),
            cancel_nav_fn=lambda: setattr(self, "cancelled", True),
            attach_item_fn=lambda item_id: setattr(self, "attach_called", True) or True,
            release_item_fn=lambda: setattr(self, "release_called", True) or True,
            leg_timeout_s=10.0,
            dwell_time_s=2.0,
        )

    def tearDown(self):
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_full_mission_lifecycle_success(self):
        # 1. Dispatch
        ok = self.orchestrator.dispatch_next_task(0.0, 0.0, sim_time_s=10.0)
        self.assertTrue(ok)
        self.assertEqual(self.orchestrator.state, STATE_TO_SHELF)
        self.assertEqual(len(self.goals_sent), 1)

        # 2. Arrive at shelf
        self.orchestrator.on_nav_result("succeeded", "", sim_time_s=15.0)
        self.assertEqual(self.orchestrator.state, STATE_AT_SHELF)

        # 3. Dwell at shelf (2s dwell)
        self.orchestrator.step(0.0, 0.0, sim_time_s=16.0)
        self.assertEqual(self.orchestrator.state, STATE_AT_SHELF)  # Not yet 2s

        self.orchestrator.step(0.0, 0.0, sim_time_s=17.0)
        self.assertEqual(self.orchestrator.state, STATE_TO_DROPOFF)
        self.assertTrue(self.attach_called)
        self.assertEqual(len(self.goals_sent), 2)

        # 4. Arrive at dropoff
        self.orchestrator.on_nav_result("succeeded", "", sim_time_s=22.0)
        self.assertEqual(self.orchestrator.state, STATE_AT_DROPOFF)

        # 5. Dwell at dropoff
        self.orchestrator.step(5.0, 5.0, sim_time_s=24.0)
        self.assertTrue(self.release_called)
        self.assertEqual(self.orchestrator.state, STATE_IDLE)
        self.assertIsNotNone(self.orchestrator.latest_summary)
        self.assertEqual(self.orchestrator.latest_summary.outcome, OUTCOME_SUCCESS)

    def test_timeout_cancels_and_fails(self):
        self.orchestrator.dispatch_next_task(0.0, 0.0, sim_time_s=10.0)
        self.assertEqual(self.orchestrator.state, STATE_TO_SHELF)

        # Step beyond leg_timeout_s (10s)
        self.orchestrator.step(0.0, 0.0, sim_time_s=25.0)
        self.assertTrue(self.cancelled)
        self.assertEqual(self.orchestrator.state, STATE_IDLE)
        self.assertEqual(self.orchestrator.latest_summary.outcome, OUTCOME_FAIL)
        self.assertEqual(self.orchestrator.latest_summary.fail_reason, FAIL_TIMEOUT)


if __name__ == "__main__":
    unittest.main()
