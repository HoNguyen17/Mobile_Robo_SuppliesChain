"""ab_batch passes the seeded protocol to ab_run (ADR-019), and only ab_run subscribes to /sim/npc_poses."""
import os
import re
import sys
import tempfile
import unittest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
if TEST_DIR not in sys.path:
    sys.path.insert(0, TEST_DIR)

import support  # noqa: F401,E402 (adds scripts/ to sys.path)
import ab_batch  # noqa: E402
from test_ab_batch import FakeRunner  # noqa: E402

ROS1_DIR = os.path.normpath(os.path.join(TEST_DIR, "..", ".."))


class SeededBatchTest(unittest.TestCase):
    def _ab_run_cmd(self, extra):
        with tempfile.TemporaryDirectory() as tmpdir:
            fake = FakeRunner()
            args = ab_batch.parse_args(["--planner", "astar", "--repeats", "2", "--out", tmpdir] + extra)
            ret = ab_batch.run_batch(args, runner=fake)
            self.assertEqual(ret, 0)
            cmd = [c for c in fake.popen_calls if "ab_run.py" in " ".join(c)][0]
            reports = [c for c in fake.run_calls if "ab_report.py" in " ".join(c)]
        return cmd, reports

    def test_defaults_leave_the_old_ab_run_command_unchanged(self):
        cmd, reports = self._ab_run_cmd([])
        self.assertNotIn("--seed-base", cmd)
        self.assertNotIn("--first-index", cmd)
        self.assertEqual(len(reports), 1)

    def test_seed_base_and_first_index_are_passed_on(self):
        cmd, _ = self._ab_run_cmd(["--seed-base", "1000", "--first-index", "7"])
        self.assertEqual(cmd[cmd.index("--seed-base") + 1], "1000")
        self.assertEqual(cmd[cmd.index("--first-index") + 1], "7")

    def test_no_report_skips_the_report(self):
        _, reports = self._ab_run_cmd(["--no-report"])
        self.assertEqual(reports, [])

    def test_parse_args(self):
        args = ab_batch.parse_args([])
        self.assertEqual((args.seed_base, args.first_index, args.no_report), (None, 1, False))

    def test_meta_hashes_the_scenario_files(self):
        for rel in ("ros1/warehouse_eval/config/scenarios.yaml", "WarehouseProjectURP/Assets/StreamingAssets/scenarios.json",
                    "WarehouseProjectURP/Assets/Scripts/Robot/NpcPath.cs"):
            self.assertIn(rel, ab_batch.META_TRACKED_FILES)


class NpcPosesRuleTest(unittest.TestCase):
    """/sim/npc_poses is ground truth for the metrics. No planner, follower, mission node or tool may read it."""

    ALLOWED = {"ab_run.py"}

    def test_only_ab_run_mentions_the_topic_in_ros_code(self):
        offenders = []
        for root, dirs, files in os.walk(ROS1_DIR):
            dirs[:] = [d for d in dirs if d not in ("test", "__pycache__", "build", "devel")]
            for name in files:
                if not name.endswith((".py", ".launch", ".yaml", ".xml")) or name in self.ALLOWED:
                    continue
                path = os.path.join(root, name)
                with open(path, "r", encoding="utf-8", errors="replace") as f:
                    if "npc_poses" in f.read():
                        offenders.append(os.path.relpath(path, ROS1_DIR))
        self.assertEqual(offenders, [])

    def test_ab_run_subscribes_exactly_once(self):
        with open(os.path.join(ROS1_DIR, "turtlebot_control", "scripts", "ab_run.py"), "r", encoding="utf-8") as f:
            src = f.read()
        self.assertEqual(len(re.findall(r'Subscriber\(\s*"/sim/npc_poses"', src)), 1)
        self.assertNotRegex(src, r'Publisher\(\s*"/sim/npc_poses"')

    def test_the_planners_and_the_follower_have_no_pose_array_input(self):
        for name in ("astar_planner.py", "dstar_planner.py", "path_follower.py", "nav_common.py"):
            with open(os.path.join(ROS1_DIR, "turtlebot_control", "scripts", name), "r", encoding="utf-8") as f:
                src = f.read()
            self.assertNotIn("PoseArray", src, name)
            self.assertNotIn("npc_poses", src, name)


if __name__ == "__main__":
    unittest.main()
