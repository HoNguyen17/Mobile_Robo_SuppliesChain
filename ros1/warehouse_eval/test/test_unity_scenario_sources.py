"""Source checks on the Unity side of ADR-019 (nothing here runs Unity): the old protocol is still there, the new
files have .meta files, and the seeded path is only taken for messages with a '?'."""
import os
import re
import unittest

from warehouse_eval import scenario_spec as spec

ASSETS = os.path.join(spec.REPO_DIR, "WarehouseProjectURP", "Assets")


def read(*parts):
    with open(os.path.join(ASSETS, *parts), "r", encoding="utf-8") as f:
        return f.read()


class UnityScenarioSourcesTest(unittest.TestCase):
    def setUp(self):
        self.loader = read("Scripts", "Ros", "ScenarioLoader.cs")

    def test_the_plain_reset_still_builds_the_hardcoded_scenarios(self):
        self.assertIn('CreateCrossingNpc(new Vector3(-6f, 1f, 0f), new Vector3(6f, 1f, 0f), 0.5f, "S04_Worker")', self.loader)
        for sid in ("S-01", "S-02", "S-03", "S-04"):
            self.assertIn('scenarioId == "%s"' % sid, self.loader)

    def test_only_a_message_with_a_question_mark_takes_the_seeded_path(self):
        body = self.loader[self.loader.index("private void OnResetScenario"):self.loader.index("// ---- Seeded protocol")]
        self.assertRegex(body, r"IndexOf\('\?'\) >= 0\)\s*\{\s*OnSeededReset\(msg\.data\);\s*return;")
        self.assertIn("string scenarioId = msg.data.Trim().ToUpper();", body)
        self.assertIn("ResetRobotPose();\n            SetupScenarioObstacles(scenarioId);", body.replace("\r\n", "\n"))

    def test_a_plain_reset_ends_seeded_mode_so_the_draggable_box_comes_back(self):
        body = self.loader[self.loader.index("private void EndSeededMode"):self.loader.index("private void SetupSpecScenario")]
        self.assertIn("prePlacedDraggableObstacle.SetActive(true)", body)
        self.assertIn("draggableHiddenBySeeded", body)

    def test_the_seeded_path_never_falls_back_to_the_old_one(self):
        seeded = self.loader[self.loader.index("private void OnSeededReset"):self.loader.index("private void FailSeededReset")]
        self.assertNotIn("SetupScenarioObstacles(", seeded)
        self.assertGreaterEqual(seeded.count("FailSeededReset("), 4)

    def test_isolation_defaults_on_and_the_hotkeys_respect_it(self):
        self.assertRegex(self.loader, r"public bool isolateSeededScenarios = true;")
        self.assertIn("if (seededActive && isolateSeededScenarios)", self.loader)

    def test_the_legacy_npc_mover_behaviour_is_kept(self):
        mover = read("Scripts", "Robot", "NpcMover.cs")
        self.assertIn("Vector3.MoveTowards(transform.position, target, actualSpeed * Time.deltaTime)", mover)
        self.assertIn("if (specPath != null) return;", mover)

    def test_npc_poses_are_published_here_only_as_ground_truth(self):
        self.assertIn('npcPosesTopic = "/sim/npc_poses"', self.loader)
        self.assertIn("PoseArrayMsg", self.loader)
        self.assertNotRegex(self.loader, r"Subscribe<PoseArrayMsg>")

    def test_every_new_unity_file_has_a_meta_file_with_a_unique_guid(self):
        files = [("Scripts", "Robot", "NpcPath.cs"), ("Scripts", "Robot", "ScenarioSpec.cs"),
                 ("Tests", "EditMode", "NpcPathTests.cs"), ("StreamingAssets", "scenarios.json")]
        guids = []
        for parts in files:
            meta = read(*parts[:-1], parts[-1] + ".meta")
            m = re.search(r"^guid: ([0-9a-f]{32})$", meta, re.M)
            self.assertIsNotNone(m, parts)
            guids.append(m.group(1))
        folder = read("StreamingAssets.meta")
        self.assertIn("folderAsset: yes", folder)
        guids.append(re.search(r"^guid: ([0-9a-f]{32})$", folder, re.M).group(1))
        self.assertEqual(len(guids), len(set(guids)))


if __name__ == "__main__":
    unittest.main()
