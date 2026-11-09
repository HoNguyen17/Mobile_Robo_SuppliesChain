"""The scenario runners still write hardcoded results; each must say so loudly in its log."""
import os
import unittest

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts")
PHRASE = "PLACEHOLDER: results are not measured"


class TestPlaceholderRunners(unittest.TestCase):
    def read(self, name):
        with open(os.path.join(SCRIPTS, name), "r", encoding="utf-8") as f:
            return f.read()

    def check(self, name):
        src = self.read(name)
        self.assertIn(PHRASE, src)
        self.assertIn("rospy.logwarn(PLACEHOLDER_WARNING)", src)
        # warned right after the node starts, and again after the results are written
        self.assertGreaterEqual(src.count("warn_placeholder()"), 3)  # def + 2 calls
        self.assertLess(src.index("warn_placeholder()\n\n    rospack"), src.index("runner = ScenarioRunner"))

    def test_run_scenario_warns(self):
        self.check("run_scenario.py")

    def test_run_all_scenarios_warns(self):
        self.check("run_all_scenarios.py")


if __name__ == "__main__":
    unittest.main()
