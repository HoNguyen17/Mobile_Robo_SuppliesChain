"""Source checks for scripts/mission_node.py, which cannot be imported without rospy."""
import os
import unittest

NODE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "mission_node.py")


class TestMissionNodePlannerLabel(unittest.TestCase):
    def setUp(self):
        with open(NODE, "r", encoding="utf-8") as f:
            self.src = f.read()

    def test_planner_label_is_a_parameter_defaulting_to_astar(self):
        self.assertIn('rospy.get_param("~planner_name", "astar")', self.src)

    def test_planner_label_is_not_hardcoded(self):
        self.assertNotIn('MetricsCollector(planner_name="astar")', self.src)


if __name__ == "__main__":
    unittest.main()
