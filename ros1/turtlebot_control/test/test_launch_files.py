"""Static XML tests for ROS launch files using xml.etree.ElementTree.

Pure Python 3.8, 100% ROS-free. Validates launch file schemas, argument
pass-through, parameter definitions, and planner node exclusions.
"""

import os
import unittest
import xml.etree.ElementTree as ET


class TestLaunchFiles(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        test_dir = os.path.dirname(os.path.abspath(__file__))
        cls.bringup_dstar_path = os.path.normpath(
            os.path.join(test_dir, "..", "..", "warehouse_bringup", "launch", "bringup_dstar.launch")
        )
        cls.bringup_path = os.path.normpath(
            os.path.join(test_dir, "..", "..", "warehouse_bringup", "launch", "bringup.launch")
        )
        cls.nav_dstar_path = os.path.normpath(
            os.path.join(test_dir, "..", "launch", "navigation_dstar.launch")
        )

        cls.tree_bringup_dstar = ET.parse(cls.bringup_dstar_path)
        cls.tree_bringup = ET.parse(cls.bringup_path)
        cls.tree_nav_dstar = ET.parse(cls.nav_dstar_path)

    def test_bringup_include_args_declared(self):
        """(a) Every arg passed to <include> of bringup.launch in bringup_dstar.launch is declared in bringup.launch."""
        root_dstar = self.tree_bringup_dstar.getroot()
        root_bringup = self.tree_bringup.getroot()

        # Find the <include> targeting bringup.launch
        bringup_includes = [
            inc for inc in root_dstar.findall("include")
            if "bringup.launch" in (inc.get("file") or "")
        ]
        self.assertEqual(len(bringup_includes), 1, "Expected exactly one include of bringup.launch")
        include_elem = bringup_includes[0]

        # Arguments declared in bringup.launch
        declared_args = {arg.get("name") for arg in root_bringup.findall("arg") if arg.get("name")}

        # Arguments passed in the include
        passed_args = [arg.get("name") for arg in include_elem.findall("arg") if arg.get("name")]
        self.assertTrue(len(passed_args) > 0, "No arguments passed to bringup.launch include")

        for arg_name in passed_args:
            self.assertIn(
                arg_name,
                declared_args,
                f"Arg '{arg_name}' passed to bringup.launch is not declared in bringup.launch"
            )

    def test_nav_arg_passed_as_false(self):
        """(b) nav argument is passed as false to bringup.launch."""
        root_dstar = self.tree_bringup_dstar.getroot()
        bringup_includes = [
            inc for inc in root_dstar.findall("include")
            if "bringup.launch" in (inc.get("file") or "")
        ]
        self.assertEqual(len(bringup_includes), 1)
        include_elem = bringup_includes[0]

        nav_args = [arg for arg in include_elem.findall("arg") if arg.get("name") == "nav"]
        self.assertEqual(len(nav_args), 1, "Expected exactly one 'nav' arg passed in include")
        self.assertEqual(
            nav_args[0].get("value", "").lower(),
            "false",
            "nav argument must be passed as 'false'"
        )

    def test_navigation_dstar_launch_included(self):
        """(c) navigation_dstar.launch is included in bringup_dstar.launch."""
        root_dstar = self.tree_bringup_dstar.getroot()
        includes = root_dstar.findall("include")
        has_nav_dstar = any(
            "navigation_dstar.launch" in (inc.get("file") or "") for inc in includes
        )
        self.assertTrue(has_nav_dstar, "bringup_dstar.launch must include navigation_dstar.launch")

    def test_dstar_planner_parameters(self):
        """(d) The three /dstar_planner params exist with true, true, false."""
        root_dstar = self.tree_bringup_dstar.getroot()
        params = {p.get("name"): p.get("value") for p in root_dstar.findall("param")}

        self.assertIn("/dstar_planner/filter_static_hits", params)
        self.assertEqual(params["/dstar_planner/filter_static_hits"].lower(), "true")

        self.assertIn("/dstar_planner/blocked_use_plan_radius", params)
        self.assertEqual(params["/dstar_planner/blocked_use_plan_radius"].lower(), "true")

        self.assertIn("/dstar_planner/verify_costmap", params)
        self.assertEqual(params["/dstar_planner/verify_costmap"].lower(), "false")

    def test_no_astar_planner_node_anywhere(self):
        """(e) No astar_planner node anywhere in the three launch files."""
        files = [
            ("bringup_dstar.launch", self.tree_bringup_dstar),
            ("bringup.launch", self.tree_bringup),
            ("navigation_dstar.launch", self.tree_nav_dstar),
        ]
        for fname, tree in files:
            root = tree.getroot()
            for node in root.findall(".//node"):
                node_name = node.get("name", "")
                node_type = node.get("type", "")
                self.assertNotIn(
                    "astar_planner",
                    node_name,
                    f"Found astar_planner node name in {fname}: {node_name}"
                )
                self.assertNotIn(
                    "astar_planner",
                    node_type,
                    f"Found astar_planner node type in {fname}: {node_type}"
                )


if __name__ == "__main__":
    unittest.main()
