"""Source checks for the planner nodes, which cannot be imported without rospy (ADR-016, ADR-017).

The nodes must hand every plan to the PlanScheduler, listen to /nav/cancel, and take the live
costmap radius from nav_common.costmap_radius.
"""
import ast
import os
import unittest

SCRIPTS = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts")
)
PLANNERS = {"astar_planner.py": "AStarPlanner", "dstar_planner.py": "DStarPlanner"}


def load(name):
    with open(os.path.join(SCRIPTS, name), "r", encoding="utf-8") as f:
        src = f.read()
    return src, ast.parse(src)


def method(tree, class_name, method_name):
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == method_name:
                    return item
    raise AssertionError("%s.%s not found" % (class_name, method_name))


def calls_attr(func, attr):
    return [
        n for n in ast.walk(func)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == attr
    ]


class PlannerSchedulingSourceTest(unittest.TestCase):
    def each(self):
        for name, cls in PLANNERS.items():
            src, tree = load(name)
            yield name, cls, src, tree

    def test_planners_subscribe_to_nav_cancel(self):
        for name, _cls, src, _tree in self.each():
            self.assertIn('rospy.Subscriber("/nav/cancel", Empty, self.on_cancel)', src, name)

    def test_planners_use_the_scheduler(self):
        for name, _cls, src, _tree in self.each():
            self.assertIn("from plan_scheduler import", src, name)
            self.assertIn("PlanScheduler(", src, name)

    def test_no_callback_plans_directly(self):
        for name, cls, _src, tree in self.each():
            for callback in ("on_scan", "on_goal", "on_replan", "on_cancel"):
                self.assertEqual(
                    calls_attr(method(tree, cls, callback), "plan_for_goal"),
                    [],
                    "%s.%s must not call plan_for_goal; post it to the scheduler" % (name, callback),
                )

    def test_only_the_scheduler_run_function_plans(self):
        for name, cls, _src, tree in self.each():
            run = method(tree, cls, "_run_plan")
            self.assertEqual(len(calls_attr(run, "plan_for_goal")), 1, name)
            callers = [
                item.name
                for node in ast.walk(tree)
                if isinstance(node, ast.ClassDef) and node.name == cls
                for item in node.body
                if isinstance(item, ast.FunctionDef) and calls_attr(item, "plan_for_goal")
            ]
            self.assertEqual(callers, ["_run_plan"], name)

    def test_callbacks_post_the_right_reasons(self):
        for name, cls, _src, tree in self.each():
            posted = {
                cb: [
                    c.args[0].id if c.args and isinstance(c.args[0], ast.Name) else None
                    for c in calls_attr(method(tree, cls, cb), "post")
                ]
                for cb in ("on_scan", "on_goal", "on_replan")
            }
            for cb, reason in (("on_scan", "SCAN_TRIGGER"), ("on_goal", "GOAL"), ("on_replan", "REPLAN")):
                self.assertEqual(posted[cb], [reason], "%s.%s must post %s exactly once" % (name, cb, reason))

    def test_on_cancel_clears_the_leg_state(self):
        for name, cls, _src, tree in self.each():
            on_cancel = method(tree, cls, "on_cancel")
            self.assertEqual(len(calls_attr(on_cancel, "cancel")), 1, name)
            assigned = {
                t.attr
                for n in ast.walk(on_cancel)
                if isinstance(n, ast.Assign)
                for t in n.targets
                if isinstance(t, ast.Attribute)
            }
            for attr in ("latest_goal", "current_path_corners", "current_path_radius"):
                self.assertIn(attr, assigned, "%s.on_cancel must reset %s" % (name, attr))

    def test_plan_for_goal_drops_a_cancelled_result_before_publishing(self):
        for name, cls, src, tree in self.each():
            plan = method(tree, cls, "plan_for_goal")
            self.assertEqual([a.arg for a in plan.args.args], ["self", "msg", "ticket"], name)
            text = ast.get_source_segment(src, plan)
            self.assertIn("ticket.cancelled", text, name)
            # the check comes before the first publish of the path
            self.assertLess(text.index("ticket.cancelled"), text.index("self.publish_path("), name)

    def test_live_costmap_uses_the_shared_radius_function(self):
        """ADR-017: the live costmap radius comes from nav_common.costmap_radius and the current plan."""
        for name, cls, src, tree in self.each():
            live = ast.get_source_segment(src, method(tree, cls, "publish_live_costmap"))
            self.assertIn("costmap_radius(self.current_path_radius, nominal)", live, name)


if __name__ == "__main__":
    unittest.main()
