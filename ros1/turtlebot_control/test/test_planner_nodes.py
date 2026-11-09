"""The real planner nodes against a fake rospy: scheduling and /nav/cancel (ADR-016).

Both nodes are loaded from scripts/ with a fake rospy (see fake_rospy_context in
test_dstar_lite.py). Planning runs on the PlanScheduler thread, so every test waits for
`node.scheduler.wait_idle()` before it looks at what was published.
"""
import threading
import unittest

import support  # noqa: F401  (puts scripts/ on sys.path)
from test_dstar_lite import (
    FakeTime,
    LaserScanStub,
    OccupancyGridStub,
    PoseStampedStub,
    fake_rospy_context,
)

WAIT = 10.0
PLANNERS = [
    ("astar_planner.py", "AStarPlanner", "astar"),
    ("dstar_planner.py", "DStarPlanner", "dstar"),
]


def free_map(w=200, h=120, res=0.05):
    msg = OccupancyGridStub()
    msg.info.resolution = res
    msg.info.width = w
    msg.info.height = h
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * (w * h)
    return msg


def pose_at(sim_time, x=1.0, y=3.0):
    msg = PoseStampedStub()
    msg.header.frame_id = "map"
    msg.header.stamp = FakeTime(sim_time)
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    return msg


def goal_at(sim_time, x, y=3.0):
    msg = PoseStampedStub()
    msg.header.frame_id = "map"
    msg.header.stamp = FakeTime(sim_time)
    msg.pose.position.x = x
    msg.pose.position.y = y
    return msg


def scan_with_hit_ahead(sim_time, distance=1.5):
    """A scan that is empty except for one hit straight ahead of a robot with yaw 0."""
    msg = LaserScanStub()
    msg.header.stamp = FakeTime(sim_time)
    msg.ranges = [float("inf")] * 360
    msg.ranges[0] = distance
    return msg


def hold_first_plan(node, mod, kind):
    """Make the first search block until `release` is set. Later searches run normally."""
    started, release = threading.Event(), threading.Event()
    calls = []

    def wrap(original):
        def slow(*args, **kwargs):
            calls.append(1)
            if len(calls) == 1:
                started.set()
                release.wait(WAIT)
            return original(*args, **kwargs)

        return slow

    if kind == "astar":
        mod.plan_inflated = wrap(mod.plan_inflated)
    else:
        node.planner.plan = wrap(node.planner.plan)
    return started, release


def fail_first_plan(node, mod, kind):
    calls = []

    def wrap(original):
        def failing(*args, **kwargs):
            calls.append(1)
            if len(calls) == 1:
                raise RuntimeError("boom")
            return original(*args, **kwargs)

        return failing

    if kind == "astar":
        mod.plan_inflated = wrap(mod.plan_inflated)
    else:
        node.planner.plan = wrap(node.planner.plan)


class PlannerNodeTest(unittest.TestCase):
    def rig(self, script, cls, kind, stack):
        ctx = fake_rospy_context({}, "pn_%s_%d" % (kind, id(stack)), script)
        mod, fake = ctx.__enter__()
        stack.callback(ctx.__exit__, None, None, None)
        node = getattr(mod, cls)()
        node.on_map(free_map())
        node.on_pose(pose_at(fake.sim_time))
        return mod, fake, node

    def each(self):
        for script, cls, kind in PLANNERS:
            with self.subTest(planner=kind):
                import contextlib

                with contextlib.ExitStack() as stack:
                    mod, fake, node = self.rig(script, cls, kind, stack)
                    yield mod, fake, node, kind

    @staticmethod
    def paths(fake):
        return fake.publishers["/planned_path"]

    def test_a_goal_is_planned_off_the_callback_thread(self):
        for mod, fake, node, kind in self.each():
            started, release = hold_first_plan(node, mod, kind)
            try:
                node.on_goal(goal_at(fake.sim_time, 4.0))  # must return while the search is held
                self.assertTrue(started.wait(WAIT))
                self.assertEqual(self.paths(fake), [])
            finally:
                release.set()
            self.assertTrue(node.scheduler.wait_idle(WAIT))
            self.assertEqual(len(self.paths(fake)), 1)

    def test_cancel_while_planning_drops_the_result(self):
        for mod, fake, node, kind in self.each():
            started, release = hold_first_plan(node, mod, kind)
            try:
                node.on_goal(goal_at(fake.sim_time, 4.0))
                self.assertTrue(started.wait(WAIT))
                node.on_cancel(object())
            finally:
                release.set()
            self.assertTrue(node.scheduler.wait_idle(WAIT))
            self.assertEqual(self.paths(fake), [])
            self.assertEqual(fake.publishers["/nav/leg_result"], [])
            self.assertTrue(any("Plan dropped" in log[1] for log in fake.logs))
            self.assertIsNone(node.latest_goal)

    def test_a_newer_goal_drops_the_result_for_the_old_one(self):
        for mod, fake, node, kind in self.each():
            started, release = hold_first_plan(node, mod, kind)
            try:
                node.on_goal(goal_at(fake.sim_time, 2.0))
                self.assertTrue(started.wait(WAIT))
                node.on_goal(goal_at(fake.sim_time, 4.0))
            finally:
                release.set()
            self.assertTrue(node.scheduler.wait_idle(WAIT))
            paths = self.paths(fake)
            self.assertEqual(len(paths), 1)
            self.assertAlmostEqual(paths[0].poses[-1].pose.position.x, 4.0, places=1)

    def test_cancel_forgets_the_goal_and_the_path(self):
        for mod, fake, node, kind in self.each():
            node.on_goal(goal_at(fake.sim_time, 4.0))
            self.assertTrue(node.scheduler.wait_idle(WAIT))
            self.assertIsNotNone(node.latest_goal)
            self.assertIsNotNone(node.current_path_corners)
            self.assertIsNotNone(node.current_path_radius)

            node.on_cancel(object())
            self.assertIsNone(node.latest_goal)
            self.assertIsNone(node.current_path_corners)
            self.assertIsNone(node.current_path_radius)

            node.on_replan(object())  # nothing to replan for
            self.assertTrue(node.scheduler.wait_idle(WAIT))
            self.assertEqual(len(self.paths(fake)), 1)

    def test_a_stale_path_is_republished_without_a_cancel_but_not_after_one(self):
        """The bug of ADR-016: after the leg ended, a lidar hit near the old path triggered a replan."""
        for cancel in (False, True):
            for script, cls, kind in PLANNERS:
                with self.subTest(planner=kind, cancel=cancel):
                    import contextlib

                    with contextlib.ExitStack() as stack:
                        mod, fake, node = self.rig(script, cls, kind, stack)
                        node.on_goal(goal_at(fake.sim_time, 9.0))
                        self.assertTrue(node.scheduler.wait_idle(WAIT))
                        self.assertEqual(len(self.paths(fake)), 1)

                        if cancel:
                            node.on_cancel(object())

                        fake.sim_time += 2.0  # past the 1 s throttle of the early replan
                        node.on_pose(pose_at(fake.sim_time))
                        node.on_scan(scan_with_hit_ahead(fake.sim_time))
                        self.assertTrue(node.scheduler.wait_idle(WAIT))
                        self.assertEqual(len(self.paths(fake)), 1 if cancel else 2)

    def test_a_goal_after_a_cancel_is_planned(self):
        for mod, fake, node, kind in self.each():
            node.on_goal(goal_at(fake.sim_time, 4.0))
            self.assertTrue(node.scheduler.wait_idle(WAIT))
            node.on_cancel(object())
            node.on_goal(goal_at(fake.sim_time, 6.0))
            self.assertTrue(node.scheduler.wait_idle(WAIT))
            paths = self.paths(fake)
            self.assertEqual(len(paths), 2)
            self.assertAlmostEqual(paths[-1].poses[-1].pose.position.x, 6.0, places=1)

    def live_radius(self, mod, node, kind, fake):
        """The inflation radius the live costmap was built with by the last on_scan (main thread)."""
        if kind == "astar":
            radii = []
            original = mod.inflate

            def record(grid, radius):
                if threading.current_thread() is threading.main_thread():  # not the planning thread
                    radii.append(radius)
                return original(grid, radius)

            mod.inflate = record
            try:
                fake.sim_time += 2.0
                node.on_pose(pose_at(fake.sim_time))
                node.on_scan(scan_with_hit_ahead(fake.sim_time))
            finally:
                mod.inflate = original
            self.assertEqual(len(radii), 1)
            return radii[0]
        node._live_radius = None
        fake.sim_time += 2.0
        node.on_pose(pose_at(fake.sim_time))
        node.on_scan(scan_with_hit_ahead(fake.sim_time))
        return node._live_radius

    def test_the_live_costmap_uses_the_radius_of_the_plan_being_followed(self):
        """ADR-017: before a plan the nominal radius, during a leg the planned one, after a cancel nominal again."""
        import contextlib

        for script, cls, kind in PLANNERS:
            with self.subTest(planner=kind), contextlib.ExitStack() as stack:
                ctx = fake_rospy_context({"/nav/inflation_radius": 0.35}, "pn_live_" + kind, script)
                mod, fake = ctx.__enter__()
                stack.callback(ctx.__exit__, None, None, None)
                node = getattr(mod, cls)()
                node.on_map(free_map())
                node.on_pose(pose_at(fake.sim_time))

                self.assertEqual(self.live_radius(mod, node, kind, fake), 0.35)  # no plan yet
                node.last_costmap_pub_time = 0.0

                node.on_goal(goal_at(fake.sim_time, 9.0))
                self.assertTrue(node.scheduler.wait_idle(WAIT))
                self.assertEqual(node.current_path_radius, 0.40)  # the planner prefers the wide margin
                node.last_costmap_pub_time = 0.0
                node.last_auto_replan_time = 1e12  # keep the scan below from starting a replan
                self.assertEqual(self.live_radius(mod, node, kind, fake), 0.40)

                node.on_cancel(object())
                node.last_costmap_pub_time = 0.0
                self.assertEqual(self.live_radius(mod, node, kind, fake), 0.35)

    def test_a_failing_plan_is_logged_and_the_node_keeps_planning(self):
        for mod, fake, node, kind in self.each():
            fail_first_plan(node, mod, kind)
            node.on_goal(goal_at(fake.sim_time, 4.0))
            self.assertTrue(node.scheduler.wait_idle(WAIT))
            self.assertEqual(self.paths(fake), [])
            self.assertTrue(any(log[0] == "ERR" and "Planning failed" in log[1] for log in fake.logs))

            node.on_goal(goal_at(fake.sim_time, 4.0))
            self.assertTrue(node.scheduler.wait_idle(WAIT))
            self.assertEqual(len(self.paths(fake)), 1)


if __name__ == "__main__":
    unittest.main()
