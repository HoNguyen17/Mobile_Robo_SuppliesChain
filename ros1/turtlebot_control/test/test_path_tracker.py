"""Unit tests for path_tracker: the follower's controller and path state."""
import math
import sys
import threading
import unittest
from types import SimpleNamespace

import support  # noqa: F401  (puts scripts/ on sys.path)
from path_tracker import (
    ARRIVED,
    DRIVING,
    FINAL_TOL,
    IDLE,
    K_LIN,
    MAX_ANG,
    STALE,
    PathTracker,
    normalize,
    yaw_from_quaternion,
)


def quaternion(yaw):
    return SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class AngleTest(unittest.TestCase):
    def test_yaw_survives_a_quaternion_round_trip(self):
        for yaw in (-3.0, -1.0, 0.0, 0.5, 2.5):
            self.assertAlmostEqual(yaw_from_quaternion(quaternion(yaw)), yaw)

    def test_normalize_wraps_into_minus_pi_to_pi(self):
        self.assertAlmostEqual(normalize(2 * math.pi + 0.1), 0.1)
        self.assertAlmostEqual(normalize(-2 * math.pi - 0.1), -0.1)
        self.assertAlmostEqual(normalize(math.pi + 0.1), -math.pi + 0.1)


class PathTrackerTest(unittest.TestCase):
    def setUp(self):
        self.tracker = PathTracker(waypoint_tol=0.3, pose_timeout=0.5)

    def step(self, x=0.0, y=0.0, yaw=0.0, age=0.0, max_lin=0.5):
        return self.tracker.step(x, y, yaw, age, max_lin)

    # ---- idle and path handling ----
    def test_without_a_path_it_is_idle(self):
        cmd = self.step()
        self.assertEqual(cmd.status, IDLE)
        self.assertFalse(self.tracker.active)

    def test_set_path_makes_it_active(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        self.assertTrue(self.tracker.active)

    def test_it_heads_for_the_second_waypoint_because_the_first_is_the_start(self):
        self.tracker.set_path([(0.0, 0.0), (0.0, 5.0)])  # second point is straight up
        cmd = self.step(yaw=math.pi / 2)  # robot already faces it
        self.assertEqual(cmd.status, DRIVING)
        self.assertAlmostEqual(cmd.angular, 0.0)
        self.assertGreater(cmd.linear, 0.0)

    def test_a_one_point_path_heads_for_that_point(self):
        self.tracker.set_path([(0.4, 0.0)])
        cmd = self.step()
        self.assertEqual(cmd.status, DRIVING)
        self.assertAlmostEqual(cmd.linear, K_LIN * 0.4)

    def test_an_empty_path_is_ignored(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        self.tracker.set_path([])
        self.assertTrue(self.tracker.active)

    def test_a_new_path_replaces_the_old_one_and_restarts_from_its_second_point(self):
        self.tracker.set_path([(0.0, 0.0), (1.0, 0.0), (1.0, 5.0)])
        self.step(x=0.9)  # moves on to the third point
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        cmd = self.step(x=0.0)
        self.assertAlmostEqual(cmd.angular, 0.0)  # now chasing (5, 0), not (1, 5)
        self.assertGreater(cmd.linear, 0.0)

    # ---- controller ----
    def test_it_turns_in_place_when_the_heading_error_is_large(self):
        self.tracker.set_path([(0.0, 0.0), (0.0, 5.0)])
        cmd = self.step(yaw=0.0)  # target is 90 degrees to the left
        self.assertEqual(cmd.linear, 0.0)
        self.assertEqual(cmd.angular, MAX_ANG)

    def test_it_turns_towards_a_target_on_the_right(self):
        self.tracker.set_path([(0.0, 0.0), (0.0, -5.0)])
        cmd = self.step(yaw=0.0)
        self.assertEqual(cmd.linear, 0.0)
        self.assertEqual(cmd.angular, -MAX_ANG)

    def test_the_speed_limit_comes_from_max_lin_between_corners(self):
        self.tracker.set_path([(0.0, 0.0), (1.0, 0.0), (10.0, 0.0)])
        self.assertEqual(self.step(max_lin=0.2).linear, 0.2)

    def test_the_final_approach_slows_down(self):
        self.tracker.set_path([(0.0, 0.0), (0.4, 0.0)])
        self.assertAlmostEqual(self.step().linear, K_LIN * 0.4)

    def test_the_final_approach_never_exceeds_max_lin(self):
        self.tracker.set_path([(0.0, 0.0), (10.0, 0.0)])
        self.assertEqual(self.step(max_lin=0.1).linear, 0.1)

    def test_it_moves_on_when_it_is_close_to_an_intermediate_waypoint(self):
        self.tracker.set_path([(0.0, 0.0), (1.0, 0.0), (1.0, 5.0)])
        cmd = self.step(x=0.9)  # within 0.3 m of (1, 0): now heads for (1, 5)
        self.assertEqual(cmd.status, DRIVING)
        self.assertEqual(cmd.linear, 0.0)  # (1, 5) is far to the left: turn first
        self.assertGreater(cmd.angular, 0.0)

    def test_it_can_skip_several_waypoints_in_one_step(self):
        self.tracker.set_path([(0.0, 0.0), (0.1, 0.0), (0.2, 0.0), (5.0, 0.0)])
        cmd = self.step()
        self.assertEqual(cmd.status, DRIVING)
        self.assertGreater(cmd.linear, 0.0)

    # ---- arrival ----
    def test_it_arrives_inside_the_final_tolerance_and_then_goes_idle(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        cmd = self.step(x=5.0 - FINAL_TOL / 2)
        self.assertEqual((cmd.status, cmd.linear, cmd.angular), (ARRIVED, 0.0, 0.0))
        self.assertFalse(self.tracker.active)
        self.assertEqual(self.step().status, IDLE)

    def test_it_keeps_driving_just_outside_the_final_tolerance(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        self.assertEqual(self.step(x=5.0 - 1.5 * FINAL_TOL).status, DRIVING)

    def test_arrival_is_reported_only_once(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        statuses = [self.step(x=5.0).status for _ in range(3)]
        self.assertEqual(statuses, [ARRIVED, IDLE, IDLE])

    # ---- stale pose ----
    def test_a_stale_pose_stops_the_robot_but_keeps_the_path(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        cmd = self.step(age=0.6)
        self.assertEqual((cmd.status, cmd.linear, cmd.angular), (STALE, 0.0, 0.0))
        self.assertTrue(self.tracker.active)
        self.assertEqual(self.step(age=0.0).status, DRIVING)

    def test_a_pose_exactly_at_the_timeout_is_still_fresh(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        self.assertEqual(self.step(age=0.5).status, DRIVING)

    def test_a_pose_that_is_not_a_number_counts_as_stale(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        for bad in (
            {"x": math.nan},
            {"y": math.inf},
            {"yaw": math.nan},
            {"age": math.nan},
        ):
            self.assertEqual(self.step(**bad).status, STALE, bad)

    def test_a_stale_pose_does_not_complete_a_leg(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        self.assertEqual(self.step(x=5.0, age=9.0).status, STALE)
        self.assertTrue(self.tracker.active)

    # ---- cancel ----
    def test_cancel_drops_the_path_and_says_whether_a_leg_was_active(self):
        self.tracker.set_path([(0.0, 0.0), (5.0, 0.0)])
        self.assertTrue(self.tracker.cancel())
        self.assertFalse(self.tracker.active)
        self.assertEqual(self.step().status, IDLE)
        self.assertFalse(self.tracker.cancel())

    # ---- threads ----
    def test_replacing_the_path_while_stepping_never_breaks_the_step(self):
        # Every waypoint is at the robot, so each step walks the whole index.
        # A step that read waypoints and index separately could index past the end
        # of a shorter path that was swapped in meanwhile.
        long_path = [(0.0, 0.0)] * 50
        short_path = [(0.0, 0.0)] * 2
        stop = threading.Event()

        def writer():
            use_short = False
            while not stop.is_set():
                self.tracker.set_path(short_path if use_short else long_path)
                use_short = not use_short

        old_interval = sys.getswitchinterval()
        sys.setswitchinterval(1e-6)  # let the two threads interleave inside a step
        thread = threading.Thread(target=writer)
        thread.start()
        try:
            for _ in range(20000):
                self.step()
        finally:
            stop.set()
            thread.join()
            sys.setswitchinterval(old_interval)


if __name__ == "__main__":
    unittest.main()
