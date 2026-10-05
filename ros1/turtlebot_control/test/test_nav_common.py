"""Unit tests for nav_common: leg result JSON and parameter sanitising."""
import json
import math
import unittest

import support  # noqa: F401  (puts scripts/ on sys.path)
from nav_common import (
    ABORTED,
    CANCELLED,
    NO_PATH,
    SUCCEEDED,
    ClockWatch,
    finite_or,
    leg_result,
    wait_for_clock,
)


class LegResultTest(unittest.TestCase):
    def test_a_success_has_an_empty_reason(self):
        self.assertEqual(
            json.loads(leg_result(SUCCEEDED)), {"outcome": "succeeded", "reason": ""}
        )

    def test_an_abort_carries_its_reason(self):
        self.assertEqual(
            json.loads(leg_result(ABORTED, NO_PATH)),
            {"outcome": "aborted", "reason": "no_path"},
        )
        cancelled = json.loads(leg_result(ABORTED, CANCELLED))
        self.assertEqual(cancelled["reason"], "cancelled")


class FiniteOrTest(unittest.TestCase):
    def test_a_number_at_or_above_the_minimum_is_kept(self):
        self.assertEqual(finite_or(0.2, 1.0, 0.0), 0.2)
        self.assertEqual(finite_or(0.0, 1.0, 0.0), 0.0)

    def test_a_numeric_string_is_converted(self):
        self.assertEqual(finite_or("0.25", 1.0, 0.0), 0.25)

    def test_values_that_are_not_numbers_give_the_default(self):
        for bad in (None, "fast", [], {}):
            self.assertEqual(finite_or(bad, 1.0, 0.0), 1.0, bad)

    def test_nan_and_infinity_give_the_default(self):
        for bad in (math.nan, math.inf, -math.inf):
            self.assertEqual(finite_or(bad, 1.0, 0.0), 1.0, bad)

    def test_a_value_below_the_minimum_gives_the_default(self):
        self.assertEqual(finite_or(-0.1, 1.0, 0.0), 1.0)


class WaitForClockTest(unittest.TestCase):
    """The simulation clock reads 0 until Unity's first /clock message arrives."""

    def test_returns_at_once_when_the_clock_is_already_running(self):
        sleeps = []
        self.assertTrue(wait_for_clock(lambda: 12.5, sleeps.append))
        self.assertEqual(sleeps, [])

    def test_polls_until_the_clock_starts(self):
        readings = iter([0.0, 0.0, 0.0, 5.0])
        sleeps = []
        self.assertTrue(wait_for_clock(lambda: next(readings), sleeps.append, poll=0.05))
        self.assertEqual(sleeps, [0.05, 0.05, 0.05])

    def test_gives_up_when_ros_shuts_down(self):
        calls = {"n": 0}

        def is_shutdown():
            calls["n"] += 1
            return calls["n"] > 3

        self.assertFalse(wait_for_clock(lambda: 0.0, lambda s: None, is_shutdown=is_shutdown))

    def test_reports_how_long_it_has_waited(self):
        readings = iter([0.0] * 80 + [1.0])
        waited = []
        wait_for_clock(
            lambda: next(readings), lambda s: None, on_wait=waited.append, poll=0.05, report_every=2.0
        )
        self.assertEqual(len(waited), 2)  # after 2 s and after 4 s
        self.assertAlmostEqual(waited[0], 2.0)
        self.assertAlmostEqual(waited[1], 4.0)


class ClockWatchTest(unittest.TestCase):
    """Unity pressed Play again: the simulation clock restarts from 0."""

    def test_a_clock_that_runs_forward_never_jumps_back(self):
        watch = ClockWatch(tolerance=1.0)
        self.assertFalse(any(watch.jumped_back(t) for t in (0.0, 0.5, 1.0, 50.0, 50.0, 51.0)))

    def test_the_first_reading_is_never_a_jump(self):
        self.assertFalse(ClockWatch().jumped_back(120.0))

    def test_a_small_step_back_is_ignored(self):
        watch = ClockWatch(tolerance=1.0)
        watch.jumped_back(100.0)
        self.assertFalse(watch.jumped_back(99.5))

    def test_a_big_step_back_is_reported_once(self):
        watch = ClockWatch(tolerance=1.0)
        watch.jumped_back(100.0)
        self.assertTrue(watch.jumped_back(2.0))
        self.assertFalse(watch.jumped_back(2.5))
        self.assertFalse(watch.jumped_back(3.0))


if __name__ == "__main__":
    unittest.main()
