"""Unit tests for nav_common: leg result JSON and parameter sanitising."""
import json
import math
import unittest

import support  # noqa: F401  (puts scripts/ on sys.path)
from nav_common import ABORTED, CANCELLED, NO_PATH, SUCCEEDED, finite_or, leg_result


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


if __name__ == "__main__":
    unittest.main()
