"""Scenario spec (ADR-019): reset message, seeded offsets, worker path maths, yaml validation, and the Unity JSON."""
import copy
import math
import os
import unittest

from warehouse_eval import scenario_spec as spec

REPO = spec.REPO_DIR
DYNAMIC_IDS = ["S-04", "S-00c", "S-06", "S-07", "S-07b", "S-08", "S-09", "S-10", "S-11"]


def worker(**kw):
    w = {
        "name": "w", "speed_mps": 0.5, "radius_m": 0.15, "mode": "once", "start_trigger": "robot_departs",
        "start_delay_s": 0.0, "start_jitter_s": 0.0,
        "waypoints": [{"x": 0.0, "y": 0.0, "dwell_s": 0.0}, {"x": 3.0, "y": 0.0, "dwell_s": 0.0}],
    }
    w.update(kw)
    return w


class ResetMessageTest(unittest.TestCase):
    def test_a_plain_id_has_no_seed(self):
        self.assertEqual(spec.parse_reset_id("S-04"), ("S-04", None))
        self.assertEqual(spec.parse_reset_id("  S-04\n"), ("S-04", None))

    def test_a_seeded_id(self):
        self.assertEqual(spec.parse_reset_id("S-06?seed=7"), ("S-06", 7))
        self.assertEqual(spec.parse_reset_id("S-00c?seed=0"), ("S-00c", 0))

    def test_malformed_messages_are_rejected(self):
        for bad in ("S-04?seed=-1", "S-04?seed=", "S-04?seed=x", "?seed=3", "S-04?x=1", "S-04?seed=1&seed=2", "S-04?"):
            with self.assertRaises(ValueError, msg=bad):
                spec.parse_reset_id(bad)

    def test_without_a_seed_the_message_is_the_old_bare_id(self):
        self.assertEqual(spec.reset_message("S-04", None), "S-04")
        self.assertEqual(spec.reset_message("S-06", 7), "S-06?seed=7")
        self.assertEqual(spec.parse_reset_id(spec.reset_message("S-06", 12)), ("S-06", 12))
        with self.assertRaises(ValueError):
            spec.reset_message("S-06", -1)


class SeededUnitTest(unittest.TestCase):
    def test_pinned_values_match_the_csharp_test_vectors(self):
        # The same numbers are asserted in WarehouseProjectURP/Assets/Tests/EditMode/NpcPathTests.cs
        for seed, salt, expected in [(0, 0, 0.330931), (7, 0, 0.27441), (7, 1, 0.352948), (1, 0, 0.479139),
                                     (2 ** 31 - 1, 5, 0.875656)]:
            self.assertAlmostEqual(spec.seeded_unit(seed, salt), expected, places=6)

    def test_range_and_spread(self):
        values = [spec.seeded_unit(s, 0) for s in range(200)]
        self.assertTrue(all(0.0 <= v < 1.0 for v in values))
        self.assertGreater(len(set(values)), 195)
        self.assertGreater(max(values) - min(values), 0.9)

    def test_two_workers_of_one_run_get_different_offsets(self):
        self.assertNotEqual(spec.seeded_unit(7, 0), spec.seeded_unit(7, 1))

    def test_start_delay_is_base_plus_jitter_share(self):
        w = worker(start_delay_s=2.0, start_jitter_s=4.0)
        self.assertAlmostEqual(spec.worker_start_delay_s(w, 7, 0), 2.0 + 4.0 * 0.27441, places=5)
        self.assertEqual(spec.worker_start_delay_s(worker(start_delay_s=1.5), 99, 3), 1.5)


class PositionAtTest(unittest.TestCase):
    def setUp(self):
        self.export = {s["id"]: s for s in spec.export(spec.load_config())["scenarios"]}

    def test_a_once_worker_walks_then_holds_the_last_waypoint(self):
        w = worker()
        self.assertEqual(spec.position_at(w, 0.0), (0.0, 0.0))
        self.assertAlmostEqual(spec.position_at(w, 3.0)[0], 1.5)
        self.assertEqual(spec.position_at(w, 6.0), (3.0, 0.0))
        self.assertEqual(spec.position_at(w, 600.0), (3.0, 0.0))
        self.assertEqual(spec.cycle_duration_s(w), 6.0)

    def test_negative_time_is_the_first_waypoint(self):
        self.assertEqual(spec.position_at(worker(), -5.0), (0.0, 0.0))

    def test_dwell_holds_the_worker_then_it_walks_on(self):
        w = worker(waypoints=[{"x": 0.0, "y": 0.0, "dwell_s": 0.0}, {"x": 3.0, "y": 0.0, "dwell_s": 4.0},
                              {"x": 3.0, "y": 3.0, "dwell_s": 0.0}])
        self.assertEqual(spec.position_at(w, 6.0), (3.0, 0.0))     # arrived at 6 s
        self.assertEqual(spec.position_at(w, 9.99), (3.0, 0.0))    # waits until 10 s
        self.assertAlmostEqual(spec.position_at(w, 13.0)[1], 1.5)  # 3 s into the last leg
        self.assertEqual(spec.position_at(w, 16.0), (3.0, 3.0))

    def test_a_forever_dwell_never_leaves(self):
        w = worker(waypoints=[{"x": 0.0, "y": 0.0, "dwell_s": 0.0}, {"x": 3.0, "y": 0.0, "dwell_s": -1.0},
                              {"x": 3.0, "y": 3.0, "dwell_s": 0.0}])
        self.assertEqual(spec.position_at(w, 1e6), (3.0, 0.0))
        self.assertTrue(math.isinf(spec.cycle_duration_s(w)))

    def test_pingpong_goes_there_and_back(self):
        w = worker(mode="pingpong")
        self.assertEqual(spec.cycle_duration_s(w), 12.0)
        self.assertEqual(spec.position_at(w, 6.0), (3.0, 0.0))
        self.assertAlmostEqual(spec.position_at(w, 9.0)[0], 1.5)
        self.assertAlmostEqual(spec.position_at(w, 12.0)[0], 0.0)
        self.assertAlmostEqual(spec.position_at(w, 15.0)[0], 1.5)

    def test_pingpong_with_three_waypoints_revisits_the_middle(self):
        w = worker(mode="pingpong", waypoints=[{"x": 0.0, "y": 0.0, "dwell_s": 0.0}, {"x": 1.0, "y": 0.0, "dwell_s": 0.0},
                                              {"x": 1.0, "y": 1.0, "dwell_s": 0.0}])
        # 0->1 (2 s), 1->(1,1) (2 s), back to (1,0) (2 s), then to 0 (2 s): 8 s
        self.assertEqual(spec.cycle_duration_s(w), 8.0)
        self.assertEqual(spec.position_at(w, 4.0), (1.0, 1.0))
        self.assertEqual(spec.position_at(w, 6.0), (1.0, 0.0))
        self.assertAlmostEqual(spec.position_at(w, 7.0)[0], 0.5)

    def test_s04_is_the_old_hardcoded_crossing_line(self):
        w = self.export["S-04"]["workers"][0]
        # C#: Unity (-6,1,0) -> (6,1,0) at 0.5 m/s = 2 u/s. ROS x = Z/4 = 0, y = -X/4 = +1.5 -> -1.5, a 6 s leg.
        self.assertEqual(spec.position_at(w, 0.0), (0.0, 1.5))
        self.assertEqual(spec.position_at(w, 6.0), (0.0, -1.5))
        self.assertEqual(spec.position_at(w, 12.0), (0.0, 1.5))
        self.assertEqual(w["speed_mps"], 0.5)

    def test_s07_stops_for_six_seconds_in_the_route_and_s07b_stays(self):
        w = self.export["S-07"]["workers"][0]
        a, b = w["waypoints"][0], w["waypoints"][1]
        t_arrive = math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / w["speed_mps"]
        self.assertAlmostEqual(t_arrive, 3.0, places=2)
        for dt in (0.0, 3.0, 5.9):
            x, y = spec.position_at(w, t_arrive + dt)
            self.assertAlmostEqual(math.hypot(x - b["x"], y - b["y"]), 0.0, places=6)
        x, y = spec.position_at(w, t_arrive + 6.5)
        self.assertGreater(math.hypot(x - b["x"], y - b["y"]), 0.2)
        stay = self.export["S-07b"]["workers"][0]
        self.assertEqual(spec.position_at(stay, 1e5), spec.position_at(stay, 1e3))

    def test_s11_is_twice_as_fast_as_a_walker_and_s10_slower(self):
        self.assertEqual(self.export["S-11"]["workers"][0]["speed_mps"], 1.0)
        self.assertEqual(self.export["S-10"]["workers"][0]["speed_mps"], 0.2)


class ResolveTest(unittest.TestCase):
    def setUp(self):
        self.cfg = spec.load_config()
        (self.sx, self.sy), (self.gx, self.gy) = spec.route_of(self.cfg)

    def test_f_zero_and_one_are_start_and_goal(self):
        a = spec.resolve_waypoint(self.cfg, {"f": 0.0})
        b = spec.resolve_waypoint(self.cfg, {"f": 1.0})
        self.assertAlmostEqual(a["x"], self.sx, places=3)
        self.assertAlmostEqual(a["y"], self.sy, places=3)
        self.assertAlmostEqual(b["x"], self.gx, places=3)
        self.assertAlmostEqual(b["y"], self.gy, places=3)

    def test_offset_is_perpendicular_and_to_the_left_of_the_travel_direction(self):
        dx, dy = self.gx - self.sx, self.gy - self.sy
        mid = spec.resolve_waypoint(self.cfg, {"f": 0.5})
        left = spec.resolve_waypoint(self.cfg, {"f": 0.5, "offset_m": 2.0})
        vx, vy = left["x"] - mid["x"], left["y"] - mid["y"]
        self.assertAlmostEqual(math.hypot(vx, vy), 2.0, places=3)
        self.assertAlmostEqual(vx * dx + vy * dy, 0.0, places=2)
        self.assertGreater(dx * vy - dy * vx, 0.0)  # cross(direction, offset) > 0: counter-clockwise = left

    def test_absolute_waypoints_pass_through_and_dwell_defaults_to_zero(self):
        self.assertEqual(spec.resolve_waypoint(self.cfg, {"x": 1.0, "y": -2.0}), {"x": 1.0, "y": -2.0, "dwell_s": 0.0})

    def test_a_waypoint_needs_a_position(self):
        with self.assertRaises(ValueError):
            spec.resolve_waypoint(self.cfg, {"dwell_s": 1.0})


class ValidateTest(unittest.TestCase):
    def cfg_with(self, **worker_kw):
        cfg = copy.deepcopy(spec.load_config())
        cfg["scenarios"]["S-06"]["workers"] = [dict(cfg["scenarios"]["S-06"]["workers"][0], **worker_kw)]
        return cfg

    def test_the_shipped_file_is_valid(self):
        self.assertEqual(spec.validate(spec.load_config()), [])

    def test_bad_values_are_reported(self):
        cases = [
            {"speed_mps": 0.0}, {"speed_mps": 5.0}, {"radius_m": 0.0}, {"mode": "loop"}, {"start_trigger": "never"},
            {"start_jitter_s": -1.0}, {"waypoints": [{"x": 0.0, "y": 0.0}]}, {"waypoints": [{"x": 0.0, "y": 0.0}, {"x": 99.0, "y": 0.0}]},
            {"mode": "pingpong", "waypoints": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0, "dwell_s": -1}]},
            {"waypoints": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0, "dwell_s": -3}]},
        ]
        for kw in cases:
            self.assertTrue(spec.validate(self.cfg_with(**kw)), msg=str(kw))

    def test_export_refuses_an_invalid_file(self):
        with self.assertRaises(ValueError):
            spec.export(self.cfg_with(speed_mps=0.0))

    def test_layout_checked_is_required(self):
        cfg = copy.deepcopy(spec.load_config())
        del cfg["scenarios"]["S-06"]["layout_checked"]
        self.assertTrue(any("layout_checked" in p for p in spec.validate(cfg)))

    def test_duplicate_worker_names_are_reported(self):
        cfg = copy.deepcopy(spec.load_config())
        cfg["scenarios"]["S-08"]["workers"][1]["name"] = cfg["scenarios"]["S-08"]["workers"][0]["name"]
        self.assertTrue(any("duplicate" in p for p in spec.validate(cfg)))


class ExportTest(unittest.TestCase):
    def setUp(self):
        self.cfg = spec.load_config()
        self.out = spec.export(self.cfg)

    def test_only_spec_scenarios_are_exported(self):
        ids = [s["id"] for s in self.out["scenarios"]]
        self.assertEqual(ids, DYNAMIC_IDS)
        for static in ("S-00", "S-01", "S-02", "S-03", "S-05"):
            self.assertNotIn(static, ids)
            self.assertFalse(spec.is_spec_scenario(self.cfg, static))

    def test_the_control_has_no_workers_and_s08_has_two(self):
        by_id = {s["id"]: s for s in self.out["scenarios"]}
        self.assertEqual(by_id["S-00c"]["workers"], [])
        self.assertEqual(len(by_id["S-08"]["workers"]), 2)

    def test_the_goal_defaults_to_the_route_goal(self):
        for sid in DYNAMIC_IDS:
            self.assertEqual(spec.scenario_goal(self.cfg, sid), (-3.75, -3.13))

    def test_every_dynamic_scenario_is_in_the_test_plan_twice(self):
        with open(os.path.join(REPO, "docs", "test-plan.md"), "r", encoding="utf-8") as f:
            plan = f.read()
        for sid in DYNAMIC_IDS:
            self.assertGreaterEqual(plan.count("| **%s** |" % sid) + plan.count("| %s |" % sid), 2, msg=sid)

    def test_scenarios_json_is_up_to_date(self):
        with open(spec.DEFAULT_JSON, "r", encoding="utf-8", newline="") as f:
            self.assertEqual(f.read().replace("\r\n", "\n"), spec.dumps(self.out),
                             "run: python3 -m warehouse_eval.scenario_spec --write")

    def test_the_json_has_the_schema_unity_reads(self):
        self.assertEqual(self.out["schema"], 1)
        w = self.out["scenarios"][2]["workers"][0]
        self.assertEqual(sorted(w), ["mode", "name", "radius_m", "speed_mps", "start_delay_s", "start_jitter_s",
                                     "start_trigger", "waypoints"])
        self.assertEqual(sorted(w["waypoints"][0]), ["dwell_s", "x", "y"])


if __name__ == "__main__":
    unittest.main()
