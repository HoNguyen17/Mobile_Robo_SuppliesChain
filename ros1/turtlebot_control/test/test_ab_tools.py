"""Unit tests for A/B testing tools (ab_run.py and ab_report.py).

Pure Python, 100% ROS-free. Runs without ROS or Unity.
Tests parsers, source string integrity, metric aggregations, fake ROS driver,
AST node structure, motion-quality metrics, settle retry self-healing,
and report generation (Markdown & self-contained HTML).
Python 3.8 compatible.
"""

import ast
import contextlib
import csv
import html
import html.parser
import json
import math
import os
import sys
import tempfile
import types
import unittest
import unittest.mock
from typing import Any, Dict, List, Optional, Sequence, Tuple

import support  # noqa: F401 (adds scripts/ to sys.path)
import ab_report
import ab_run


# ---------------- 1. Parser & Source Integrity Tests ----------------

class TestPlannedParsers(unittest.TestCase):
    def test_parse_astar_planned_real_format(self):
        text = "Planned: 5 corners, 12.4 m, 38 ms, inflation 0.35 m (0 dynamic obstacles)"
        res = ab_run.parse_planned_line(text)
        self.assertIsNotNone(res)
        self.assertFalse(res["is_dstar"])
        self.assertEqual(res["corners"], 5)
        self.assertAlmostEqual(res["path_len_m"], 12.4)
        self.assertAlmostEqual(res["plan_ms"], 38.0)
        self.assertAlmostEqual(res["inflation_m"], 0.35)
        self.assertEqual(res["dynamic_obs"], 0)
        self.assertIsNone(res["rebuilt"])
        self.assertIsNone(res["expanded"])
        self.assertIsNone(res["level"])

    def test_parse_dstar_planned_real_format(self):
        text = (
            "Planned: 7 corners, 9.4 m, 85 ms, inflation 0.35 m (12 dynamic obstacles), "
            "expanded=150, rebuilt=True, level=0, search=40 ms, changed=10, set_obs=2 ms, "
            "costmap_inflate=25 ms, costmap_pub=5 ms, path_pub=3 ms, path_ready=45 ms, "
            "cpu=38 ms, build=15 ms, sync=5 ms, cells=2 ms, compute=18 ms, path=3 ms, "
            "format=2 ms, raw=+12/-0"
        )
        res = ab_run.parse_planned_line(text)
        self.assertIsNotNone(res)
        self.assertTrue(res["is_dstar"])
        self.assertEqual(res["corners"], 7)
        self.assertAlmostEqual(res["path_len_m"], 9.4)
        self.assertAlmostEqual(res["plan_ms"], 85.0)
        self.assertAlmostEqual(res["inflation_m"], 0.35)
        self.assertEqual(res["dynamic_obs"], 12)
        self.assertEqual(res["expanded"], 150)
        self.assertTrue(res["rebuilt"])
        self.assertEqual(res["level"], 0)
        self.assertAlmostEqual(res["search_ms"], 40.0)
        self.assertAlmostEqual(res["path_ready_ms"], 45.0)

    def test_parse_dstar_with_rebuilt_false(self):
        text = (
            "Planned: 4 corners, 6.1 m, 18 ms, inflation 0.40 m (3 dynamic obstacles), "
            "expanded=0, rebuilt=False, level=1, search=8 ms, changed=2, set_obs=1 ms, "
            "costmap_inflate=5 ms, costmap_pub=2 ms, path_pub=1 ms, path_ready=10 ms, "
            "cpu=7 ms, build=0 ms, sync=1 ms, cells=1 ms, compute=4 ms, path=1 ms, "
            "format=1 ms, raw=+1/-1"
        )
        res = ab_run.parse_planned_line(text)
        self.assertIsNotNone(res)
        self.assertTrue(res["is_dstar"])
        self.assertEqual(res["expanded"], 0)
        self.assertFalse(res["rebuilt"])
        self.assertEqual(res["level"], 1)

    def test_parse_real_planned_lines_from_bringup_logs(self):
        """W4: Quote one real Planned line from bringup_astar.log and one from bringup_dstar.log."""
        # Real line from bringup_astar.log session 20261025_071408
        line_astar = "Planned: 4 corners, 9.1 m, 220 ms, inflation 0.40 m (574 dynamic obstacles)"
        res_a = ab_run.parse_planned_line(line_astar)
        self.assertIsNotNone(res_a)
        self.assertFalse(res_a["is_dstar"])
        self.assertEqual(res_a["corners"], 4)
        self.assertAlmostEqual(res_a["path_len_m"], 9.1)
        self.assertAlmostEqual(res_a["plan_ms"], 220.0)
        self.assertAlmostEqual(res_a["inflation_m"], 0.40)
        self.assertEqual(res_a["dynamic_obs"], 574)

        # Real line from bringup_dstar.log session 20261025_071408
        line_dstar = (
            "Planned: 4 corners, 9.0 m, 474 ms, inflation 0.40 m (43 dynamic obstacles), "
            "expanded=3834, rebuilt=True, level=0, search=464 ms, changed=0, set_obs=0 ms, "
            "costmap_inflate=0 ms, costmap_pub=6 ms, path_pub=1 ms, path_ready=465 ms, "
            "cpu=437 ms, build=180 ms, sync=0 ms, cells=0 ms, compute=279 ms, path=1 ms, "
            "format=4 ms, raw=+43/-0"
        )
        res_d = ab_run.parse_planned_line(line_dstar)
        self.assertIsNotNone(res_d)
        self.assertTrue(res_d["is_dstar"])
        self.assertEqual(res_d["corners"], 4)
        self.assertAlmostEqual(res_d["path_len_m"], 9.0)
        self.assertAlmostEqual(res_d["plan_ms"], 474.0)
        self.assertEqual(res_d["expanded"], 3834)
        self.assertTrue(res_d["rebuilt"])
        self.assertAlmostEqual(res_d["search_ms"], 464.0)
        self.assertAlmostEqual(res_d["path_ready_ms"], 465.0)

    def test_parse_early_replan_lines(self):
        line_astar = (
            "Dynamic obstacle detected blocking current planned path ahead! "
            "Triggering early replan..."
        )
        line_dstar = (
            "Dynamic obstacle detected blocking current planned path ahead! "
            "Triggering early replan... (check radius 0.33)"
        )
        self.assertTrue(ab_run.parse_early_replan_line(line_astar))
        self.assertTrue(ab_run.parse_early_replan_line(line_dstar))
        self.assertFalse(ab_run.parse_early_replan_line("Some other log message"))

    def test_planner_source_format_string_integrity(self):
        """Fails if format strings in astar_planner.py or dstar_planner.py change."""
        base_dir = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts")
        )
        astar_file = os.path.join(base_dir, "astar_planner.py")
        dstar_file = os.path.join(base_dir, "dstar_planner.py")

        with open(astar_file, "r", encoding="utf-8") as f:
            astar_src = f.read()
        with open(dstar_file, "r", encoding="utf-8") as f:
            dstar_src = f.read()

        # Check A* log strings
        expected_astar_planned = "Planned: %d corners, %.1f m, %.0f ms, inflation %.2f m (%d dynamic obstacles)"
        expected_astar_early = "Dynamic obstacle detected blocking current planned path ahead! Triggering early replan..."
        self.assertIn(
            expected_astar_planned,
            astar_src,
            "astar_planner.py 'Planned:' format string has changed!",
        )
        self.assertIn(
            expected_astar_early,
            astar_src,
            "astar_planner.py early replan format string has changed!",
        )

        # Check D* log strings
        expected_dstar_prefix = "Planned: %d corners, %.1f m, %.0f ms, inflation %.2f m (%d dynamic obstacles), expanded=%d, rebuilt=%s"
        expected_dstar_early = "Dynamic obstacle detected blocking current planned path ahead! Triggering early replan... (check radius %.2f)"
        self.assertIn(
            expected_dstar_prefix,
            dstar_src,
            "dstar_planner.py 'Planned:' format string has changed!",
        )
        self.assertIn(
            expected_dstar_early,
            dstar_src,
            "dstar_planner.py early replan format string has changed!",
        )


# ---------------- 2. Aggregation & Metrics Tests ----------------

class TestMetricsAggregation(unittest.TestCase):
    def test_driven_distance(self):
        poses = [(0.0, 0.0), (3.0, 4.0), (3.0, 6.0)]
        # (0,0) -> (3,4) is 5.0m; (3,4) -> (3,6) is 2.0m; total = 7.0m
        dist = ab_run.compute_driven_distance(poses)
        self.assertAlmostEqual(dist, 7.0)

        # Also test 4-element poses: (sim_time, x, y, yaw)
        poses_4 = [(0.0, 0.0, 0.0, 0.0), (1.0, 3.0, 4.0, 0.0), (2.0, 3.0, 6.0, 0.0)]
        dist_4 = ab_run.compute_driven_distance(poses_4)
        self.assertAlmostEqual(dist_4, 7.0)

    def test_displacement_and_path_ratio_synthetic(self):
        # 1. Straight line: (0, 0) -> (5, 0) -> (10, 0)
        straight_poses = [(0.0, 0.0, 0.0, 0.0), (1.0, 5.0, 0.0, 0.0), (2.0, 10.0, 0.0, 0.0)]
        disp_s = ab_run.compute_displacement(straight_poses, outcome="succeeded")
        self.assertAlmostEqual(disp_s, 10.0)
        driven_s = ab_run.compute_driven_distance(straight_poses)
        ratio_s = ab_run.compute_path_ratio(driven_s, disp_s)
        self.assertAlmostEqual(ratio_s, 1.0)

        # 2. Detour: (0, 0) -> (5, 5) -> (10, 0)
        detour_poses = [(0.0, 0.0, 0.0, 0.0), (1.0, 5.0, 5.0, 0.0), (2.0, 10.0, 0.0, 0.0)]
        disp_d = ab_run.compute_displacement(detour_poses, outcome="succeeded")
        self.assertAlmostEqual(disp_d, 10.0)
        driven_d = ab_run.compute_driven_distance(detour_poses)
        self.assertAlmostEqual(driven_d, 10.0 * math.sqrt(2))
        ratio_d = ab_run.compute_path_ratio(driven_d, disp_d)
        self.assertGreater(ratio_d, 1.0)
        self.assertAlmostEqual(ratio_d, math.sqrt(2))

        # 3. Non-succeeded outcomes: displacement is None, path_ratio is None
        self.assertIsNone(ab_run.compute_displacement(straight_poses, outcome="timeout"))
        self.assertIsNone(ab_run.compute_displacement(straight_poses, outcome="aborted"))
        self.assertIsNone(ab_run.compute_displacement(straight_poses, outcome="invalid"))
        self.assertIsNone(ab_run.compute_path_ratio(10.0, None))

        # 4. Empty or single pose
        self.assertIsNone(ab_run.compute_displacement([], outcome="succeeded"))
        self.assertAlmostEqual(ab_run.compute_displacement([(0.0, 1.0, 2.0)], outcome="succeeded"), 0.0)

    def test_plan_ms_total_and_plan_share_pct_synthetic(self):
        """W1/W4: Check plan_ms_total and plan_share_pct on synthetic Planned lines and edge cases."""
        # 3 plans: 220 ms, 180 ms, 100 ms -> sum = 500 ms (0.5 s)
        planned_logs = [
            {"plan_ms": 220.0, "is_dstar": False},
            {"plan_ms": 180.0, "is_dstar": False},
            {"plan_ms": 100.0, "is_dstar": False},
        ]
        stats = ab_run.aggregate_plan_stats(planned_logs)
        self.assertEqual(stats["plans"], 3)
        self.assertAlmostEqual(stats["plan_ms_total"], 500.0)

        # Succeeded run: sim_time = 40.0 s -> plan_share_pct = 100 * (500 / 1000) / 40.0 = 1.25%
        share = ab_run.compute_plan_share_pct(stats["plan_ms_total"], sim_time_to_goal_s=40.0, outcome="succeeded")
        self.assertEqual(share, 1.25)

        # Non-succeeded run: share must be None (blank in CSV)
        share_timeout = ab_run.compute_plan_share_pct(stats["plan_ms_total"], sim_time_to_goal_s=40.0, outcome="timeout")
        self.assertIsNone(share_timeout)

        share_aborted = ab_run.compute_plan_share_pct(stats["plan_ms_total"], sim_time_to_goal_s=40.0, outcome="aborted")
        self.assertIsNone(share_aborted)

        # Zero or negative sim time
        self.assertIsNone(ab_run.compute_plan_share_pct(stats["plan_ms_total"], sim_time_to_goal_s=0.0, outcome="succeeded"))

        # Empty planned_logs: blank strings
        empty_stats = ab_run.aggregate_plan_stats([])
        self.assertEqual(empty_stats["plans"], 0)
        self.assertEqual(empty_stats["plan_ms_total"], "")
        self.assertIsNone(ab_run.compute_plan_share_pct(empty_stats["plan_ms_total"], sim_time_to_goal_s=40.0, outcome="succeeded"))

    def test_stops_calculation(self):
        # 1. Driving smoothly at 0.25 m/s: 0 stops
        poses_driving = [
            (t, t * 0.25, 0.0, 0.0) for t in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
        ]
        self.assertEqual(ab_run.compute_stops(poses_driving, start_sim_time=0.0), 0)

        # 2. Driving, then stopping from t=2.0 to t=4.0 (duration 2.0s >= 1.5s), then moving
        poses_one_stop = [
            (0.0, 0.0, 0.0, 0.0),
            (1.0, 0.25, 0.0, 0.0),
            (1.5, 0.50, 0.0, 0.0),
            (2.0, 0.60, 0.0, 0.0),  # stop begins
            (2.5, 0.60, 0.0, 0.0),
            (3.0, 0.60, 0.0, 0.0),
            (3.5, 0.60, 0.0, 0.0),
            (4.0, 0.60, 0.0, 0.0),  # stopped 2.0s -> 1 stop
            (4.5, 0.80, 0.0, 0.0),  # moving again
            (5.0, 1.00, 0.0, 0.0),
        ]
        self.assertEqual(ab_run.compute_stops(poses_one_stop, start_sim_time=0.0), 1)

        # 3. Stop happening entirely within the first second (t <= 1.0s): ignored
        poses_early_stop = [
            (0.0, 0.0, 0.0, 0.0),
            (0.3, 0.0, 0.0, 0.0),
            (0.7, 0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0),
            (1.5, 0.2, 0.0, 0.0),  # moving after 1s
            (2.0, 0.4, 0.0, 0.0),
            (2.5, 0.6, 0.0, 0.0),
        ]
        self.assertEqual(ab_run.compute_stops(poses_early_stop, start_sim_time=0.0), 0)

        # 4. Brief stop of 1.0s (< 1.5s threshold): not counted
        poses_brief_stop = [
            (1.0, 0.0, 0.0, 0.0),
            (1.5, 0.1, 0.0, 0.0),
            (2.0, 0.2, 0.0, 0.0),  # moving
            (2.5, 0.2, 0.0, 0.0),  # stopped 0.5s
            (3.0, 0.2, 0.0, 0.0),  # stopped 1.0s total (< 1.5s)
            (3.5, 0.4, 0.0, 0.0),  # moving again
            (4.0, 0.6, 0.0, 0.0),
        ]
        self.assertEqual(ab_run.compute_stops(poses_brief_stop, start_sim_time=0.0), 0)

        # 5. 4 s of in-place rotation (speed=0, |yaw_rate|=0.5 >= 0.05): 0 stops
        poses_rotation = [
            (t, 0.0, 0.0, t * 0.5) for t in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
        ]
        self.assertEqual(ab_run.compute_stops(poses_rotation, start_sim_time=0.0), 0)

        # 6. 2 s of true standstill: 1 stop
        poses_standstill = [
            (t, 0.0, 0.0, 0.0) for t in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
        ]
        self.assertEqual(ab_run.compute_stops(poses_standstill, start_sim_time=0.0), 1)

    def test_min_clearance_calculation(self):
        # Range values: inf, nan, <0.12 (noise), and valid 0.40m
        scans = [
            [float("inf"), float("nan"), 0.05, 0.80, 0.65],
            [1.20, 0.40, 0.55],
            [0.70, 0.90],
        ]
        # Min valid range is 0.40m. Clearance = 0.40 - 0.257 = 0.143m
        clr = ab_run.compute_min_clearance(scans, footprint_radius=0.257)
        self.assertAlmostEqual(clr, 0.143, places=3)

    def test_percentile_helper(self):
        data = [10.0, 20.0, 30.0, 40.0, 50.0]
        self.assertAlmostEqual(ab_run.percentile(data, 50.0), 30.0)
        self.assertAlmostEqual(ab_run.percentile(data, 0.0), 10.0)
        self.assertAlmostEqual(ab_run.percentile(data, 100.0), 50.0)

    def test_plan_stats_aggregation_astar(self):
        plans = [
            {"plan_ms": 120.0, "is_dstar": False},
            {"plan_ms": 40.0, "is_dstar": False},
            {"plan_ms": 60.0, "is_dstar": False},
        ]
        stats = ab_run.aggregate_plan_stats(plans, is_dstar=False)
        self.assertEqual(stats["plans"], 3)
        self.assertEqual(stats["first_plan_ms"], 120.0)
        self.assertEqual(stats["first_plan_rebuilt"], "")
        self.assertEqual(stats["replan_ms_median"], 50.0)
        self.assertEqual(stats["replan_ms_max"], 60.0)
        self.assertEqual(stats["expanded_total"], "")
        self.assertEqual(stats["rebuilds"], "")

    def test_plan_stats_aggregation_dstar(self):
        plans = [
            {"plan_ms": 80.0, "is_dstar": True, "rebuilt": True, "expanded": 200, "level": 0},
            {"plan_ms": 15.0, "is_dstar": True, "rebuilt": False, "expanded": 10, "level": 0},
            {"plan_ms": 25.0, "is_dstar": True, "rebuilt": False, "expanded": 30, "level": 1},
        ]
        stats = ab_run.aggregate_plan_stats(plans, is_dstar=True)
        self.assertEqual(stats["plans"], 3)
        self.assertEqual(stats["first_plan_ms"], 80.0)
        self.assertEqual(stats["first_plan_rebuilt"], "True")
        self.assertEqual(stats["replan_ms_median"], 20.0)
        self.assertEqual(stats["expanded_total"], 240)
        self.assertEqual(stats["rebuilds"], 1)
        self.assertEqual(stats["level_max"], 1)

    # TASK E: Pure function tests for Motion Quality Metrics
    def test_point_to_segment_dist(self):
        # 1. Point projecting onto interior of segment (0,0)-(2,0)
        d1 = ab_run.point_to_segment_dist(1.0, 2.0, 0.0, 0.0, 2.0, 0.0)
        self.assertAlmostEqual(d1, 2.0)

        # 2. Point projecting before start of segment (0,0)-(2,0)
        d2 = ab_run.point_to_segment_dist(-1.0, 1.0, 0.0, 0.0, 2.0, 0.0)
        self.assertAlmostEqual(d2, math.hypot(-1.0, 1.0))

        # 3. Point projecting after end of segment (0,0)-(2,0)
        d3 = ab_run.point_to_segment_dist(3.0, 1.0, 0.0, 0.0, 2.0, 0.0)
        self.assertAlmostEqual(d3, math.hypot(1.0, 1.0))

        # 4. Degenerate zero-length segment (1,1)-(1,1)
        d4 = ab_run.point_to_segment_dist(4.0, 5.0, 1.0, 1.0, 1.0, 1.0)
        self.assertAlmostEqual(d4, math.hypot(3.0, 4.0))

    def test_point_to_polyline_dist_synthetic(self):
        # L-shaped polyline: (0,0) -> (10,0) -> (10,10)
        poly = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)]

        # Point along first segment
        self.assertAlmostEqual(ab_run.point_to_polyline_dist(5.0, 2.0, poly), 2.0)

        # Point along second segment
        self.assertAlmostEqual(ab_run.point_to_polyline_dist(12.0, 5.0, poly), 2.0)

        # Point on corner
        self.assertAlmostEqual(ab_run.point_to_polyline_dist(10.0, 0.0, poly), 0.0)

        # Point on end
        self.assertAlmostEqual(ab_run.point_to_polyline_dist(10.0, 10.0, poly), 0.0)

        # Empty polyline
        self.assertEqual(ab_run.point_to_polyline_dist(5.0, 5.0, []), 0.0)

        # Single point polyline
        self.assertAlmostEqual(ab_run.point_to_polyline_dist(4.0, 4.0, [(1.0, 0.0)]), 5.0)

    def test_compute_cross_track_errors(self):
        # Poses: (sim_time, x, y, yaw)
        poses = [
            (100.0, 0.0, 1.0, 0.0),   # dist to polyline is 1.0
            (101.0, 5.0, 2.0, 0.0),   # dist to polyline is 2.0
            (102.0, 8.0, 0.0, 0.0),   # dist to polyline is 0.0
        ]
        # Paths: [(sim_time, [(x, y), ...])]
        paths = [
            (100.0, [(0.0, 0.0), (10.0, 0.0)]),
        ]
        max_xte, mean_xte = ab_run.compute_cross_track_errors(poses, paths)
        self.assertIsNotNone(max_xte)
        self.assertIsNotNone(mean_xte)
        self.assertAlmostEqual(max_xte, 2.0)
        self.assertAlmostEqual(mean_xte, (1.0 + 2.0 + 0.0) / 3.0)

        # Empty inputs
        self.assertEqual(ab_run.compute_cross_track_errors([], paths), (None, None))
        self.assertEqual(ab_run.compute_cross_track_errors(poses, []), (None, None))

    def test_compute_heading_oscillations(self):
        # cmd_vels: list of (sim_time, lin_x, ang_z)
        # Sign transitions above 0.2:
        # +0.3 -> +0.4 (no change) -> -0.3 (change 1) -> -0.5 (no change) -> +0.3 (change 2) -> 0.0 (ignored) -> -0.4 (change 3)
        cmd_vels = [
            (100.0, 0.2, 0.3),
            (100.1, 0.2, 0.4),
            (100.2, 0.2, -0.3),
            (100.3, 0.2, -0.5),
            (100.4, 0.2, 0.3),
            (100.5, 0.2, 0.05),  # within [-0.2, 0.2], ignored
            (100.6, 0.2, -0.4),
        ]
        osc = ab_run.compute_heading_oscillations(cmd_vels, thresh=0.2)
        self.assertEqual(osc, 3)

        # All values below threshold
        sub_thresh = [(100.0, 0.2, 0.1), (100.1, 0.2, -0.15), (100.2, 0.2, 0.1)]
        self.assertEqual(ab_run.compute_heading_oscillations(sub_thresh, thresh=0.2), 0)

        # Monotonic positive
        pos_only = [(100.0, 0.2, 0.3), (100.1, 0.2, 0.5), (100.2, 0.2, 0.4)]
        self.assertEqual(ab_run.compute_heading_oscillations(pos_only, thresh=0.2), 0)


# ---------------- 3. Driver & FakeRospy Tests ----------------

class FakeRosTime:
    def __init__(self, t: float = 0.0) -> None:
        self._t = float(t)

    def to_sec(self) -> float:
        return self._t


class FakeRospy:
    def __init__(self, initial_sim_time: float = 100.0) -> None:
        self.sim_time = initial_sim_time
        self.publishers: Dict[str, List[Any]] = {}
        self.subscribers: Dict[str, Any] = {}
        self.publisher_connections: Dict[str, int] = {}
        self.on_sleep: Optional[Any] = None
        self._shutdown = False
        self.Time = self

    def now(self) -> FakeRosTime:
        return FakeRosTime(self.sim_time)

    def is_shutdown(self) -> bool:
        return self._shutdown

    def Rate(self, _hz: float):
        class RateObj:
            def __init__(rate_self, parent):
                rate_self.parent = parent
            def sleep(rate_self):
                rate_self.parent.sim_time += 0.05
                if rate_self.parent.on_sleep:
                    rate_self.parent.on_sleep()
        return RateObj(self)

    def Publisher(self, topic: str, _msg_cls: Any, queue_size: Any = None):
        if topic not in self.publishers:
            self.publishers[topic] = []

        class Pub:
            def __init__(pub_self, top, parent):
                pub_self.top = top
                pub_self.parent = parent
            def get_num_connections(pub_self):
                return pub_self.parent.publisher_connections.get(pub_self.top, 1)
            def publish(pub_self, msg):
                self.publishers[pub_self.top].append(msg)
                # Auto-dispatch logic for simulation hooks
                if pub_self.top == "/sim/reset_scenario":
                    if "/sim/ack" in self.subscribers:
                        ack_msg = types.SimpleNamespace(
                            data=json.dumps({"cmd": "reset_scenario", "ok": True, "error": ""})
                        )
                        self.subscribers["/sim/ack"](ack_msg)
                elif pub_self.top == "/move_base_simple/goal":
                    # Mock /planned_path dispatch
                    if "/planned_path" in self.subscribers:
                        p1 = types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=5.01, y=-0.05)))
                        p2 = types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0)))
                        path_cls = sys.modules.get("nav_msgs.msg", types.SimpleNamespace()).Path
                        if callable(path_cls):
                            path_msg = path_cls(poses=[p1, p2], stamp=self.sim_time)
                        else:
                            path_msg = types.SimpleNamespace(
                                header=types.SimpleNamespace(stamp=FakeRosTime(self.sim_time), frame_id="map"),
                                poses=[p1, p2]
                            )
                        self.subscribers["/planned_path"](path_msg)

                    # Mock /cmd_vel dispatch
                    if "/cmd_vel" in self.subscribers:
                        twist_cls = sys.modules.get("geometry_msgs.msg", types.SimpleNamespace()).Twist
                        if callable(twist_cls):
                            twist_msg = twist_cls(lin_x=0.2, ang_z=0.05)
                        else:
                            twist_msg = types.SimpleNamespace(
                                linear=types.SimpleNamespace(x=0.2, y=0.0, z=0.0),
                                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.05)
                            )
                        self.subscribers["/cmd_vel"](twist_msg)

                    # Mock /rosout and /nav/leg_result dispatch
                    if "/rosout" in self.subscribers:
                        log_msg = types.SimpleNamespace(
                            name="/dstar_planner",
                            msg="Planned: 5 corners, 10.0 m, 75 ms, inflation 0.35 m (0 dynamic obstacles), expanded=120, rebuilt=True, level=0, search=30 ms, changed=0, set_obs=1 ms, costmap_inflate=20 ms, costmap_pub=5 ms, path_pub=2 ms, path_ready=35 ms, cpu=30 ms, build=10 ms, sync=0 ms, cells=0 ms, compute=15 ms, path=2 ms, format=1 ms, raw=+0/-0"
                        )
                        self.subscribers["/rosout"](log_msg)
                    if "/nav/leg_result" in self.subscribers:
                        res_msg = types.SimpleNamespace(
                            data=json.dumps({"outcome": "succeeded", "reason": ""})
                        )
                        self.subscribers["/nav/leg_result"](res_msg)

        return Pub(topic, self)

    def Subscriber(self, topic: str, _msg_cls: Any, callback: Any) -> None:
        self.subscribers[topic] = callback

    def loginfo(self, _msg: str, *args: Any) -> None:
        pass

    def logwarn(self, _msg: str, *args: Any) -> None:
        pass

    def logwarn_throttle(self, _period: float, _msg: str, *args: Any) -> None:
        pass

    def logerr(self, _msg: str, *args: Any) -> None:
        pass


@contextlib.contextmanager
def fake_ros_environment(active_nodes: List[str]):
    fake_rospy = FakeRospy()
    modules_to_mock = {
        "rospy": fake_rospy,
        "rosnode": types.ModuleType("rosnode"),
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
        "rosgraph_msgs": types.ModuleType("rosgraph_msgs"),
        "rosgraph_msgs.msg": types.ModuleType("rosgraph_msgs.msg"),
        "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": types.ModuleType("nav_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
    }

    modules_to_mock["rosnode"].get_node_names = lambda: list(active_nodes)

    class PoseMsgStub:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=FakeRosTime(100.0), frame_id="map")
            self.pose = types.SimpleNamespace(
                position=types.SimpleNamespace(x=5.01, y=-0.05, z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            )

    class TwistStub:
        def __init__(self, lin_x: float = 0.0, ang_z: float = 0.0):
            self.linear = types.SimpleNamespace(x=lin_x, y=0.0, z=0.0)
            self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=ang_z)

    class PathStub:
        def __init__(self, poses=None, stamp: float = 0.0):
            self.header = types.SimpleNamespace(stamp=FakeRosTime(stamp), frame_id="map")
            self.poses = poses or []

    class EmptyStub:
        pass

    class StringStub:
        def __init__(self, data=""):
            self.data = data

    modules_to_mock["geometry_msgs.msg"].PoseStamped = PoseMsgStub
    modules_to_mock["geometry_msgs.msg"].Twist = TwistStub
    modules_to_mock["nav_msgs.msg"].Path = PathStub
    modules_to_mock["rosgraph_msgs.msg"].Clock = types.SimpleNamespace
    modules_to_mock["rosgraph_msgs.msg"].Log = types.SimpleNamespace
    modules_to_mock["nav_msgs.msg"].OccupancyGrid = types.SimpleNamespace
    modules_to_mock["sensor_msgs.msg"].LaserScan = types.SimpleNamespace
    modules_to_mock["std_msgs.msg"].Empty = EmptyStub
    modules_to_mock["std_msgs.msg"].String = StringStub

    with unittest.mock.patch.dict(sys.modules, modules_to_mock):
        yield fake_rospy, PoseMsgStub


class TestDriverLogic(unittest.TestCase):
    def test_planner_detection(self):
        self.assertEqual(ab_run.detect_planner(["/rosout", "/dstar_planner"]), ("dstar", "/dstar_planner"))
        self.assertEqual(ab_run.detect_planner(["/rosout", "/astar_planner"]), ("astar", "/astar_planner"))

        with self.assertRaises(RuntimeError):
            ab_run.detect_planner(["/rosout"])  # 0 planners

        with self.assertRaises(RuntimeError):
            ab_run.detect_planner(["/rosout", "/astar_planner", "/dstar_planner"])  # 2 planners

    def test_normal_successful_run(self):
        with fake_ros_environment(["/dstar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                def trigger_prereqs():
                    fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                    fake_rospy.subscribers["/map"](types.SimpleNamespace())
                    p = PoseMsgStub()
                    fake_rospy.subscribers["/robot/pose"](p)
                    fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0, 1.2]))

                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        trigger_prereqs()
                fake_rospy.Subscriber = hook_sub

                rows = ab_run.run_ab_experiment(
                    scenario="S-04",
                    goal_xy=(-3.75, -3.13),
                    repeats=1,
                    settle_s=0.1,
                    timeout_s=10.0,
                    out_dir=tmpdir,
                    custom_label="test_dstar",
                    rospy_mod=fake_rospy,
                )
                self.assertEqual(len(rows), 1)
                row = rows[0]
                self.assertEqual(row["outcome"], "succeeded")
                self.assertEqual(row["planner"], "dstar")
                self.assertEqual(row["scenario"], "S-04")
                self.assertEqual(row["plans"], 1)
                self.assertAlmostEqual(row["first_plan_ms"], 75.0)
                self.assertIn("displacement_m", row)
                self.assertEqual(row["settle_retry"], 0)
                self.assertIn("poses", json.loads(row["settle_diag"]))
                self.assertIn("/nav/cancel", fake_rospy.publishers)

    def test_invalid_pose_after_reset(self):
        with fake_ros_environment(["/dstar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        # Pose far away: (0.0, 0.0) vs (5.01, -0.05)
                        bad_pose = PoseMsgStub()
                        bad_pose.pose.position.x = 0.0
                        bad_pose.pose.position.y = 0.0
                        fake_rospy.subscribers["/robot/pose"](bad_pose)
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                rows = ab_run.run_ab_experiment(
                    scenario="S-04",
                    goal_xy=(-3.75, -3.13),
                    repeats=1,
                    settle_s=0.1,
                    timeout_s=5.0,
                    out_dir=tmpdir,
                    rospy_mod=fake_rospy,
                )
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["outcome"], "invalid")

    def test_missing_ack_marks_invalid(self):
        with fake_ros_environment(["/astar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_pub = fake_rospy.Publisher
                def no_ack_pub(top, cls, queue_size=1):
                    p = orig_pub(top, cls, queue_size)
                    p.publish = lambda msg: fake_rospy.publishers[top].append(msg)
                    return p
                fake_rospy.Publisher = no_ack_pub

                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                rows = ab_run.run_ab_experiment(
                    scenario="S-04",
                    goal_xy=(-3.75, -3.13),
                    repeats=1,
                    settle_s=0.1,
                    timeout_s=5.0,
                    out_dir=tmpdir,
                    rospy_mod=fake_rospy,
                )
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["outcome"], "invalid")
                self.assertEqual(rows[0]["invalid_reason"], "ack timeout")

    def test_sim_time_timeout(self):
        with fake_ros_environment(["/dstar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_pub = fake_rospy.Publisher
                def no_leg_pub(top, cls, queue_size=1):
                    p = orig_pub(top, cls, queue_size)
                    def custom_publish(msg):
                        fake_rospy.publishers[top].append(msg)
                        if top == "/sim/reset_scenario":
                            if "/sim/ack" in fake_rospy.subscribers:
                                fake_rospy.subscribers["/sim/ack"](
                                    types.SimpleNamespace(data=json.dumps({"cmd": "reset_scenario", "ok": True}))
                                )
                        elif top == "/move_base_simple/goal":
                            if "/rosout" in fake_rospy.subscribers:
                                fake_rospy.subscribers["/rosout"](
                                    types.SimpleNamespace(
                                        name="/dstar_planner",
                                        msg="Planned: 5 corners, 10.0 m, 75 ms, inflation 0.35 m (0 dynamic obstacles)"
                                    )
                                )
                    p.publish = custom_publish
                    return p
                fake_rospy.Publisher = no_leg_pub

                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                rows = ab_run.run_ab_experiment(
                    scenario="S-04",
                    goal_xy=(-3.75, -3.13),
                    repeats=1,
                    settle_s=0.05,
                    timeout_s=0.1,
                    out_dir=tmpdir,
                    rospy_mod=fake_rospy,
                )
                self.assertEqual(len(rows), 1)
                row = rows[0]
                self.assertEqual(row["outcome"], "timeout")
                # /nav/cancel published on timeout and at post-run cleanup
                self.assertGreaterEqual(len(fake_rospy.publishers.get("/nav/cancel", [])), 1)
                csv_file = os.path.join(tmpdir, "dstar_runs.csv")
                self.assertTrue(os.path.exists(csv_file))

    def test_late_leg_result_ignored(self):
        with fake_ros_environment(["/dstar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                orig_pub = fake_rospy.Publisher
                run_count = [0]
                def late_leg_pub(top, cls, queue_size=1):
                    p = orig_pub(top, cls, queue_size)
                    orig_publish = p.publish
                    def custom_publish(msg):
                        orig_publish(msg)
                        if top == "/sim/reset_scenario":
                            run_count[0] += 1
                            if run_count[0] == 2:
                                if "/nav/leg_result" in fake_rospy.subscribers:
                                    late_msg = types.SimpleNamespace(
                                        data=json.dumps({"outcome": "aborted", "reason": "late_stale_result"})
                                    )
                                    fake_rospy.subscribers["/nav/leg_result"](late_msg)
                    p.publish = custom_publish
                    return p
                fake_rospy.Publisher = late_leg_pub

                rows = ab_run.run_ab_experiment(
                    scenario="S-04",
                    goal_xy=(-3.75, -3.13),
                    repeats=2,
                    settle_s=0.05,
                    timeout_s=5.0,
                    out_dir=tmpdir,
                    rospy_mod=fake_rospy,
                )
                self.assertEqual(len(rows), 2)
                self.assertEqual(rows[0]["outcome"], "succeeded")
                self.assertEqual(rows[1]["outcome"], "succeeded")

    def test_invalid_reasons(self):
        # 1. ack not ok
        with fake_ros_environment(["/astar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_pub = fake_rospy.Publisher
                def bad_ack_pub(top, cls, queue_size=1):
                    p = orig_pub(top, cls, queue_size)
                    def custom_publish(msg):
                        fake_rospy.publishers[top].append(msg)
                        if top == "/sim/reset_scenario" and "/sim/ack" in fake_rospy.subscribers:
                            fake_rospy.subscribers["/sim/ack"](
                                types.SimpleNamespace(data=json.dumps({"cmd": "reset_scenario", "ok": False, "error": "sim error"}))
                            )
                    p.publish = custom_publish
                    return p
                fake_rospy.Publisher = bad_ack_pub
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub
                rows = ab_run.run_ab_experiment(
                    scenario="S-04", goal_xy=(-3.75, -3.13), repeats=1, settle_s=0.05, timeout_s=1.0,
                    out_dir=tmpdir, rospy_mod=fake_rospy,
                )
                self.assertEqual(rows[0]["outcome"], "invalid")
                self.assertEqual(rows[0]["invalid_reason"], "ack not ok")

        # 2. settle movement (persisting across retry)
        with fake_ros_environment(["/astar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                def move_during_settle():
                    if fake_rospy.sim_time >= 100.35:
                        bad_pose = PoseMsgStub()
                        bad_pose.header.stamp = FakeRosTime(fake_rospy.sim_time)
                        bad_pose.pose.position.x = 2.0
                        fake_rospy.subscribers["/robot/pose"](bad_pose)
                fake_rospy.on_sleep = move_during_settle

                rows = ab_run.run_ab_experiment(
                    scenario="S-04", goal_xy=(-3.75, -3.13), repeats=1, settle_s=0.4, timeout_s=1.0,
                    out_dir=tmpdir, rospy_mod=fake_rospy,
                )
                self.assertEqual(rows[0]["outcome"], "invalid")
                self.assertEqual(rows[0]["invalid_reason"], "settle movement")
                self.assertEqual(rows[0]["settle_retry"], 1)

        # 3. publisher not connected
        with fake_ros_environment(["/astar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                fake_rospy.publisher_connections["/sim/reset_scenario"] = 0
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                rows = ab_run.run_ab_experiment(
                    scenario="S-04", goal_xy=(-3.75, -3.13), repeats=1, settle_s=0.05, timeout_s=1.0,
                    out_dir=tmpdir, rospy_mod=fake_rospy,
                )
                self.assertEqual(rows[0]["outcome"], "invalid")
                self.assertEqual(rows[0]["invalid_reason"], "publisher not connected")

        # 4. planner node mismatch
        with fake_ros_environment(["/dstar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                orig_pub = fake_rospy.Publisher
                def mismatch_pub(top, cls, queue_size=1):
                    p = orig_pub(top, cls, queue_size)
                    def custom_publish(msg):
                        fake_rospy.publishers[top].append(msg)
                        if top == "/sim/reset_scenario":
                            if "/sim/ack" in fake_rospy.subscribers:
                                fake_rospy.subscribers["/sim/ack"](
                                    types.SimpleNamespace(data=json.dumps({"cmd": "reset_scenario", "ok": True}))
                                )
                        elif top == "/move_base_simple/goal":
                            if "/rosout" in fake_rospy.subscribers:
                                fake_rospy.subscribers["/rosout"](
                                    types.SimpleNamespace(
                                        name="/rogue_planner",
                                        msg="Planned: 5 corners, 10.0 m, 75 ms, inflation 0.35 m (0 dynamic obstacles)"
                                    )
                                )
                            if "/nav/leg_result" in fake_rospy.subscribers:
                                fake_rospy.subscribers["/nav/leg_result"](
                                    types.SimpleNamespace(data=json.dumps({"outcome": "succeeded", "reason": ""}))
                                )
                    p.publish = custom_publish
                    return p
                fake_rospy.Publisher = mismatch_pub

                rows = ab_run.run_ab_experiment(
                    scenario="S-04", goal_xy=(-3.75, -3.13), repeats=1, settle_s=0.05, timeout_s=1.0,
                    out_dir=tmpdir, rospy_mod=fake_rospy,
                )
                self.assertEqual(rows[0]["outcome"], "invalid")
                self.assertEqual(rows[0]["invalid_reason"], "planner node mismatch")

        # 5. no Planned line
        with fake_ros_environment(["/astar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                orig_pub = fake_rospy.Publisher
                def no_plan_pub(top, cls, queue_size=1):
                    p = orig_pub(top, cls, queue_size)
                    def custom_publish(msg):
                        fake_rospy.publishers[top].append(msg)
                        if top == "/sim/reset_scenario":
                            if "/sim/ack" in fake_rospy.subscribers:
                                fake_rospy.subscribers["/sim/ack"](
                                    types.SimpleNamespace(data=json.dumps({"cmd": "reset_scenario", "ok": True}))
                                )
                        elif top == "/move_base_simple/goal":
                            if "/nav/leg_result" in fake_rospy.subscribers:
                                fake_rospy.subscribers["/nav/leg_result"](
                                    types.SimpleNamespace(data=json.dumps({"outcome": "succeeded", "reason": ""}))
                                )
                    p.publish = custom_publish
                    return p
                fake_rospy.Publisher = no_plan_pub

                rows = ab_run.run_ab_experiment(
                    scenario="S-04", goal_xy=(-3.75, -3.13), repeats=1, settle_s=0.05, timeout_s=1.0,
                    out_dir=tmpdir, rospy_mod=fake_rospy,
                )
                self.assertEqual(rows[0]["outcome"], "invalid")
                self.assertEqual(rows[0]["invalid_reason"], "no Planned line")

    def test_stale_pose_before_ack_plus_03_ignored(self):
        # 1. Stale pose stamped before ack_time + 0.3 must NOT invalidate the run
        with fake_ros_environment(["/dstar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                def inject_poses():
                    if fake_rospy.sim_time < 100.3:
                        # Stale pose far away
                        stale_pose = PoseMsgStub()
                        stale_pose.header.stamp = FakeRosTime(fake_rospy.sim_time)
                        stale_pose.pose.position.x = 0.0
                        stale_pose.pose.position.y = 0.0
                        fake_rospy.subscribers["/robot/pose"](stale_pose)
                    else:
                        # Valid start pose stamped after ack_time + 0.3
                        good_pose = PoseMsgStub()
                        good_pose.header.stamp = FakeRosTime(fake_rospy.sim_time)
                        good_pose.pose.position.x = 5.01
                        good_pose.pose.position.y = -0.05
                        fake_rospy.subscribers["/robot/pose"](good_pose)
                fake_rospy.on_sleep = inject_poses

                rows = ab_run.run_ab_experiment(
                    scenario="S-04", goal_xy=(-3.75, -3.13), repeats=1, settle_s=0.4, timeout_s=1.0,
                    out_dir=tmpdir, rospy_mod=fake_rospy,
                )
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["outcome"], "succeeded")

        # 2. Real move > 0.3 m after ack_time + 0.3 MUST invalidate the run
        with fake_ros_environment(["/dstar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                def inject_move_after():
                    if fake_rospy.sim_time >= 100.35:
                        moved_pose = PoseMsgStub()
                        moved_pose.header.stamp = FakeRosTime(fake_rospy.sim_time)
                        moved_pose.pose.position.x = 5.50  # > 0.3 m from start
                        moved_pose.pose.position.y = -0.05
                        fake_rospy.subscribers["/robot/pose"](moved_pose)
                fake_rospy.on_sleep = inject_move_after

                rows = ab_run.run_ab_experiment(
                    scenario="S-04", goal_xy=(-3.75, -3.13), repeats=1, settle_s=0.4, timeout_s=1.0,
                    out_dir=tmpdir, rospy_mod=fake_rospy,
                )
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["outcome"], "invalid")
                self.assertEqual(rows[0]["invalid_reason"], "settle movement")

    def test_header_mismatch_raises(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_path = os.path.join(tmpdir, "test.csv")
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["run_id", "outcome", "time"])
            with self.assertRaises(ValueError) as ctx:
                ab_run.write_csv_row(csv_path, {"run_id": "1", "outcome": "succeeded"})
            self.assertIn("CSV header mismatch", str(ctx.exception))
            self.assertIn("move the old CSV aside", str(ctx.exception))

            # Test run_ab_experiment exits before reset if CSV header mismatch
            with fake_ros_environment(["/astar_planner", "/rosout"]) as (fake_rospy, _):
                bad_csv = os.path.join(tmpdir, "astar_runs.csv")
                with open(bad_csv, "w", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow(["bad", "header"])
                with self.assertRaises(ValueError) as ctx:
                    ab_run.run_ab_experiment(
                        scenario="S-04", goal_xy=(-3.75, -3.13), repeats=1, settle_s=0.05, timeout_s=1.0,
                        out_dir=tmpdir, rospy_mod=fake_rospy,
                    )
                self.assertIn("CSV header mismatch", str(ctx.exception))
                self.assertIn("move the old CSV aside", str(ctx.exception))
                self.assertEqual(len(fake_rospy.publishers.get("/sim/reset_scenario", [])), 0)

    # TASK E: Settle retry self-healing tests
    def test_settle_retry_self_healing_success(self):
        """Move detected on attempt 1 -> runner retries once -> succeeds on attempt 2."""
        with fake_ros_environment(["/dstar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                # On attempt 1 (sim_time around 100.35), move robot > 0.3 m.
                # On attempt 2 (after cancel + 2.0s sim wait, sim_time > 102.5), keep robot at start.
                def inject_transient_move():
                    if 100.35 <= fake_rospy.sim_time < 101.5:
                        moved_pose = PoseMsgStub()
                        moved_pose.header.stamp = FakeRosTime(fake_rospy.sim_time)
                        moved_pose.pose.position.x = 2.0  # moved
                        moved_pose.pose.position.y = 0.0
                        fake_rospy.subscribers["/robot/pose"](moved_pose)
                    elif fake_rospy.sim_time >= 102.0:
                        good_pose = PoseMsgStub()
                        good_pose.header.stamp = FakeRosTime(fake_rospy.sim_time)
                        good_pose.pose.position.x = 5.01
                        good_pose.pose.position.y = -0.05
                        fake_rospy.subscribers["/robot/pose"](good_pose)

                fake_rospy.on_sleep = inject_transient_move

                rows = ab_run.run_ab_experiment(
                    scenario="S-04",
                    goal_xy=(-3.75, -3.13),
                    repeats=1,
                    settle_s=0.4,
                    timeout_s=5.0,
                    out_dir=tmpdir,
                    rospy_mod=fake_rospy,
                )
                self.assertEqual(len(rows), 1)
                row = rows[0]
                self.assertEqual(row["outcome"], "succeeded")
                self.assertEqual(row["settle_retry"], 1)
                diag = json.loads(row["settle_diag"])
                self.assertIn("attempt1", diag)
                self.assertIn("attempt2", diag)
                # /nav/cancel was published during retry and at post-run cleanup
                self.assertGreaterEqual(len(fake_rospy.publishers.get("/nav/cancel", [])), 2)

    def test_settle_retry_both_attempts_fail(self):
        """Move detected on both attempts -> row invalid 'settle movement' with both diags."""
        with fake_ros_environment(["/dstar_planner", "/rosout"]) as (fake_rospy, PoseMsgStub):
            with tempfile.TemporaryDirectory() as tmpdir:
                orig_sub = fake_rospy.Subscriber
                def hook_sub(top, cls, cb):
                    orig_sub(top, cls, cb)
                    if top == "/scan":
                        fake_rospy.subscribers["/clock"](types.SimpleNamespace())
                        fake_rospy.subscribers["/map"](types.SimpleNamespace())
                        fake_rospy.subscribers["/robot/pose"](PoseMsgStub())
                        fake_rospy.subscribers["/scan"](types.SimpleNamespace(ranges=[1.0]))
                fake_rospy.Subscriber = hook_sub

                # Continuously moved during settle
                def inject_persistent_move():
                    if fake_rospy.sim_time >= 100.35:
                        moved_pose = PoseMsgStub()
                        moved_pose.header.stamp = FakeRosTime(fake_rospy.sim_time)
                        moved_pose.pose.position.x = 2.0  # moved
                        moved_pose.pose.position.y = 0.0
                        fake_rospy.subscribers["/robot/pose"](moved_pose)

                fake_rospy.on_sleep = inject_persistent_move

                rows = ab_run.run_ab_experiment(
                    scenario="S-04",
                    goal_xy=(-3.75, -3.13),
                    repeats=1,
                    settle_s=0.4,
                    timeout_s=5.0,
                    out_dir=tmpdir,
                    rospy_mod=fake_rospy,
                )
                self.assertEqual(len(rows), 1)
                row = rows[0]
                self.assertEqual(row["outcome"], "invalid")
                self.assertEqual(row["invalid_reason"], "settle movement")
                self.assertEqual(row["settle_retry"], 1)
                diag = json.loads(row["settle_diag"])
                self.assertIn("attempt1", diag)
                self.assertIn("attempt2", diag)

    def test_ab_run_parse_args_defaults(self):
        """F1: Ensure ab_run.py parse_args defaults match Phase 3."""
        args = ab_run.parse_args([])
        self.assertEqual(args.scenario, "S-04")
        self.assertEqual(args.goal, [-3.75, -3.13])
        self.assertEqual(args.repeats, 5)
        self.assertEqual(args.settle, 7.0)
        self.assertEqual(args.timeout, 120.0)
        self.assertEqual(args.out, "~/ab_runs")
        self.assertIsNone(args.label)

    def test_settle_diag_downsampling(self):
        """F3: Verify settle_diag stores downsampled poses (<= 4/s) but checks every pose."""
        recorder = ab_run.RunRecorder()
        recorder.start_settle(ack_sim_time=100.0)

        # Feed 20 poses at 20 Hz (sim_time 100.00 to 100.95, dt=0.05)
        # At t=100.35 (which falls between 100.25 and 100.50, so skipped by downsampling),
        # inject a large position deviation > 0.30m
        for i in range(20):
            t = 100.0 + i * 0.05
            if abs(t - 100.35) < 1e-4:
                recorder.on_pose(x=2.0, y=-0.05, sim_time=t)
            else:
                recorder.on_pose(x=5.01, y=-0.05, sim_time=t)

        diag = recorder.get_settle_diag()
        poses = diag["poses"]
        # Exactly 4 or 5 poses stored over 1.0 second
        self.assertLessEqual(len(poses), 5)
        self.assertGreaterEqual(len(poses), 4)

        # The downsampled poses do not include 100.35
        stored_times = [p[0] for p in poses]
        self.assertNotIn(100.35, stored_times)

        # But the movement check evaluated every pose and caught the movement!
        self.assertTrue(recorder.settle_moved)


# ---------------- 4. AST Structure Tests ----------------

class TestASTStructure(unittest.TestCase):
    def test_ab_run_ast_subscriptions_before_loop(self):
        """Verify ab_run.py creates publishers/subscribers before entering the experiment loop."""
        file_path = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "ab_run.py")
        )
        with open(file_path, "r", encoding="utf-8") as f:
            tree = ast.parse(f.read(), filename=file_path)

        # 1. Verify rospy is not imported at module top-level
        for node in tree.body:
            if isinstance(node, ast.Import):
                for alias in node.names:
                    self.assertNotEqual(alias.name, "rospy", "rospy imported at module top-level in ab_run.py")
            elif isinstance(node, ast.ImportFrom):
                self.assertNotEqual(node.module, "rospy", "rospy imported at module top-level in ab_run.py")

        # 2. Inspect run_ab_experiment function
        func_node = None
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == "run_ab_experiment":
                func_node = node
                break
        self.assertIsNotNone(func_node, "run_ab_experiment function not found in ab_run.py")

        # Find the main for loop in run_ab_experiment
        main_loop = None
        sub_calls_before_loop = 0
        pub_calls_before_loop = 0

        for stmt in func_node.body:
            if isinstance(stmt, ast.For):
                main_loop = stmt
                break
            for child in ast.walk(stmt):
                if isinstance(child, ast.Call) and isinstance(child.func, ast.Attribute):
                    if child.func.attr == "Subscriber":
                        sub_calls_before_loop += 1
                    elif child.func.attr == "Publisher":
                        pub_calls_before_loop += 1

        self.assertIsNotNone(main_loop, "Per-run execution for-loop not found in run_ab_experiment")
        self.assertGreaterEqual(
            sub_calls_before_loop, 6, "Expected >= 6 Subscriber calls before main run loop"
        )
        self.assertGreaterEqual(
            pub_calls_before_loop, 3, "Expected >= 3 Publisher calls before main run loop"
        )


# ---------------- 5. Report Generation Tests ----------------

class StrictHTMLParser(html.parser.HTMLParser):
    def __init__(self):
        super().__init__()
        self.tags = []

    def handle_starttag(self, tag, attrs):
        if tag not in ("meta", "br", "hr", "rect", "path", "circle", "line"):
            self.tags.append(tag)

    def handle_endtag(self, tag):
        if self.tags and self.tags[-1] == tag:
            self.tags.pop()


class TestReportGeneration(unittest.TestCase):
    def test_report_on_synthetic_data(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_astar = os.path.join(tmpdir, "astar_runs.csv")
            csv_dstar = os.path.join(tmpdir, "dstar_runs.csv")

            # Sample A* runs (2 succeeded, 1 timeout, 1 invalid)
            astar_rows = [
                {
                    "run_id": "a1", "label": "astar", "planner": "astar", "scenario": "S-04",
                    "goal_x": -3.75, "goal_y": -3.13, "run_index": 1, "outcome": "succeeded",
                    "sim_time_to_goal_s": 25.0, "wall_time_s": 24.5, "driven_m": 8.0,
                    "straight_m": 7.0, "path_ratio": 1.14, "stops": 0, "collisions": 0,
                    "min_clearance_m": 0.15, "plans": 1, "early_replans": 0, "first_plan_ms": 100.0,
                    "first_plan_rebuilt": "", "replan_ms_median": "", "replan_ms_p95": "",
                    "replan_ms_max": "", "expanded_total": "", "rebuilds": "", "level_max": "",
                    "started_at": "", "ended_at": "", "settle_plans": 0, "invalid_reason": "",
                    "first_plan_path_ready_ms": "", "replan_path_ready_median_ms": "",
                    "replan_path_ready_p95_ms": "", "settle_diag": "{}", "settle_retry": 0,
                    "max_xte_m": 0.150, "mean_xte_m": 0.080, "heading_osc": 2, "path_switches": 1,
                },
                {
                    "run_id": "a2", "label": "astar", "planner": "astar", "scenario": "S-04",
                    "goal_x": -3.75, "goal_y": -3.13, "run_index": 2, "outcome": "succeeded",
                    "sim_time_to_goal_s": 35.0, "wall_time_s": 34.0, "driven_m": 8.4,
                    "straight_m": 7.0, "path_ratio": 1.20, "stops": 1, "collisions": 0,
                    "min_clearance_m": 0.17, "plans": 2, "early_replans": 1, "first_plan_ms": 110.0,
                    "first_plan_rebuilt": "", "replan_ms_median": 50.0, "replan_ms_p95": 50.0,
                    "replan_ms_max": 50.0, "expanded_total": "", "rebuilds": "", "level_max": "",
                    "started_at": "", "ended_at": "", "settle_plans": 0, "invalid_reason": "",
                    "first_plan_path_ready_ms": "", "replan_path_ready_median_ms": "",
                    "replan_path_ready_p95_ms": "", "settle_diag": "{}", "settle_retry": 0,
                    "max_xte_m": 0.180, "mean_xte_m": 0.090, "heading_osc": 3, "path_switches": 2,
                },
                {
                    "run_id": "a3", "label": "astar", "planner": "astar", "scenario": "S-04",
                    "goal_x": -3.75, "goal_y": -3.13, "run_index": 3, "outcome": "timeout",
                    "sim_time_to_goal_s": 120.0, "wall_time_s": 110.0, "driven_m": 15.0,
                    "straight_m": 7.0, "path_ratio": 2.14, "stops": 3, "collisions": 1,
                    "min_clearance_m": 0.12, "plans": 4, "early_replans": 2, "first_plan_ms": 105.0,
                    "first_plan_rebuilt": "", "replan_ms_median": 45.0, "replan_ms_p95": 48.0,
                    "replan_ms_max": 50.0, "expanded_total": "", "rebuilds": "", "level_max": "",
                    "started_at": "", "ended_at": "", "settle_plans": 0, "invalid_reason": "",
                    "first_plan_path_ready_ms": "", "replan_path_ready_median_ms": "",
                    "replan_path_ready_p95_ms": "", "settle_diag": "{}", "settle_retry": 0,
                    "max_xte_m": 0.350, "mean_xte_m": 0.200, "heading_osc": 8, "path_switches": 4,
                },
                {
                    "run_id": "a4", "label": "astar", "planner": "astar", "scenario": "S-04",
                    "goal_x": -3.75, "goal_y": -3.13, "run_index": 4, "outcome": "invalid",
                    "sim_time_to_goal_s": 0.0, "wall_time_s": 0.0, "driven_m": 0.0,
                    "straight_m": 0.0, "path_ratio": 1.0, "stops": 0, "collisions": 0,
                    "min_clearance_m": "", "plans": 0, "early_replans": 0, "first_plan_ms": "",
                    "first_plan_rebuilt": "", "replan_ms_median": "", "replan_ms_p95": "",
                    "replan_ms_max": "", "expanded_total": "", "rebuilds": "", "level_max": "",
                    "started_at": "", "ended_at": "", "settle_plans": 0, "invalid_reason": "settle movement",
                    "first_plan_path_ready_ms": "", "replan_path_ready_median_ms": "",
                    "replan_path_ready_p95_ms": "", "settle_diag": "{}", "settle_retry": 1,
                    "max_xte_m": "", "mean_xte_m": "", "heading_osc": "", "path_switches": "",
                },
            ]

            # Sample D* runs (2 succeeded, 1 aborted)
            dstar_rows = [
                {
                    "run_id": "d1", "label": "dstar", "planner": "dstar", "scenario": "S-04",
                    "goal_x": -3.75, "goal_y": -3.13, "run_index": 1, "outcome": "succeeded",
                    "sim_time_to_goal_s": 22.0, "wall_time_s": 21.0, "driven_m": 8.1,
                    "straight_m": 7.0, "path_ratio": 1.15, "stops": 0, "collisions": 0,
                    "min_clearance_m": 0.16, "plans": 2, "early_replans": 1, "first_plan_ms": 80.0,
                    "first_plan_rebuilt": "True", "replan_ms_median": 15.0, "replan_ms_p95": 15.0,
                    "replan_ms_max": 15.0, "expanded_total": 150, "rebuilds": 1, "level_max": 0,
                    "started_at": "", "ended_at": "", "settle_plans": 0, "invalid_reason": "",
                    "first_plan_path_ready_ms": 30.0, "replan_path_ready_median_ms": 10.0,
                    "replan_path_ready_p95_ms": 10.0, "settle_diag": "{}", "settle_retry": 0,
                    "max_xte_m": 0.120, "mean_xte_m": 0.060, "heading_osc": 1, "path_switches": 2,
                },
                {
                    "run_id": "d2", "label": "dstar", "planner": "dstar", "scenario": "S-04",
                    "goal_x": -3.75, "goal_y": -3.13, "run_index": 2, "outcome": "succeeded",
                    "sim_time_to_goal_s": 24.0, "wall_time_s": 23.0, "driven_m": 8.3,
                    "straight_m": 7.0, "path_ratio": 1.18, "stops": 0, "collisions": 0,
                    "min_clearance_m": 0.18, "plans": 3, "early_replans": 2, "first_plan_ms": 20.0,
                    "first_plan_rebuilt": "False", "replan_ms_median": 25.0, "replan_ms_p95": 25.0,
                    "replan_ms_max": 25.0, "expanded_total": 80, "rebuilds": 0, "level_max": 0,
                    "started_at": "", "ended_at": "", "settle_plans": 0, "invalid_reason": "",
                    "first_plan_path_ready_ms": 12.0, "replan_path_ready_median_ms": 15.0,
                    "replan_path_ready_p95_ms": 15.0, "settle_diag": "{}", "settle_retry": 0,
                    "max_xte_m": 0.140, "mean_xte_m": 0.070, "heading_osc": 1, "path_switches": 3,
                },
                {
                    "run_id": "d3", "label": "dstar", "planner": "dstar", "scenario": "S-04",
                    "goal_x": -3.75, "goal_y": -3.13, "run_index": 3, "outcome": "aborted",
                    "sim_time_to_goal_s": 50.0, "wall_time_s": 48.0, "driven_m": 6.0,
                    "straight_m": 7.0, "path_ratio": 1.30, "stops": 2, "collisions": 0,
                    "min_clearance_m": 0.14, "plans": 2, "early_replans": 1, "first_plan_ms": 70.0,
                    "first_plan_rebuilt": "True", "replan_ms_median": 20.0, "replan_ms_p95": 20.0,
                    "replan_ms_max": 20.0, "expanded_total": 100, "rebuilds": 1, "level_max": 0,
                    "started_at": "", "ended_at": "", "settle_plans": 0, "invalid_reason": "",
                    "first_plan_path_ready_ms": 25.0, "replan_path_ready_median_ms": 12.0,
                    "replan_path_ready_p95_ms": 12.0, "settle_diag": "{}", "settle_retry": 0,
                    "max_xte_m": 0.220, "mean_xte_m": 0.110, "heading_osc": 4, "path_switches": 2,
                },
            ]

            for r in astar_rows:
                ab_run.write_csv_row(csv_astar, r)
            for r in dstar_rows:
                ab_run.write_csv_row(csv_dstar, r)

            md_path, html_path = ab_report.generate_report_from_csvs([csv_astar, csv_dstar], tmpdir)

            self.assertTrue(os.path.exists(md_path))
            self.assertTrue(os.path.exists(html_path))

            with open(md_path, "r", encoding="utf-8") as f:
                md_text = f.read()

            with open(html_path, "r", encoding="utf-8") as f:
                html_text = f.read()

            # Verify Markdown contains expected values
            self.assertIn("Scenario: S-04", md_text)
            self.assertIn("Success Rate", md_text)
            self.assertIn("2/3", md_text)  # 2 succeeded out of 3 valid runs
            # Verify timeout and aborted runs do NOT alter the time-to-goal median
            self.assertIn("30.0 s", md_text)  # A* median time (25 + 35) / 2 = 30.0 (timeout run excluded!)
            self.assertIn("23.0 s", md_text)  # D* median time (22 + 24) / 2 = 23.0 (aborted run excluded!)

            # TASK D: Verify motion-quality metric rows and footnote 3 in Markdown
            self.assertIn("Max XTE", md_text)
            self.assertIn("Mean XTE", md_text)
            self.assertIn("Heading Oscillations", md_text)
            self.assertIn("Path Switches", md_text)
            self.assertIn("3. max_xte_m and mean_xte_m: distance from each recorded pose to the polyline", md_text)
            self.assertIn("displacement_m reflects actual endpoint distance", md_text)
            self.assertIn("First Plan ms (cold)", md_text)
            self.assertIn("First Plan ms (warm)", md_text)
            self.assertNotIn("First Plan ms (rebuilt=True)", md_text)
            self.assertIn("median of run medians; worst-run p95:", md_text)
            self.assertIn("worst-run max:", md_text)
            self.assertIn("4. Ratio (A*/D*) above 1 means D* has the lower value for time and count metrics.", md_text)
            self.assertIn("5. n per planner and [min, max] ranges: with n=5 treat differences inside the run-to-run spread as not established.", md_text)

            # TASK D: Verify motion-quality metric rows and footnote 3 in HTML
            self.assertIn("Max XTE", html_text)
            self.assertIn("Mean XTE", html_text)
            self.assertIn("Heading Oscillations", html_text)
            self.assertIn("Path Switches", html_text)
            self.assertIn("max_xte_m and mean_xte_m: distance from each recorded pose to the polyline", html_text)
            self.assertIn("displacement_m reflects actual endpoint distance", html_text)
            self.assertIn("First Plan ms (cold)", html_text)
            self.assertIn("First Plan ms (warm)", html_text)
            self.assertNotIn("First Plan ms (rebuilt=True)", html_text)
            self.assertIn("median of run medians; worst-run p95:", html_text)
            self.assertIn("worst-run max:", html_text)
            self.assertIn("Ratio (A*/D*) above 1 means D* has the lower value for time and count metrics.", html_text)
            self.assertIn("n per planner and [min, max] ranges: with n=5 treat differences inside the run-to-run spread as not established.", html_text)

            # Verify HTML parser doesn't crash
            parser = StrictHTMLParser()
            parser.feed(html_text)

            # Verify no external links exist in self-contained HTML
            self.assertNotIn("http://", html_text)
            self.assertNotIn("https://", html_text)

    def test_old_csv_without_planner_time_columns_renders_dash(self):
        """W2/W4: Old CSVs without plan_ms_total/plan_share_pct must still render with '-'."""
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_file = os.path.join(tmpdir, "astar_runs.csv")
            # Write old CSV without plan_ms_total or plan_share_pct
            with open(csv_file, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["run_id", "label", "planner", "scenario", "goal_x", "goal_y", "run_index", "outcome", "sim_time_to_goal_s"])
                writer.writerow(["r1", "astar", "astar", "S-04", "-3.75", "-3.13", "1", "succeeded", "40.0"])

            md_path, html_path = ab_report.generate_report_from_csvs([csv_file], tmpdir)
            with open(md_path, "r", encoding="utf-8") as f:
                md_text = f.read()
            with open(html_path, "r", encoding="utf-8") as f:
                html_text = f.read()

            self.assertIn("Planner time per run (s)", md_text)
            self.assertIn("Planner time share of run (%)", md_text)
            self.assertIn("| Planner time per run (s) | - | - | - |", md_text)
            self.assertIn("| Planner time share of run (%) | - | - | - |", md_text)

            self.assertIn("Planner time per run (s)", html_text)
            self.assertIn("Planner time share of run (%)", html_text)
            self.assertIn("<td>-</td>", html_text)

    def test_report_planner_time_rows_and_footnote(self):
        """W2/W4: Report includes rows 'Planner time per run (s)', 'Planner time share of run (%)' and footnote 6."""
        with tempfile.TemporaryDirectory() as tmpdir:
            csv_a = os.path.join(tmpdir, "astar_runs.csv")
            csv_d = os.path.join(tmpdir, "dstar_runs.csv")
            with open(csv_a, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["run_id", "label", "planner", "scenario", "outcome", "sim_time_to_goal_s", "plan_ms_total", "plan_share_pct"])
                writer.writerow(["a1", "astar", "astar", "S-04", "succeeded", "40.0", "3200.0", "8.00"])
            with open(csv_d, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["run_id", "label", "planner", "scenario", "outcome", "sim_time_to_goal_s", "plan_ms_total", "plan_share_pct"])
                writer.writerow(["d1", "dstar", "dstar", "S-04", "succeeded", "40.0", "640.0", "1.60"])

            md_path, html_path = ab_report.generate_report_from_csvs([csv_a, csv_d], tmpdir)
            with open(md_path, "r", encoding="utf-8") as f:
                md_text = f.read()
            with open(html_path, "r", encoding="utf-8") as f:
                html_text = f.read()

            # Rows in Markdown
            self.assertIn("| Planner time per run (s) | 3.20 s", md_text)
            self.assertIn("0.64 s", md_text)
            self.assertIn("5.00x", md_text)
            self.assertIn("| Planner time share of run (%) | 8.0 %", md_text)
            self.assertIn("1.6 %", md_text)

            # Footnote 6 in Markdown
            expected_fn = (
                "Sum of the ms timer of all Planned lines in the leg; "
                "only plans started by the dynamic-obstacle check run inside the /scan callback [verified in code: "
                "astar_planner.py line 181 'self.plan_for_goal(self.latest_goal)', "
                "dstar_planner.py line 234 'self.plan_for_goal(self.latest_goal)'], "
                "first plans, goal changes and lookahead replan requests run in other callbacks [verified in code: "
                "astar_planner.py line 228 'self.plan_for_goal(msg)', "
                "astar_planner.py line 233 'self.plan_for_goal(self.latest_goal)', "
                "dstar_planner.py line 294 'self.plan_for_goal(msg)', "
                "dstar_planner.py line 299 'self.plan_for_goal(self.latest_goal)']; "
                "a ROS Python node runs each subscriber callback in its own thread [verified in code: "
                "path_tracker.py line 3-4 'ROS callbacks run on their own threads'], "
                "so only the scan-triggered part delays scan processing."
            )
            self.assertEqual(ab_report.FOOTNOTE_PLANNER_TIME, expected_fn)
            self.assertIn("6. " + expected_fn, md_text)

            # HTML report
            self.assertIn("Planner time per run (s)", html_text)
            self.assertIn("Planner time share of run (%)", html_text)
            self.assertIn(html.escape(expected_fn), html_text)


if __name__ == "__main__":
    unittest.main()
