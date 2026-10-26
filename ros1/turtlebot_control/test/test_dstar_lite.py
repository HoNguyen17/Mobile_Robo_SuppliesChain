import ast
import contextlib
import importlib.util
import math
import os
import random
import sys
import types
import unittest
import unittest.mock
from typing import Any

import support  # noqa: F401 (puts scripts/ on sys.path)
from dstar_lite import (
    DStarLite,
    DStarLitePlanner,
    IncrementalInflation,
    disc_offsets,
    expand_lidar_hit,
    path_near_points,
)
from grid_planner import (
    Grid,
    InflatedMaps,
    NoPath,
    SQRT2,
    astar,
    cell_center,
    cell_index,
    inflate,
    overlay_obstacles,
)
from support import blocked_cells, make_grid


def path_cost(path, width):
    """Compute octile distance along a sequence of cell indices."""
    if not path or len(path) < 2:
        return 0.0
    cost = 0.0
    for a, b in zip(path, path[1:]):
        ax, ay = a % width, a // width
        bx, by = b % width, b // width
        cost += SQRT2 if (ax != bx and ay != by) else 1.0
    return cost


class DiscOffsetsTest(unittest.TestCase):
    def test_matches_inflate_for_one_obstacle_across_all_radii(self):
        res = 0.05
        w, h = 50, 50
        oi, oj = 25, 25
        data = [0] * (w * h)
        data[oj * w + oi] = 100
        grid = Grid(data, w, h, res)

        radii = [0, 0.02, 0.05, 0.12, 0.28, 0.35, 0.40, 0.50]
        for r in radii:
            inf = inflate(grid, r)
            expected = blocked_cells(inf)
            offsets = disc_offsets(res, r)
            actual = {(oi + di, oj + dj) for di, dj in offsets}
            self.assertEqual(actual, expected, f"disc_offsets mismatch for radius={r}")


class IncrementalInflationTest(unittest.TestCase):
    def test_matches_inflate_overlay_obstacles_on_random_grids(self):
        rng = random.Random(42)
        for _ in range(20):
            w, h, res = 30, 20, 0.5
            raw_data = [100 if rng.random() < 0.10 else 0 for _ in range(w * h)]
            raw = Grid(raw_data, w, h, res)
            radius = 0.8
            static_inf = inflate(raw, radius)
            inc = IncrementalInflation(static_inf, radius)

            for _ in range(5):
                overlay_cells = [
                    (rng.randint(-5, w + 5), rng.randint(-5, h + 5))
                    for _ in range(15)
                ]
                inc.sync(overlay_cells)

                valid = [(i, j) for i, j in overlay_cells if 0 <= i < w and 0 <= j < h]
                pts = [cell_center(raw, j * w + i) for i, j in valid]
                expected_inf = inflate(overlay_obstacles(raw, pts), radius)

                inc_blocked = [1 if v else 0 for v in inc.view.data]
                exp_blocked = [1 if v else 0 for v in expected_inf.data]
                self.assertEqual(inc_blocked, exp_blocked)

    def test_matches_inflate_overlay_across_multiple_radii_and_resolutions(self):
        """IncrementalInflation vs inflate(overlay_obstacles(...)) across radii and resolutions."""
        radii_list = [0.0, 0.05, 0.12, 0.28, 0.35, 0.40]
        res = 0.05
        rng = random.Random(999)
        for radius in radii_list:
            w, h = 40, 30
            raw_data = [100 if rng.random() < 0.12 else 0 for _ in range(w * h)]
            raw = Grid(raw_data, w, h, res)
            static_inf = inflate(raw, radius)
            inc = IncrementalInflation(static_inf, radius)

            for _ in range(6):
                overlay_cells = [
                    (rng.randint(0, w - 1), rng.randint(0, h - 1))
                    for _ in range(rng.randint(3, 20))
                ]
                inc.sync(overlay_cells)

                valid = [(i, j) for i, j in overlay_cells if 0 <= i < w and 0 <= j < h]
                pts = [cell_center(raw, j * w + i) for i, j in valid]
                expected_inf = inflate(overlay_obstacles(raw, pts), radius)

                inc_blocked = [1 if v else 0 for v in inc.view.data]
                exp_blocked = [1 if v else 0 for v in expected_inf.data]
                self.assertEqual(
                    inc_blocked, exp_blocked,
                    f"Mismatch at radius={radius}, res={res}"
                )

    def test_returns_to_static_map_when_overlay_is_empty(self):
        w, h, res = 20, 20, 0.1
        data = [0] * (w * h)
        data[10 * w + 10] = 100
        raw = Grid(data, w, h, res)
        static_inf = inflate(raw, 0.35)
        inc = IncrementalInflation(static_inf, 0.35)

        inc.sync([(5, 5), (15, 15)])
        self.assertNotEqual(list(inc.view.data), list(static_inf.data))

        flipped = inc.sync([])
        self.assertGreater(len(flipped), 0)
        self.assertEqual(list(inc.view.data), list(static_inf.data))

    def test_reports_only_flipped_cells_and_ignores_cells_inside_static_obstacles(self):
        w, h, res = 20, 20, 0.1
        data = [0] * (w * h)
        data[10 * w + 10] = 100
        raw = Grid(data, w, h, res)
        static_inf = inflate(raw, 0.35)
        inc = IncrementalInflation(static_inf, 0.35)

        # (10, 10) is inside the static obstacle; adding it to overlay should flip zero cells
        flipped = inc.sync([(10, 10)])
        self.assertEqual(flipped, [])
        self.assertEqual(list(inc.view.data), list(static_inf.data))

    def test_ignores_out_of_grid_cells(self):
        w, h, res = 10, 10, 1.0
        static_inf = Grid([0] * (w * h), w, h, res)
        inc = IncrementalInflation(static_inf, 0.5)

        flipped = inc.sync([(-10, -5), (100, 200)])
        self.assertEqual(flipped, [])
        self.assertEqual(list(inc.view.data), list(static_inf.data))


class DStarLiteCostAgainstAStarTest(unittest.TestCase):
    def test_dstar_lite_path_cost_equals_astar_on_60_random_grids(self):
        rng = random.Random(12345)
        for trial in range(60):
            w, h = 20, 20
            density = rng.choice([0.05, 0.15, 0.25])
            data = [100 if rng.random() < density else 0 for _ in range(w * h)]
            s = rng.randint(0, w * h - 1)
            g = rng.randint(0, w * h - 1)
            data[s] = 0
            data[g] = 0
            grid = Grid(data, w, h, 1.0)

            astar_p = astar(grid, s, g)
            dsl = DStarLite(grid, s, g)
            dsl.compute()
            dsl_p = dsl.path_cells()

            if astar_p is None:
                self.assertIsNone(dsl_p, f"Trial {trial}: astar None but dsl found path")
            else:
                self.assertIsNotNone(dsl_p, f"Trial {trial}: astar found path but dsl returned None")
                ac = path_cost(astar_p, w)
                dc = path_cost(dsl_p, w)
                self.assertAlmostEqual(ac, dc, places=9, msg=f"Cost mismatch trial {trial}")

                # Check each step is legal: not blocked, no corner cutting
                for u, v in zip(dsl_p, dsl_p[1:]):
                    self.assertEqual(grid.data[v], 0)
                    ux, uy = u % w, u // w
                    vx, vy = v % w, v // w
                    if ux != vx and uy != vy:
                        self.assertEqual(grid.data[uy * w + vx], 0)
                        self.assertEqual(grid.data[vy * w + ux], 0)


class RandomizedRepairTest(unittest.TestCase):
    def test_randomized_repair_over_30_flips_and_start_moves_matches_astar(self):
        rng = random.Random(888)
        w, h = 25, 25
        data = bytearray(w * h)
        # Sparse obstacles so a path exists initially
        for i in range(w * h):
            if rng.random() < 0.15:
                data[i] = 100
        s = 0
        g = w * h - 1
        data[s] = 0
        data[g] = 0
        grid = Grid(data, w, h, 1.0)

        dsl = DStarLite(grid, s, g)
        dsl.compute()
        curr_path = dsl.path_cells()
        self.assertIsNotNone(curr_path, "Initial path should exist")

        for step in range(30):
            # Advance start along current path with 50% probability
            if curr_path and len(curr_path) > 2 and rng.random() < 0.5:
                s = curr_path[1]
                dsl.move_start(s)

            # Flip 1-4 random cells
            k = rng.randint(1, 4)
            flipped = []
            for _ in range(k):
                c = rng.randint(0, w * h - 1)
                if c != s and c != g:
                    data[c] = 0 if data[c] != 0 else 100
                    flipped.append(c)

            dsl.cells_changed(flipped)
            dsl.compute()
            curr_path = dsl.path_cells()

            astar_p = astar(grid, s, g)
            if astar_p is None:
                self.assertIsNone(curr_path, f"Step {step}: astar None but dsl path found")
            else:
                self.assertIsNotNone(curr_path, f"Step {step}: astar found path but dsl returned None")
                ac = path_cost(astar_p, w)
                dc = path_cost(curr_path, w)
                self.assertAlmostEqual(ac, dc, places=9, msg=f"Repair mismatch at step {step}")

    def test_randomized_repair_25_trials_with_varying_seeds(self):
        """25 independent trials, each with its own seed and a fresh 22x18 grid (density 0.2).

        30 steps each, flipping 1-4 cells per step (never start or goal),
        moving start along current path with 70% probability.
        After each step, compare path COST with astar.  'both None' counts as equal.
        """
        for trial in range(25):
            seed = 7000 + trial
            rng = random.Random(seed)
            w, h = 22, 18
            data = bytearray(w * h)
            for idx in range(w * h):
                if rng.random() < 0.2:
                    data[idx] = 100
            s = 0
            g = w * h - 1
            data[s] = 0
            data[g] = 0
            grid = Grid(data, w, h, 1.0)

            dsl = DStarLite(grid, s, g)
            dsl.compute()
            curr_path = dsl.path_cells()

            for step in range(30):
                # Move start along current path with 70% probability
                if curr_path and len(curr_path) > 2 and rng.random() < 0.7:
                    s = curr_path[1]
                    dsl.move_start(s)

                # Flip 1-4 random cells (never start or goal)
                k = rng.randint(1, 4)
                flipped = []
                for _ in range(k):
                    c = rng.randint(0, w * h - 1)
                    if c != s and c != g:
                        data[c] = 0 if data[c] != 0 else 100
                        flipped.append(c)

                dsl.cells_changed(flipped)
                dsl.compute()
                curr_path = dsl.path_cells()

                astar_p = astar(grid, s, g)
                if astar_p is None and curr_path is None:
                    # Both None: equal, no cost to compare
                    continue
                if astar_p is None:
                    self.fail(
                        f"Trial {trial} seed {seed} step {step}: "
                        f"astar None but dsl found path"
                    )
                if curr_path is None:
                    self.fail(
                        f"Trial {trial} seed {seed} step {step}: "
                        f"astar found path but dsl returned None"
                    )
                ac = path_cost(astar_p, w)
                dc = path_cost(curr_path, w)
                self.assertAlmostEqual(
                    ac, dc, places=9,
                    msg=f"Trial {trial} seed {seed} step {step}: cost mismatch"
                )

    def test_it_matches_astar_after_every_repair_on_open_grids_with_tied_costs(self):
        for seed in range(200):
            rng = random.Random(seed)
            w, h = 30, 30
            data = bytearray(w * h)
            grid = Grid(data, w, h, 1.0)
            s = rng.randint(0, w * h - 1)
            g = rng.randint(0, w * h - 1)
            while g == s:
                g = rng.randint(0, w * h - 1)

            dsl = DStarLite(grid, s, g)
            dsl.compute()
            path = dsl.path_cells()

            for step in range(12):
                k = rng.randint(1, 3)
                flipped = []
                while len(flipped) < k:
                    c = rng.randint(0, w * h - 1)
                    if c != s and c != g and c not in flipped:
                        data[c] = 0 if data[c] != 0 else 100
                        flipped.append(c)

                dsl.cells_changed(flipped)
                dsl.compute()
                path = dsl.path_cells()

                astar_p = astar(grid, s, g)
                if astar_p is None and path is None:
                    pass
                elif astar_p is None:
                    self.fail(f"Seed {seed} step {step}: astar None but dsl found path")
                elif path is None:
                    self.fail(f"Seed {seed} step {step}: astar found path but dsl returned None")
                else:
                    ac = path_cost(astar_p, w)
                    dc = path_cost(path, w)
                    self.assertAlmostEqual(
                        ac, dc, places=6,
                        msg=f"Seed {seed} step {step}: cost mismatch {ac} vs {dc}"
                    )

                if path and len(path) > 2 and rng.random() < 0.5:
                    s = path[1]
                    dsl.move_start(s)


class DStarLiteExpansionTest(unittest.TestCase):
    def test_expansion_counts_for_far_change_start_move_and_on_path_change(self):
        w, h = 40, 40
        data = bytearray(w * h)
        grid = Grid(data, w, h, 1.0)
        s = 2 * w + 2
        g = 38 * w + 38
        dsl = DStarLite(grid, s, g)
        dsl.compute()
        path = dsl.path_cells()
        self.assertIsNotNone(path)

        # 1. A change far from the best path -> expanded == 0
        far_cell = 2 * w + 38
        data[far_cell] = 100
        dsl.cells_changed([far_cell])
        before = dsl.expanded
        dsl.compute()
        self.assertEqual(dsl.expanded - before, 0)

        # 2. Start moving along the path -> expanded == 0
        next_s = path[1]
        dsl.move_start(next_s)
        before = dsl.expanded
        dsl.compute()
        self.assertEqual(dsl.expanded - before, 0)

        # 3. A change ON the best path -> expanded > 100 and cost still equals astar
        mid_cell = path[len(path) // 2]
        data[mid_cell] = 100
        dsl.cells_changed([mid_cell])
        before = dsl.expanded
        dsl.compute()
        new_path = dsl.path_cells()
        self.assertIsNotNone(new_path)
        self.assertGreater(dsl.expanded - before, 100)

        astar_p = astar(grid, next_s, g)
        self.assertAlmostEqual(
            path_cost(astar_p, w),
            path_cost(new_path, w),
            places=9,
        )


class DStarLitePlannerTest(unittest.TestCase):
    def setUp(self):
        rows = [".........."] * 10
        self.raw = make_grid(rows, res=0.1)
        self.maps = InflatedMaps(self.raw)
        self.planner = DStarLitePlanner(self.maps, widen_every=5.0)

    def test_path_starts_and_ends_where_asked(self):
        start = (0.15, 0.15)
        goal = (0.85, 0.85)
        path = self.planner.plan(start, goal, snap_distance=1.0, radii=[0.1])
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)

    def test_keeps_its_search_between_calls(self):
        start = (0.15, 0.15)
        goal = (0.85, 0.85)
        path1 = self.planner.plan(start, goal, snap_distance=1.0, radii=[0.1], now=0.0)
        self.assertTrue(self.planner.stats.rebuilt)
        first_exp = self.planner.stats.expanded

        # Move slightly along path
        start2 = path1[1] if len(path1) > 2 else (0.25, 0.25)
        self.planner.plan(start2, goal, snap_distance=1.0, radii=[0.1], now=0.5)
        self.assertFalse(self.planner.stats.rebuilt)
        self.assertLess(self.planner.stats.expanded, first_exp / 5.0)

    def test_new_goal_triggers_rebuilt(self):
        self.planner.plan((0.15, 0.15), (0.85, 0.85), snap_distance=1.0, radii=[0.1], now=0.0)
        self.planner.plan((0.15, 0.15), (0.45, 0.45), snap_distance=1.0, radii=[0.1], now=1.0)
        self.assertTrue(self.planner.stats.rebuilt)

    def test_goes_around_overlay_bar_and_back_after_it_expires(self):
        start = (0.15, 0.45)
        goal = (0.85, 0.45)
        # Direct straight path
        p1 = self.planner.plan(start, goal, snap_distance=1.0, radii=[0.05], now=0.0)
        self.assertEqual(len(p1), 2)

        # Add vertical overlay bar across middle (i=5, j=2..7)
        bar = {(5, j) for j in range(2, 8)}
        self.planner.set_obstacles(bar)
        p2 = self.planner.plan(start, goal, snap_distance=1.0, radii=[0.05], now=0.5)
        self.assertGreater(len(p2), 2)

        # Clear overlay: returns to direct straight path
        self.planner.set_obstacles([])
        p3 = self.planner.plan(start, goal, snap_distance=1.0, radii=[0.05], now=1.0)
        self.assertEqual(len(p3), 2)

    def test_start_in_blocked_space(self):
        # Place static wall at left
        rows = ["#........."] * 10
        raw = make_grid(rows, res=0.1)
        planner = DStarLitePlanner(InflatedMaps(raw))
        # Start at (0.05, 0.55), which is inside the static obstacle cell (0, 5)
        start = (0.05, 0.55)
        goal = (0.85, 0.55)
        path = planner.plan(start, goal, snap_distance=1.0, radii=[0.0])
        self.assertEqual(path[0], start)
        self.assertNotEqual(path[1], start)

    def test_goal_covered_by_overlay_is_resnapped(self):
        start = (0.15, 0.15)
        goal = (0.85, 0.85)
        self.planner.plan(start, goal, snap_distance=1.0, radii=[0.05], now=0.0)

        # Cover goal cell (8, 8) with overlay
        self.planner.set_obstacles([(8, 8)])
        path = self.planner.plan(start, goal, snap_distance=1.0, radii=[0.05], now=0.5)
        self.assertTrue(self.planner.stats.rebuilt)
        # Snapped goal is not the original blocked goal
        self.assertNotEqual(path[-1], goal)

    def test_goal_walled_in_raises_no_path(self):
        rows = [
            "..........",
            "..........",
            "..........",
            "..........",
            "..........",
            "..........",
            "....###...",
            "....#.#...",
            "....###...",
            "..........",
        ]
        raw = make_grid(rows, res=0.1)
        planner = DStarLitePlanner(InflatedMaps(raw))
        with self.assertRaises(NoPath):
            planner.plan((0.15, 0.15), (0.55, 0.25), snap_distance=0.05, radii=[0.0])

    def test_nan_goal_raises_no_path(self):
        with self.assertRaises(NoPath):
            self.planner.plan((0.15, 0.15), (float("nan"), 0.85), snap_distance=1.0, radii=[0.1])
        with self.assertRaises(NoPath):
            self.planner.plan((float("inf"), 0.15), (0.85, 0.85), snap_distance=1.0, radii=[0.1])

    def test_start_unsnappable_on_reuse_falls_back_to_narrower_radius(self):
        """When the reuse path cannot snap the start, fall through to narrower radii."""
        res = 0.05
        w, h = 60, 60
        data = [0] * (w * h)
        raw = Grid(data, w, h, res)
        maps = InflatedMaps(raw)
        planner = DStarLitePlanner(maps)

        start = (1.0, 1.0)
        goal = (2.0, 2.0)
        # First plan at wider radius
        planner.plan(start, goal, snap_distance=1.0, radii=[0.40, 0.0], now=0.0)

        # Block a large area around a new start so it can't be snapped at radius=0.40
        block = set()
        ci, cj = int(0.5 / res), int(0.5 / res)
        for di in range(-20, 21):
            for dj in range(-20, 21):
                ni, nj = ci + di, cj + dj
                if 0 <= ni < w and 0 <= nj < h:
                    block.add((ni, nj))
        planner.set_obstacles(block)

        # The new start is far from the old start, in a blocked region at wider radius
        # but reachable at radius=0.0
        new_start = (0.525, 0.525)
        # This should NOT raise NoPath — it should fall back to radius=0.0
        path = planner.plan(new_start, goal, snap_distance=2.0, radii=[0.40, 0.0], now=1.0)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], new_start)


class DStarLiteFallbackTest(unittest.TestCase):
    def test_fallback_to_narrower_radius_and_probe_widening(self):
        res = 0.05
        w, h = 100, 100
        data = [0] * (w * h)
        # Wall at i = 40..41 from j = 0 to 99, with:
        # - wide door at j = 10..35 (passable at 0.40 m)
        # - narrow 13-cell gap at j = 60..72 (passable at 0.28 m, blocked at 0.35 m / 0.40 m)
        for j in range(h):
            if (10 <= j <= 35) or (60 <= j <= 72):
                continue
            for i in (40, 41):
                data[j * w + i] = 100
        raw = Grid(data, w, h, res)
        maps = InflatedMaps(raw)
        planner = DStarLitePlanner(maps, widen_every=5.0)

        # Coordinates with y values not on a cell border:
        # 25.3 cells * 0.05 = 1.265 m
        start = (20 * res, 25.3 * res)
        goal = (60 * res, 25.3 * res)
        radii = [0.40, 0.28]
        snap_distance = 0.5

        # 1. Level 0 at first (door is open, passable at 0.40 m)
        planner.plan(start, goal, snap_distance=snap_distance, radii=radii, now=0.0)
        self.assertEqual(planner.stats.level, 0)

        # 2. Level 1 when the door is closed (falls back to 0.28 m to use 13-cell gap)
        door_cells = [(i, j) for i in (40, 41) for j in range(10, 36)]
        planner.set_obstacles(door_cells)
        planner.plan(start, goal, snap_distance=snap_distance, radii=radii, now=1.0)
        self.assertEqual(planner.stats.level, 1)

        # 3. Door reopened; still before widen_every (now=2.0 < 1.0 + 5.0) -> still Level 1
        planner.set_obstacles([])
        planner.plan(start, goal, snap_distance=snap_distance, radii=radii, now=2.0)
        self.assertEqual(planner.stats.level, 1)

        # 4. After widen_every (now=7.0 >= 1.0 + 5.0) -> back to Level 0
        planner.plan(start, goal, snap_distance=snap_distance, radii=radii, now=7.0)
        self.assertEqual(planner.stats.level, 0)

        # 5. Whole wall closed (both door and narrow gap blocked) -> NoPath
        gap_cells = [(i, j) for i in (40, 41) for j in range(60, 73)]
        planner.set_obstacles(door_cells + gap_cells)
        with self.assertRaises(NoPath):
            planner.plan(start, goal, snap_distance=snap_distance, radii=radii, now=8.0)


class IncrementalInflationEquivalenceTest(unittest.TestCase):
    def test_sync_matches_full_inflate_on_random_sequences(self):
        rng = random.Random(7)
        for _ in range(40):
            w, h = rng.choice([(40, 30), (60, 60), (90, 70)])
            data = [0] * (w * h)
            for _ in range(rng.randint(0, 25)):
                i, j = rng.randrange(w), rng.randrange(h)
                for a in range(rng.randint(1, 6)):
                    for b in range(rng.randint(1, 4)):
                        if i + a < w and j + b < h:
                            data[(j + b) * w + i + a] = 100
            for _ in range(rng.randint(0, 6)):
                data[rng.randrange(w * h)] = -1
            grid = Grid(data, w, h, 0.05, 0.0, 0.0)
            radius = rng.choice([0.28, 0.30, 0.40])
            inc = IncrementalInflation(inflate(grid, radius), radius)
            current = set()
            for _ in range(12):
                for _ in range(rng.randint(0, 30)):
                    current.add((rng.randint(-3, w + 2), rng.randint(-3, h + 2)))
                for c in rng.sample(sorted(current), min(len(current), rng.randint(0, 25))):
                    current.discard(c)
                before = bytes(inc.blocked)
                flipped = inc.sync(current)
                pts = [cell_center(grid, j * w + i) for i, j in current if 0 <= i < w and 0 <= j < h]
                expected = inflate(overlay_obstacles(grid, pts), radius).data
                self.assertEqual([100 if v else 0 for v in expected], list(inc.blocked))
                self.assertEqual(
                    set(flipped), {k for k in range(w * h) if before[k] != inc.blocked[k]}
                )


class DStarLiteViewSnapshotTest(unittest.TestCase):
    def test_view_snapshot_returns_none_before_any_plan(self):
        w, h, res = 20, 20, 0.05
        raw = Grid([0] * (w * h), w, h, res)
        planner = DStarLitePlanner(InflatedMaps(raw))
        self.assertIsNone(planner.view_snapshot())

    def test_view_snapshot_equals_inflated_overlay_after_plan(self):
        w, h, res = 30, 30, 0.05
        raw = Grid([0] * (w * h), w, h, res)
        planner = DStarLitePlanner(InflatedMaps(raw))
        obs_cells = [(10, 10), (10, 11), (20, 20)]
        planner.set_obstacles(obs_cells)
        radius = 0.35
        path = planner.plan((0.2, 0.2), (1.2, 1.2), snap_distance=1.0, radii=[radius])
        self.assertIsNotNone(path)

        snapshot = planner.view_snapshot()
        self.assertIsNotNone(snapshot)
        pts = [cell_center(raw, j * w + i) for i, j in obs_cells]
        expected = inflate(overlay_obstacles(raw, pts), planner.stats.radius)
        self.assertEqual(
            list(snapshot.data),
            [100 if v else 0 for v in expected.data]
        )

    def test_later_plan_does_not_mutate_earlier_snapshot(self):
        w, h, res = 30, 30, 0.05
        raw = Grid([0] * (w * h), w, h, res)
        planner = DStarLitePlanner(InflatedMaps(raw))
        planner.set_obstacles([(5, 5)])
        planner.plan((0.2, 0.2), (1.2, 1.2), snap_distance=1.0, radii=[0.35])
        snap1 = planner.view_snapshot()
        self.assertIsNotNone(snap1)
        data_before = bytes(snap1.data)

        # Later plan with different obstacles
        planner.set_obstacles([(15, 15), (16, 16), (17, 17)])
        planner.plan((0.2, 0.2), (1.2, 1.2), snap_distance=1.0, radii=[0.35])

        # The earlier snapshot must remain unchanged
        self.assertEqual(bytes(snap1.data), data_before)
        snap2 = planner.view_snapshot()
        self.assertNotEqual(bytes(snap2.data), bytes(snap1.data))


class ExpandLidarHitTest(unittest.TestCase):
    def test_filter_drops_hit_on_and_beside_raw_obstacle_and_keeps_open_space(self):
        w, h, res = 30, 30, 0.05
        data = [0] * (w * h)
        data[10 * w + 10] = 100
        grid = Grid(data, w, h, res)

        # 1. Hit directly on raw obstacle cell
        cells, dropped = expand_lidar_hit(grid, 10, 10, (11, 10), filter_static=True)
        self.assertTrue(dropped)
        self.assertEqual(cells, [])

        # 2. Hit directly beside raw obstacle cell (all 8 neighbours in 3x3)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                cells, dropped = expand_lidar_hit(grid, 10 + di, 10 + dj, (15, 15), filter_static=True)
                self.assertTrue(dropped)
                self.assertEqual(cells, [])

        # 3. Hit in open space keeps hit (9 cells + depth cell, dropped False)
        cells, dropped = expand_lidar_hit(grid, 20, 20, (22, 20), filter_static=True)
        self.assertFalse(dropped)
        self.assertEqual(len(cells), 10)
        self.assertIn((22, 20), cells)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                self.assertIn((20 + di, 20 + dj), cells)

    def test_legacy_equivalence(self):
        def legacy_expand(w, h, ci, cj, depth_cell):
            new_cells = []
            if 0 <= ci < w and 0 <= cj < h:
                new_cells.append((ci, cj))
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        ni, nj = ci + di, cj + dj
                        if 0 <= ni < w and 0 <= nj < h:
                            new_cells.append((ni, nj))
                if depth_cell is not None:
                    di_depth, dj_depth = depth_cell
                    if 0 <= di_depth < w and 0 <= dj_depth < h:
                        new_cells.append((di_depth, dj_depth))
            return set(new_cells)

        w, h, res = 30, 20, 0.05
        grid = Grid([0] * (w * h), w, h, res)

        rng = random.Random(42)
        for _ in range(500):
            ci = rng.randint(-2, w + 2)
            cj = rng.randint(-2, h + 2)
            if rng.random() < 0.2:
                depth_cell = None
            else:
                depth_cell = (rng.randint(-2, w + 2), rng.randint(-2, h + 2))
            cells, dropped = expand_lidar_hit(grid, ci, cj, depth_cell, filter_static=False)
            self.assertFalse(dropped)
            self.assertEqual(set(cells), legacy_expand(w, h, ci, cj, depth_cell))

    def test_dynamic_box_boundary_cells_not_dropped(self):
        w, h, res = 200, 120, 0.05
        data = [0] * (w * h)
        for ci in range(w):
            data[0 * w + ci] = 100
            data[(h - 1) * w + ci] = 100
        for cj in range(h):
            data[cj * w + 0] = 100
            data[cj * w + (w - 1)] = 100

        for ci in range(int(round(3.0 / res)), int(round(7.0 / res))):
            for cj in range(1, int(round(2.55 / res))):
                data[cj * w + ci] = 100
            for cj in range(int(round(3.40 / res)), int(round(5.0 / res))):
                data[cj * w + ci] = 100

        grid = Grid(data, w, h, res)

        # Dynamic box of 0.3 x 0.3 m at x 2.0..2.3, y 2.85..3.15
        ci_min, ci_max = int(round(2.0 / res)), int(round(2.3 / res)) - 1
        cj_min, cj_max = int(round(2.85 / res)), int(round(3.15 / res)) - 1
        boundary = set()
        for ci in range(ci_min, ci_max + 1):
            for cj in range(cj_min, cj_max + 1):
                if ci in (ci_min, ci_max) or cj in (cj_min, cj_max):
                    boundary.add((ci, cj))

        self.assertGreater(len(boundary), 0)
        for ci, cj in boundary:
            cells, dropped = expand_lidar_hit(grid, ci, cj, None, filter_static=True)
            self.assertFalse(dropped)


class PathNearPointsTest(unittest.TestCase):
    def test_path_near_points_radius_range_and_zero_length_segments(self):
        # 1. Straight path with obstacle points at 0.37 m from it
        corners = [(0.0, 0.0), (2.0, 0.0)]
        pts = [(1.0, 0.37)]
        robot_xy = (0.0, 0.0)

        # True at radius 0.38, False at radius 0.33
        self.assertTrue(path_near_points(corners, pts, robot_xy, radius=0.38))
        self.assertFalse(path_near_points(corners, pts, robot_xy, radius=0.33))

        # 2. Points farther than 3.5 m from the robot are ignored
        far_pts = [(4.0, 0.1)]
        long_corners = [(0.0, 0.0), (5.0, 0.0)]
        self.assertFalse(path_near_points(long_corners, far_pts, robot_xy, radius=0.38, max_range=3.5))

        # 3. Zero-length segments are skipped
        corners_with_zero = [(0.0, 0.0), (0.0, 0.0), (2.0, 0.0)]
        self.assertTrue(path_near_points(corners_with_zero, pts, robot_xy, radius=0.38))


class AisleScenarioOverlayFilterTest(unittest.TestCase):
    def test_aisle_scenario_overlay_filter(self):
        w, h, res = 200, 120, 0.05
        data = [0] * (w * h)
        # Border walls 1 cell thick
        for ci in range(w):
            data[0 * w + ci] = 100
            data[(h - 1) * w + ci] = 100
        for cj in range(h):
            data[cj * w + 0] = 100
            data[cj * w + (w - 1)] = 100

        # Lower shelf x 3..7 m, y 0.05..2.575 m; upper shelf x 3..7 m, y 3.425..5.0 m
        # (aisle at y=3.0 is 0.85 m wide, detour strip y 5.0..5.95)
        for ci in range(int(round(3.0 / res)), int(round(7.0 / res))):
            for cj in range(1, int(round(2.55 / res))):
                data[cj * w + ci] = 100
            for cj in range(int(round(3.40 / res)), int(round(5.0 / res))):
                data[cj * w + ci] = 100

        grid = Grid(data, w, h, res)

        overlay_a = set()
        overlay_b = set()
        for cj in range(h):
            for ci in range(w):
                if data[cj * w + ci] != 0:
                    cx = (ci + 0.5) * res
                    if 2.0 <= cx <= 7.0:
                        has_free = any(
                            0 <= ci + di < w and 0 <= cj + dj < h and data[(cj + dj) * w + (ci + di)] == 0
                            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1))
                        )
                        if has_free:
                            ca, _ = expand_lidar_hit(grid, ci, cj, None, filter_static=False)
                            overlay_a.update(ca)
                            cb, dropped = expand_lidar_hit(grid, ci, cj, None, filter_static=True)
                            self.assertTrue(dropped)
                            overlay_b.update(cb)

        # Overlay B must be empty
        self.assertEqual(len(overlay_b), 0)

        # Plan (1, 3) -> (9, 3) with radii [0.40, 0.35, 0.30, 0.28], snap 2.0
        radii = [0.40, 0.35, 0.30, 0.28]

        # Assert B: stats.level == 0 and the path stays near y=3 (max y < 3.5)
        planner_b = DStarLitePlanner(InflatedMaps(grid))
        planner_b.set_obstacles(list(overlay_b))
        path_b = planner_b.plan((1.0, 3.0), (9.0, 3.0), snap_distance=2.0, radii=radii)
        self.assertIsNotNone(path_b)
        self.assertEqual(planner_b.stats.level, 0)
        self.assertLess(max(p[1] for p in path_b), 3.5)

        # Assert A: level > 0 OR max y > 4.4 (old behaviour closes the aisle at 0.40)
        planner_a = DStarLitePlanner(InflatedMaps(grid))
        planner_a.set_obstacles(list(overlay_a))
        path_a = planner_a.plan((1.0, 3.0), (9.0, 3.0), snap_distance=2.0, radii=radii)
        self.assertIsNotNone(path_a)
        self.assertTrue(planner_a.stats.level > 0 or max(p[1] for p in path_a) > 4.4)


class DStarPlannerInitStructureTest(unittest.TestCase):
    def test_init_contains_publishers_and_subscribers_and_callbacks_clean(self):
        planner_path = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "dstar_planner.py")
        )
        with open(planner_path, "r", encoding="utf-8") as f:
            tree = ast.parse(f.read(), filename=planner_path)

        dstar_class = None
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "DStarPlanner":
                dstar_class = node
                break

        self.assertIsNotNone(dstar_class, "DStarPlanner class not found")

        init_func = None
        other_methods = []
        for item in dstar_class.body:
            if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                if item.name == "__init__":
                    init_func = item
                else:
                    other_methods.append(item)

        self.assertIsNotNone(init_func, "__init__ not found")

        def is_call_to(node, module, attr):
            if not isinstance(node, ast.Call):
                return False
            func = node.func
            if isinstance(func, ast.Attribute) and func.attr == attr:
                if isinstance(func.value, ast.Name) and func.value.id == module:
                    return True
            return False

        init_sub_count = 0
        init_pub_count = 0
        for n in ast.walk(init_func):
            if is_call_to(n, "rospy", "Subscriber"):
                init_sub_count += 1
            elif is_call_to(n, "rospy", "Publisher"):
                init_pub_count += 1

        self.assertGreaterEqual(
            init_sub_count, 5, f"Expected >= 5 rospy.Subscriber calls in __init__, found {init_sub_count}"
        )
        self.assertGreaterEqual(
            init_pub_count, 3, f"Expected >= 3 rospy.Publisher calls in __init__, found {init_pub_count}"
        )

        for m in other_methods:
            for n in ast.walk(m):
                self.assertFalse(
                    is_call_to(n, "rospy", "Subscriber"),
                    f"Found rospy.Subscriber call outside __init__ in method {m.name}",
                )


# ---------------- ROS-free Node Smoke Test ----------------
class FakeTime:
    def __init__(self, t: float = 0.0) -> None:
        self._t = float(t)

    def to_sec(self) -> float:
        return self._t

    def __sub__(self, other):
        val = other.to_sec() if hasattr(other, "to_sec") else float(other)
        return FakeDuration(self._t - val)


class FakeDuration:
    def __init__(self, d: float) -> None:
        self._d = float(d)

    def to_sec(self) -> float:
        return self._d


class FakeRospy:
    def __init__(self, params=None) -> None:
        self.params = dict(params or {})
        self.publishers = {}
        self.subscribers = {}
        self.logs = []
        self._throttles = {}
        self.sim_time = 100.0
        self.Time = self

    def now(self) -> FakeTime:
        return FakeTime(self.sim_time)

    def get_param(self, name: str, default: Any = None) -> Any:
        return self.params.get(name, default)

    def Publisher(self, topic: str, data_class: Any, queue_size: Any = None, latch: bool = False):
        if topic not in self.publishers:
            self.publishers[topic] = []

        class Pub:
            def __init__(pub_self) -> None:
                pub_self.topic = topic

            def publish(pub_self, msg: Any) -> None:
                self.publishers[topic].append(msg)

        return Pub()

    def Subscriber(self, topic: str, data_class: Any, callback: Any) -> None:
        self.subscribers[topic] = (data_class, callback)

    def loginfo(self, msg: str, *args: Any) -> None:
        self.logs.append(("INFO", msg, args))

    def loginfo_throttle(self, period: float, msg: str, *args: Any) -> None:
        last_t = self._throttles.get(msg, -1e9)
        if self.sim_time - last_t >= period:
            self._throttles[msg] = self.sim_time
            self.logs.append(("INFO_THROTTLE", msg, args))

    def logwarn(self, msg: str, *args: Any) -> None:
        self.logs.append(("WARN", msg, args))

    def logwarn_throttle(self, period: float, msg: str, *args: Any) -> None:
        last_t = self._throttles.get(msg, -1e9)
        if self.sim_time - last_t >= period:
            self._throttles[msg] = self.sim_time
            self.logs.append(("WARN_THROTTLE", msg, args))

    def logerr_throttle(self, period: float, msg: str, *args: Any) -> None:
        last_t = self._throttles.get(msg, -1e9)
        if self.sim_time - last_t >= period:
            self._throttles[msg] = self.sim_time
            self.logs.append(("ERR_THROTTLE", msg, args))

    def init_node(self, name: str) -> None:
        pass

    def spin(self) -> None:
        pass


class HeaderStub:
    def __init__(self, frame_id: str = "map", stamp: Any = None) -> None:
        self.frame_id = frame_id
        self.stamp = stamp or FakeTime(100.0)


class PointMsgStub:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        self.x, self.y, self.z = float(x), float(y), float(z)


class QuaternionMsgStub:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0) -> None:
        self.x, self.y, self.z, self.w = float(x), float(y), float(z), float(w)


class PoseMsgStub:
    def __init__(self) -> None:
        self.position = PointMsgStub()
        self.orientation = QuaternionMsgStub()


class PoseStampedStub:
    def __init__(self) -> None:
        self.header = HeaderStub()
        self.pose = PoseMsgStub()


class MapMetaDataStub:
    def __init__(self) -> None:
        self.resolution = 0.05
        self.width = 0
        self.height = 0
        self.origin = PoseMsgStub()


class OccupancyGridStub:
    def __init__(self) -> None:
        self.header = HeaderStub()
        self.info = MapMetaDataStub()
        self.data = []


class PathStub:
    def __init__(self) -> None:
        self.header = HeaderStub()
        self.poses = []


class LaserScanStub:
    def __init__(self) -> None:
        self.header = HeaderStub()
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = 2 * math.pi / 360
        self.time_increment = 0.0
        self.scan_time = 0.1
        self.range_min = 0.05
        self.range_max = 3.5
        self.ranges = []
        self.intensities = []


class EmptyStub:
    pass


class StringStub:
    def __init__(self, data: str = "") -> None:
        self.data = data


@contextlib.contextmanager
def fake_rospy_context(params=None, mod_name="dstar_planner_smoke", script_name="dstar_planner.py"):
    fake_rospy = FakeRospy(params)
    rospy_mod = types.ModuleType("rospy")
    for k in dir(fake_rospy):
        if not k.startswith("__"):
            setattr(rospy_mod, k, getattr(fake_rospy, k))

    modules_dict = {
        "rospy": rospy_mod,
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
        "nav_msgs": types.ModuleType("nav_msgs"),
        "nav_msgs.msg": types.ModuleType("nav_msgs.msg"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
    }
    modules_dict["geometry_msgs.msg"].PoseStamped = PoseStampedStub
    modules_dict["nav_msgs.msg"].OccupancyGrid = OccupancyGridStub
    modules_dict["nav_msgs.msg"].Path = PathStub
    modules_dict["sensor_msgs.msg"].LaserScan = LaserScanStub
    modules_dict["std_msgs.msg"].Empty = EmptyStub
    modules_dict["std_msgs.msg"].String = StringStub

    planner_file = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", script_name)
    )

    with unittest.mock.patch.dict(sys.modules, modules_dict):
        spec = importlib.util.spec_from_file_location(mod_name, planner_file)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        yield mod, fake_rospy


class NodeSmokeTest(unittest.TestCase):
    def test_wiring(self):
        with fake_rospy_context({}, "dp_wiring_test") as (mod, fake_rospy):
            node = mod.DStarPlanner()
            expected_subs = {"/map", "/robot/pose", "/scan", "/move_base_simple/goal", "/nav/replan"}
            expected_pubs = {"/planned_path", "/costmap", "/nav/leg_result"}
            self.assertEqual(set(fake_rospy.subscribers.keys()), expected_subs)
            self.assertEqual(set(fake_rospy.publishers.keys()), expected_pubs)

    def test_end_to_end_aisle(self):
        w, h, res = 200, 120, 0.05
        data = [0] * (w * h)
        for ci in range(w):
            data[0 * w + ci] = 100
            data[(h - 1) * w + ci] = 100
        for cj in range(h):
            data[cj * w + 0] = 100
            data[cj * w + (w - 1)] = 100

        for ci in range(int(round(3.0 / res)), int(round(7.0 / res))):
            for cj in range(1, int(round(2.55 / res))):
                data[cj * w + ci] = 100
            for cj in range(int(round(3.40 / res)), int(round(5.0 / res))):
                data[cj * w + ci] = 100

        map_msg = OccupancyGridStub()
        map_msg.info.resolution = res
        map_msg.info.width = w
        map_msg.info.height = h
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = data

        pose_msg = PoseStampedStub()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = FakeTime(100.0)
        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 3.0
        pose_msg.pose.orientation.w = 1.0

        scan_msg = LaserScanStub()
        scan_msg.header.frame_id = "base_link"
        scan_msg.header.stamp = FakeTime(100.0)
        scan_msg.angle_min = 0.0
        scan_msg.angle_increment = 2 * math.pi / 360
        scan_msg.range_max = 3.5
        scan_msg.range_min = 0.05
        ranges = []
        for k in range(360):
            theta = 0.0 + k * scan_msg.angle_increment
            r = 0.025
            hit_r = float("inf")
            while r <= 3.5:
                px = 1.0 + r * math.cos(theta)
                py = 3.0 + r * math.sin(theta)
                ci = int(math.floor(px / res))
                cj = int(math.floor(py / res))
                if 0 <= ci < w and 0 <= cj < h:
                    if data[cj * w + ci] != 0:
                        hit_r = r
                        break
                else:
                    break
                r += 0.025
            ranges.append(hit_r)
        scan_msg.ranges = ranges

        goal_msg = PoseStampedStub()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = FakeTime(100.0)
        goal_msg.pose.position.x = 9.0
        goal_msg.pose.position.y = 3.0

        # Case 1: fix on (~filter_static_hits True, ~blocked_use_plan_radius True)
        params_on = {"~filter_static_hits": True, "~blocked_use_plan_radius": True}
        with fake_rospy_context(params_on, "dp_e2e_on") as (mod, fake_rospy):
            node_on = mod.DStarPlanner()
            node_on.on_map(map_msg)
            node_on.on_pose(pose_msg)
            node_on.on_scan(scan_msg)
            node_on.on_goal(goal_msg)
            path_on = fake_rospy.publishers["/planned_path"][-1]
            max_y_on = max(p.pose.position.y for p in path_on.poses)
            self.assertLess(max_y_on, 3.5)
            self.assertEqual(node_on.current_path_radius, 0.40)
            obs_on_len = len(node_on.obstacle_cells)

        # Case 2: fix off (both False)
        params_off = {"~filter_static_hits": False, "~blocked_use_plan_radius": False}
        with fake_rospy_context(params_off, "dp_e2e_off") as (mod, fake_rospy):
            node_off = mod.DStarPlanner()
            node_off.on_map(map_msg)
            node_off.on_pose(pose_msg)
            node_off.on_scan(scan_msg)
            node_off.on_goal(goal_msg)
            path_off = fake_rospy.publishers["/planned_path"][-1]
            max_y_off = max(p.pose.position.y for p in path_off.poses)
            self.assertTrue(max_y_off > 4.4 or node_off.current_path_radius < 0.40)
            self.assertGreater(len(node_off.obstacle_cells), obs_on_len)

    def test_check_radius(self):
        w, h, res = 200, 120, 0.05
        map_msg = OccupancyGridStub()
        map_msg.info.resolution = res
        map_msg.info.width = w
        map_msg.info.height = h
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = [0] * (w * h)

        def run_test(fix_on: bool, mod_name: str):
            params = {"~filter_static_hits": fix_on, "~blocked_use_plan_radius": fix_on}
            with fake_rospy_context(params, mod_name) as (mod, fake_rospy):
                node = mod.DStarPlanner()
                node.on_map(map_msg)

                pose_msg = PoseStampedStub()
                pose_msg.header.stamp = FakeTime(fake_rospy.sim_time)
                pose_msg.pose.position.x = 1.0
                pose_msg.pose.position.y = 3.0
                pose_msg.pose.orientation.w = 1.0
                node.on_pose(pose_msg)

                scan_msg = LaserScanStub()
                scan_msg.header.stamp = FakeTime(fake_rospy.sim_time)
                scan_msg.ranges = [float("inf")] * 360
                node.on_scan(scan_msg)

                goal_msg = PoseStampedStub()
                goal_msg.header.stamp = FakeTime(fake_rospy.sim_time)
                goal_msg.pose.position.x = 2.0
                goal_msg.pose.position.y = 3.0
                node.on_goal(goal_msg)

                # Advance fake time past throttle period (5.0s)
                fake_rospy.sim_time += 10.0
                pose_msg.header.stamp = FakeTime(fake_rospy.sim_time)
                node.on_pose(pose_msg)
                scan_msg.header.stamp = FakeTime(fake_rospy.sim_time)
                node.on_scan(scan_msg)

                overlay_logs = [log for log in fake_rospy.logs if "scan overlay:" in log[1]]
                self.assertGreater(len(overlay_logs), 0)
                last_log = overlay_logs[-1]
                check_r = last_log[2][3]
                return check_r, node.current_path_radius

        cr_off, pr_off = run_test(False, "dp_cr_off")
        self.assertEqual(cr_off, 0.38)

        cr_on, pr_on = run_test(True, "dp_cr_on")
        self.assertEqual(round(cr_on, 2), round(pr_on - 0.02, 2))

    def test_check_radius_adaptive(self):
        w, h, res = 200, 120, 0.05
        data = [0] * (w * h)
        for ci in range(w):
            data[0 * w + ci] = 100
            data[(h - 1) * w + ci] = 100
        for cj in range(h):
            data[cj * w + 0] = 100
            data[cj * w + (w - 1)] = 100

        # Aisle map without detour strip: upper shelf runs from y=3.40 to top border
        for ci in range(int(round(3.0 / res)), int(round(7.0 / res))):
            for cj in range(1, int(round(2.55 / res))):
                data[cj * w + ci] = 100
            for cj in range(int(round(3.40 / res)), h - 1):
                data[cj * w + ci] = 100

        map_msg = OccupancyGridStub()
        map_msg.info.resolution = res
        map_msg.info.width = w
        map_msg.info.height = h
        map_msg.info.origin.position.x = 0.0
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.orientation.w = 1.0
        map_msg.data = data

        def build_scan(sim_time: float):
            scan_msg = LaserScanStub()
            scan_msg.header.frame_id = "base_link"
            scan_msg.header.stamp = FakeTime(sim_time)
            scan_msg.angle_min = 0.0
            scan_msg.angle_increment = 2 * math.pi / 360
            scan_msg.range_max = 3.5
            scan_msg.range_min = 0.05
            ranges = []
            for k in range(360):
                theta = 0.0 + k * scan_msg.angle_increment
                r = 0.025
                hit_r = float("inf")
                while r <= 3.5:
                    px = 1.0 + r * math.cos(theta)
                    py = 3.0 + r * math.sin(theta)
                    ci = int(math.floor(px / res))
                    cj = int(math.floor(py / res))
                    if 0 <= ci < w and 0 <= cj < h:
                        if data[cj * w + ci] != 0:
                            hit_r = r
                            break
                    else:
                        break
                    r += 0.025
                ranges.append(hit_r)
            scan_msg.ranges = ranges
            return scan_msg

        def run_case(use_plan_radius: bool, mod_name: str):
            params = {"~filter_static_hits": False, "~blocked_use_plan_radius": use_plan_radius}
            with fake_rospy_context(params, mod_name) as (mod, fake_rospy):
                node = mod.DStarPlanner()
                node.on_map(map_msg)

                pose_msg = PoseStampedStub()
                pose_msg.header.frame_id = "map"
                pose_msg.header.stamp = FakeTime(fake_rospy.sim_time)
                pose_msg.pose.position.x = 1.0
                pose_msg.pose.position.y = 3.0
                pose_msg.pose.orientation.w = 1.0
                node.on_pose(pose_msg)

                scan_msg = build_scan(fake_rospy.sim_time)
                node.on_scan(scan_msg)

                goal_msg = PoseStampedStub()
                goal_msg.header.frame_id = "map"
                goal_msg.header.stamp = FakeTime(fake_rospy.sim_time)
                goal_msg.pose.position.x = 9.0
                goal_msg.pose.position.y = 3.0
                node.on_goal(goal_msg)

                current_path_radius = node.current_path_radius

                # Advance fake time by 10s past throttle period (5.0s)
                fake_rospy.sim_time += 10.0
                pose_msg.header.stamp = FakeTime(fake_rospy.sim_time)
                node.on_pose(pose_msg)
                scan_msg2 = build_scan(fake_rospy.sim_time)
                node.on_scan(scan_msg2)

                overlay_logs = [log for log in fake_rospy.logs if "scan overlay:" in log[1]]
                self.assertGreater(len(overlay_logs), 0)
                last_log = overlay_logs[-1]
                check_radius = last_log[2][3]
                return current_path_radius, check_radius

        # Case A: ~filter_static_hits False, ~blocked_use_plan_radius True
        pr_a, cr_a = run_case(True, "dp_adaptive_on")
        if pr_a != 0.35:
            print("Case A actual current_path_radius:", pr_a)
        self.assertLess(pr_a, 0.40)
        self.assertEqual(round(cr_a, 2), round(pr_a - 0.02, 2))
        self.assertNotEqual(cr_a, 0.38)

        # Case B: same map and sequence with ~blocked_use_plan_radius False
        pr_b, cr_b = run_case(False, "dp_adaptive_off")
        self.assertEqual(pr_b, pr_a)
        self.assertLess(pr_b, 0.40)
        self.assertEqual(cr_b, 0.38)

    def test_astar_planner_parity(self):
        """Verify AStarPlanner has identical default parameters, wiring, and static filtering."""
        with fake_rospy_context({}, "ap_wiring_test", "astar_planner.py") as (mod, fake_rospy):
            node = mod.AStarPlanner()
            expected_subs = {"/map", "/robot/pose", "/scan", "/move_base_simple/goal", "/nav/replan"}
            expected_pubs = {"/planned_path", "/costmap", "/nav/leg_result"}
            self.assertEqual(set(fake_rospy.subscribers.keys()), expected_subs)
            self.assertEqual(set(fake_rospy.publishers.keys()), expected_pubs)
            self.assertTrue(node._get_param_cached("~filter_static_hits", True))
            self.assertTrue(node._get_param_cached("~blocked_use_plan_radius", True))
            self.assertEqual(node._get_param_cached("~obstacle_memory_s", 6.0), 6.0)
            self.assertFalse(node._get_param_cached("~trigger_debug", False))


if __name__ == "__main__":
    unittest.main()

