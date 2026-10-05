"""Unit tests for grid_planner: the planning logic, with no ROS needed."""
import math
import random
import time
import unittest

import support  # noqa: F401  (puts scripts/ on sys.path)
from grid_planner import (
    Grid,
    InflatedMaps,
    NoPath,
    ascii_map,
    astar,
    cell_center,
    cell_index,
    inflate,
    line_of_sight,
    nearest_free,
    plan,
    plan_inflated,
    reachable_from,
    smooth,
)
from support import blocked_cells, make_grid


def edge_distance(point, cell):
    """Distance from a point to the closed square of cell (i, j), res 1, origin 0."""
    i, j = cell
    dx = max(abs(point[0] - (i + 0.5)) - 0.5, 0.0)
    dy = max(abs(point[1] - (j + 0.5)) - 0.5, 0.0)
    return math.hypot(dx, dy)


def random_rows(rng, density, size=8):
    """ASCII rows of a size x size grid with about `density` of the cells blocked."""
    return [
        "".join("#" if rng.random() < density else "." for _ in range(size))
        for _ in range(size)
    ]


def random_point(rng, size=8):
    return (rng.uniform(0, size - 0.01), rng.uniform(0, size - 0.01))


def segment_touches_cell(a, b, cell):
    """Exact test (slab method): does the closed segment a-b meet the square of cell?"""
    t0, t1 = 0.0, 1.0
    for p, q, low in ((a[0], b[0], cell[0]), (a[1], b[1], cell[1])):
        d = q - p
        if d == 0.0:
            if p < low or p > low + 1:
                return False
            continue
        ta, tb = (low - p) / d, (low + 1 - p) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return True


class GridTest(unittest.TestCase):
    def test_rejects_data_that_does_not_match_the_size(self):
        with self.assertRaises(ValueError):
            Grid([0, 0, 0], 2, 2, 1.0)

    def test_rejects_an_empty_grid(self):
        with self.assertRaises(ValueError):
            Grid([], 0, 0, 1.0)

    def test_rejects_a_resolution_that_is_not_positive(self):
        with self.assertRaises(ValueError):
            Grid([0], 1, 1, 0.0)

    def test_cell_index_and_centre_agree(self):
        grid = Grid([0] * 12, 4, 3, 0.5, -1.0, 2.0)
        x, y = cell_center(grid, 5)  # i = 1, j = 1
        self.assertAlmostEqual(x, -0.25)
        self.assertAlmostEqual(y, 2.75)
        self.assertEqual(cell_index(grid, x, y), 5)

    def test_cell_index_is_minus_one_outside_the_grid(self):
        grid = Grid([0] * 4, 2, 2, 1.0)
        for x, y in ((-0.1, 0.5), (0.5, -0.1), (2.0, 0.5), (0.5, 2.0)):
            self.assertEqual(cell_index(grid, x, y), -1)


class NearestFreeTest(unittest.TestCase):
    def test_a_free_point_returns_its_own_cell(self):
        grid = make_grid(["...", "...", "..."])
        self.assertEqual(nearest_free(grid, 1.5, 1.5, 2.0), 4)

    def test_a_free_point_on_a_cell_corner_returns_the_cell_that_contains_it(self):
        # (1, 1) is equally far from the centres of four cells; the one that holds it is (1, 1).
        grid = make_grid(["...", "...", "..."])
        self.assertEqual(nearest_free(grid, 1.0, 1.0, 2.0), 1 * 3 + 1)

    def test_the_cell_that_holds_the_point_must_still_be_allowed(self):
        grid = make_grid(["...", "...", "..."])
        mask = [True] * 9
        mask[4] = False
        self.assertNotEqual(nearest_free(grid, 1.5, 1.5, 2.0, mask), 4)

    def test_a_blocked_point_snaps_to_the_closest_free_cell(self):
        grid = make_grid([".....", ".###.", ".###.", ".###.", "....."])
        # The point is in blocked cell (1, 2). The closest free centre is cell (0, 2).
        self.assertEqual(nearest_free(grid, 1.6, 2.5, 3.0), 2 * 5 + 0)

    def test_the_search_area_is_a_circle_not_a_square(self):
        # The only free cell (7, 7) is 4.24 m diagonally away: inside a 3.5 m
        # square window around the point, but outside a 3.5 m radius.
        rows = ["#" * 9 for _ in range(9)]
        rows[1] = "#" * 7 + "." + "#"
        grid = make_grid(rows)
        self.assertEqual(nearest_free(grid, 4.5, 4.5, 3.5), -1)

    def test_a_free_cell_inside_the_radius_is_found(self):
        rows = ["#" * 9 for _ in range(9)]
        rows[1] = "#" * 4 + "." + "#" * 4  # free cell (4, 7), 3.0 m straight above
        grid = make_grid(rows)
        self.assertEqual(nearest_free(grid, 4.5, 4.5, 3.5), 7 * 9 + 4)

    def test_cells_outside_the_mask_are_skipped(self):
        grid = make_grid(["...."])
        mask = [False, False, True, True]
        self.assertEqual(nearest_free(grid, 0.5, 0.5, 5.0, mask), 2)

    def test_nothing_is_found_when_the_point_is_far_outside_the_grid(self):
        grid = make_grid(["..."])
        self.assertEqual(nearest_free(grid, 100.0, 100.0, 6.0), -1)

    def test_a_point_just_outside_the_grid_snaps_to_the_border(self):
        grid = make_grid(["..."])
        self.assertEqual(nearest_free(grid, -1.0, 0.5, 2.0), 0)

    def test_nothing_is_found_when_every_cell_is_blocked(self):
        grid = make_grid(["##", "##"])
        self.assertEqual(nearest_free(grid, 1.0, 1.0, 5.0), -1)


class ReachableFromTest(unittest.TestCase):
    def test_flood_fill_stops_at_walls(self):
        grid = make_grid([".#.", ".#.", ".#."])
        mask = reachable_from(grid, 0)
        self.assertEqual([idx for idx, ok in enumerate(mask) if ok], [0, 3, 6])


class InflateTest(unittest.TestCase):
    def one_block(self, res=1.0):
        rows = ["......."] * 7
        rows[3] = "...#..."
        return make_grid(rows, res=res)

    def test_zero_radius_changes_nothing(self):
        self.assertEqual(blocked_cells(inflate(self.one_block(), 0.0)), {(3, 3)})

    def test_a_radius_of_one_cell_blocks_the_eight_neighbours(self):
        expected = {(i, j) for i in range(2, 5) for j in range(2, 5)}
        self.assertEqual(blocked_cells(inflate(self.one_block(), 1.0)), expected)

    def test_the_inflated_shape_is_round(self):
        box = {(i, j) for i in range(1, 6) for j in range(1, 6)}
        corners = {(1, 1), (1, 5), (5, 1), (5, 5)}
        self.assertEqual(blocked_cells(inflate(self.one_block(), 1.6)), box - corners)

    def test_a_cell_exactly_radius_away_stays_free(self):
        # A neighbour's centre is 0.5 m from the obstacle's edge.
        self.assertEqual(blocked_cells(inflate(self.one_block(), 0.5)), {(3, 3)})

    def test_the_resolution_scales_the_inflation(self):
        expected = {(i, j) for i in range(2, 5) for j in range(2, 5)}
        grid = self.one_block(res=0.5)
        self.assertEqual(blocked_cells(inflate(grid, 0.5)), expected)

    def test_the_input_grid_is_not_modified(self):
        grid = self.one_block()
        before = list(grid.data)
        result = inflate(grid, 1.0)
        self.assertEqual(list(grid.data), before)
        self.assertIsNot(result, grid)

    def test_inflation_does_not_wrap_around_the_grid_edge(self):
        grid = make_grid(["#....", "#....", "#...."])
        blocked = blocked_cells(inflate(grid, 1.0))
        self.assertIn((1, 1), blocked)
        for j in range(3):
            self.assertNotIn((4, j), blocked)


def brute_force_inflate(grid, radius):
    """The definition, slowly: blocked if the cell centre is closer than radius to a blocked cell's edge."""
    w, h = grid.width, grid.height
    obstacles = blocked_cells(grid)
    out = set(obstacles)
    for j in range(h):
        for i in range(w):
            for (oi, oj) in obstacles:
                dx = max(abs(i - oi) - 0.5, 0.0) * grid.res
                dy = max(abs(j - oj) - 0.5, 0.0) * grid.res
                if (i, j) != (oi, oj) and math.hypot(dx, dy) < radius:
                    out.add((i, j))
                    break
    return out


class InflateAgainstTheDefinitionTest(unittest.TestCase):
    def test_matches_the_slow_definition_on_random_grids(self):
        rng = random.Random(11)
        for _ in range(60):
            res = rng.choice([1.0, 0.5, 0.05])
            rows = random_rows(rng, rng.choice([0.05, 0.15, 0.4]), size=12)
            grid = make_grid(rows, res=res)
            radius = rng.choice([0.4, 0.5, 0.9, 1.3, 2.0]) * (res if res < 1.0 else 1.0)
            radius *= rng.choice([1.0, 3.0, 7.0])
            fast = blocked_cells(inflate(grid, radius))
            self.assertEqual(fast, brute_force_inflate(grid, radius), (rows, res, radius))

    def test_a_large_radius_fills_the_whole_small_grid(self):
        grid = make_grid(["....", "....", "..#.", "...."])
        self.assertEqual(len(blocked_cells(inflate(grid, 50.0))), 16)


class InflatedMapsTest(unittest.TestCase):
    def one_block(self):
        rows = ["......."] * 7
        rows[3] = "...#..."
        return make_grid(rows)

    def test_the_same_radius_gives_the_same_grid_object(self):
        maps = InflatedMaps(self.one_block())
        self.assertIs(maps.get(1.0), maps.get(1.0))

    def test_a_different_radius_gives_a_different_grid(self):
        maps = InflatedMaps(self.one_block())
        small, large = maps.get(1.0), maps.get(1.6)
        self.assertLess(len(blocked_cells(small)), len(blocked_cells(large)))

    def test_no_inflation_gives_back_the_raw_grid(self):
        grid = self.one_block()
        self.assertIs(InflatedMaps(grid).get(0.0), grid)

    def test_only_a_few_radii_are_kept(self):
        maps = InflatedMaps(self.one_block(), limit=2)
        first = maps.get(1.0)
        maps.get(1.2)
        maps.get(1.4)  # the cache is full: older entries go
        self.assertEqual(blocked_cells(maps.get(1.0)), blocked_cells(first))  # same answer, rebuilt
        self.assertEqual(len(maps), 2)

    def test_the_result_is_the_same_as_inflating_directly(self):
        grid = self.one_block()
        self.assertEqual(blocked_cells(InflatedMaps(grid).get(1.6)), blocked_cells(inflate(grid, 1.6)))


def warehouse_grid(size=300, res=0.05):
    """A size x size grid with racks and walls, roughly like the Unity warehouse."""
    rows = [["."] * size for _ in range(size)]
    for k in range(size):  # outer walls
        rows[0][k] = rows[size - 1][k] = rows[k][0] = rows[k][size - 1] = "#"
    for rack_x in range(30, size - 40, 60):  # racks: 20 cells wide, 160 cells long, aisles between
        for i in range(rack_x, rack_x + 20):
            for j in range(40, size - 40):
                rows[j][i] = "#"
    return make_grid(["".join(r) for r in rows], res=res)


class PerformanceTest(unittest.TestCase):
    """0.05 m cells make ~90 000 cells. The bounds are loose: they catch a slowdown of a whole order."""

    def test_inflating_a_warehouse_sized_map_is_fast(self):
        grid = warehouse_grid()
        t0 = time.time()
        inflated = inflate(grid, 0.35)
        self.assertLess(time.time() - t0, 3.0)
        self.assertGreater(len(blocked_cells(inflated)), len(blocked_cells(grid)))

    def test_planning_across_a_warehouse_sized_map_is_fast(self):
        grid = warehouse_grid()
        start, goal = (0.5, 0.5), (14.5, 14.5)  # corner to corner, around the racks
        t0 = time.time()
        corners = plan(grid, start, goal, 2.0, inflation_radius=0.35)
        self.assertLess(time.time() - t0, 8.0)
        self.assertGreater(len(corners), 2)

    def test_plan_inflated_gives_the_same_path_as_plan(self):
        grid = warehouse_grid(size=120)
        start, goal = (0.4, 0.4), (5.6, 5.6)
        direct = plan(grid, start, goal, 2.0, inflation_radius=0.35)
        again = plan_inflated(inflate(grid, 0.35), start, goal, 2.0)
        self.assertEqual(direct, again)


class AStarTest(unittest.TestCase):
    def test_straight_corridor(self):
        self.assertEqual(astar(make_grid(["....."]), 0, 4), [0, 1, 2, 3, 4])

    def test_goes_around_a_wall(self):
        grid = make_grid([".#.", ".#.", "..."])
        path = astar(grid, 6, 8)  # (0, 2) -> (2, 2)
        self.assertEqual((path[0], path[-1]), (6, 8))
        wall = blocked_cells(grid)
        self.assertTrue(all((c % 3, c // 3) not in wall for c in path))
        self.assertTrue(any(c // 3 == 0 for c in path))  # through the bottom row
        for a, b in zip(path, path[1:]):
            self.assertLessEqual(max(abs(a % 3 - b % 3), abs(a // 3 - b // 3)), 1)

    def test_no_route_returns_none(self):
        self.assertIsNone(astar(make_grid([".#.", ".#.", ".#."]), 0, 2))

    def test_never_squeezes_diagonally_between_two_blocked_cells(self):
        self.assertIsNone(astar(make_grid(["#.", ".#"]), 0, 3))

    def test_uses_diagonals_in_open_space(self):
        self.assertEqual(len(astar(make_grid(["...."] * 4), 0, 15)), 4)


class LineOfSightTest(unittest.TestCase):
    def test_clear_in_open_space(self):
        grid = make_grid(["...."] * 4)
        self.assertTrue(line_of_sight(grid, (0.5, 0.5), (3.5, 2.5)))

    def test_blocked_by_a_cell_on_the_segment(self):
        grid = make_grid(["...", ".#.", "..."])
        self.assertFalse(line_of_sight(grid, (0.5, 1.5), (2.5, 1.5)))

    def test_blocked_by_a_corner_that_the_segment_only_clips(self):
        # The segment enters blocked cell (1, 1) at x = 1.914 and leaves it at
        # x = 2.0. Sampling every half cell steps right over that.
        grid = make_grid(["....", ".#..", "...."])
        self.assertFalse(line_of_sight(grid, (0.2, 0.5), (2.6, 1.2)))

    def test_a_diagonal_through_a_corner_is_blocked_by_either_side_cell(self):
        # (0.5, 0.5) -> (2.5, 2.5) passes through the lattice point (1, 1),
        # where cell (1, 0) is on one side. A* never cuts such a corner either.
        grid = make_grid(["...", "...", ".#."])
        self.assertFalse(line_of_sight(grid, (0.5, 0.5), (2.5, 2.5)))

    def test_a_segment_leaving_the_grid_is_not_visible(self):
        grid = make_grid(["..."])
        self.assertFalse(line_of_sight(grid, (0.5, 0.5), (5.5, 0.5)))

    def test_the_result_does_not_depend_on_the_direction(self):
        rng = random.Random(1)
        for _ in range(200):
            rows = random_rows(rng, 0.2)
            grid = make_grid(rows)
            a, b = random_point(rng), random_point(rng)
            forward, backward = line_of_sight(grid, a, b), line_of_sight(grid, b, a)
            self.assertEqual(forward, backward, (rows, a, b))

    def test_agrees_with_an_exact_geometric_check(self):
        rng = random.Random(7)
        seen = set()
        for _ in range(300):
            rows = random_rows(rng, 0.25)
            grid = make_grid(rows)
            a, b = random_point(rng), random_point(rng)
            cells = blocked_cells(grid)
            expected = not any(segment_touches_cell(a, b, c) for c in cells)
            self.assertEqual(line_of_sight(grid, a, b), expected, (rows, a, b))
            seen.add(expected)
        self.assertEqual(seen, {True, False})  # the random cases cover both answers


class SmoothTest(unittest.TestCase):
    def test_collinear_points_collapse_to_the_ends(self):
        grid = make_grid(["...."])
        points = [(0.5, 0.5), (1.5, 0.5), (2.5, 0.5), (3.5, 0.5)]
        self.assertEqual(smooth(grid, points), [(0.5, 0.5), (3.5, 0.5)])

    def test_keeps_the_corners_that_the_walls_force(self):
        grid = make_grid(["...", "##.", "..."])
        points = [
            (0.5, 0.5), (1.5, 0.5), (2.5, 0.5), (2.5, 1.5),
            (2.5, 2.5), (1.5, 2.5), (0.5, 2.5),
        ]
        self.assertEqual(
            smooth(grid, points),
            [(0.5, 0.5), (2.5, 0.5), (2.5, 2.5), (0.5, 2.5)],
        )


class PlanTest(unittest.TestCase):
    def test_open_grid_gives_a_straight_line_with_the_exact_ends(self):
        grid = make_grid(["......"] * 6)
        corners = plan(grid, (0.3, 0.4), (5.2, 4.9), snap_distance=2.0)
        self.assertEqual(corners, [(0.3, 0.4), (5.2, 4.9)])

    def test_an_exact_goal_on_a_grid_line_is_kept(self):
        # With 0.05 m cells, x = 9.0 lies exactly on a cell border. The goal is free: it must not move.
        grid = make_grid(["." * 200] * 160, res=0.05)
        corners = plan(grid, (1.0, 1.0), (9.0, 7.0), snap_distance=2.0)
        self.assertEqual(corners[-1], (9.0, 7.0))
        self.assertEqual(corners[0], (1.0, 1.0))

    def test_goes_around_a_wall_and_every_leg_is_clear(self):
        grid = make_grid(["...#....", "...#....", "...#....", "........"])
        corners = plan(grid, (0.5, 3.5), (7.5, 3.5), snap_distance=2.0)
        self.assertEqual((corners[0], corners[-1]), ((0.5, 3.5), (7.5, 3.5)))
        self.assertGreater(len(corners), 2)
        for a, b in zip(corners, corners[1:]):
            self.assertTrue(line_of_sight(grid, a, b))

    def test_a_goal_inside_an_obstacle_snaps_to_a_free_cell(self):
        grid = make_grid(["......", "..##..", "..##..", "......"])
        corners = plan(grid, (0.5, 0.5), (2.9, 1.9), snap_distance=2.0)
        self.assertNotEqual(corners[-1], (2.9, 1.9))
        self.assertEqual(grid.data[cell_index(grid, *corners[-1])], 0)

    def test_a_goal_too_far_from_free_space_is_no_path(self):
        grid = make_grid(["....####"])
        with self.assertRaises(NoPath):
            plan(grid, (0.5, 0.5), (7.5, 0.5), snap_distance=2.0)

    def test_a_walled_off_goal_is_no_path(self):
        grid = make_grid(["..#.....", "..#.....", "..#....."])
        with self.assertRaises(NoPath):
            plan(grid, (0.5, 1.5), (6.5, 1.5), snap_distance=1.0)

    def test_a_robot_with_no_free_cell_nearby_is_no_path(self):
        with self.assertRaises(NoPath):
            plan(make_grid(["###", "###"]), (1.0, 1.0), (2.0, 1.0), snap_distance=5.0)

    def test_coordinates_that_are_not_finite_are_no_path(self):
        grid = make_grid(["..."])
        for bad in (math.nan, math.inf):
            with self.assertRaises(NoPath):
                plan(grid, (0.5, 0.5), (bad, 0.5), snap_distance=2.0)
            with self.assertRaises(NoPath):
                plan(grid, (bad, 0.5), (1.5, 0.5), snap_distance=2.0)

    def test_the_grid_is_not_modified(self):
        grid = make_grid(["...#....", "........", "........", "........"])
        before = list(grid.data)
        corners = plan(grid, (0.5, 3.5), (7.5, 3.5), 2.0, inflation_radius=1.0)
        self.assertGreater(len(corners), 2)  # it really had to inflate and go around
        self.assertEqual(list(grid.data), before)

    def test_a_robot_already_at_the_goal_ends_on_the_goal(self):
        grid = make_grid(["...", "...", "..."])
        corners = plan(grid, (1.2, 1.2), (1.4, 1.3), snap_distance=2.0)
        self.assertEqual(corners[-1], (1.4, 1.3))


class PlanInflationTest(unittest.TestCase):
    def test_every_corner_stays_the_inflation_radius_away_from_obstacles(self):
        rows = ["............"] * 8
        for r in range(2, 6):
            rows[r] = "....####...."
        grid = make_grid(rows)
        radius = 1.5
        corners = plan(grid, (1.5, 3.5), (10.5, 3.5), 2.0, inflation_radius=radius)
        self.assertGreater(len(corners), 2)
        for corner in corners:
            for cell in blocked_cells(grid):
                gap = edge_distance(corner, cell)
                self.assertGreaterEqual(gap, radius - 1e-9, (corner, cell))

    def test_inflation_closes_a_gap_that_is_too_narrow(self):
        grid = make_grid(["...#...", "...#...", "...#...", "......."])
        start, goal = (0.5, 0.5), (6.5, 0.5)
        self.assertGreater(len(plan(grid, start, goal, 1.0)), 1)
        with self.assertRaises(NoPath):
            plan(grid, start, goal, 1.0, inflation_radius=1.0)

    def test_inflation_leaves_a_wide_gap_open(self):
        rows = ["...#..."] * 7
        for r in (2, 3, 4):
            rows[r] = "......."
        grid = make_grid(rows)
        corners = plan(grid, (0.5, 3.5), (6.5, 3.5), 1.0, inflation_radius=1.0)
        self.assertEqual(corners, [(0.5, 3.5), (6.5, 3.5)])

    def test_a_start_inside_the_inflated_zone_is_kept_as_the_first_corner(self):
        grid = make_grid(["#.....", "#.....", "#....."])
        corners = plan(grid, (1.5, 1.5), (5.5, 1.5), 2.0, inflation_radius=1.0)
        self.assertEqual(corners[0], (1.5, 1.5))  # the robot's real position
        self.assertEqual(corners[1], (2.5, 1.5))  # nearest cell free after inflation


class AsciiMapTest(unittest.TestCase):
    def test_draws_start_path_and_goal(self):
        grid = make_grid(["...", "...", "..."])
        text = ascii_map(grid, [(0.5, 0.5), (2.5, 0.5)])
        self.assertEqual(text, "..G\n..*\n..S")

    def test_draws_obstacles(self):
        grid = make_grid(["#..", "...", "..."])
        text = ascii_map(grid, [(1.5, 1.5), (1.5, 0.5)])
        self.assertEqual(text, "...\n.SG\n#..")


if __name__ == "__main__":
    unittest.main()
