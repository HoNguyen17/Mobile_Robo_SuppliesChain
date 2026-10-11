"""Grid planning for astar_planner: inflation, goal snapping, A* and smoothing.

No ROS imports, so it runs (and is unit tested) anywhere.

The grid has the layout of nav_msgs/OccupancyGrid: data[j * width + i] is the cell
whose lower-left corner is (ox + i * res, oy + j * res). Any non-zero value (100
occupied, -1 unknown) is an obstacle.
"""
import heapq
import math
from collections import deque
from dataclasses import dataclass
from typing import Iterator, List, Optional, Sequence, Tuple

SQRT2 = math.sqrt(2.0)
OCCUPIED = 100
# Two grid-line crossings closer than this (in segment length fractions) count as
# one crossing of a grid corner.
CORNER_EPS = 1e-9

Point = Tuple[float, float]


class NoPath(Exception):
    """The planner cannot produce a path. The message says why."""


@dataclass(frozen=True)
class Grid:
    data: Sequence[int]
    width: int
    height: int
    res: float
    ox: float = 0.0
    oy: float = 0.0

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid width and height must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError(
                "grid has %d cells but %d x %d were expected"
                % (len(self.data), self.width, self.height)
            )
        if not self.res > 0.0:
            raise ValueError("grid resolution must be positive")


# ---------------- cells ----------------
def cell_index(grid: Grid, x: float, y: float) -> int:
    """Index of the cell that contains (x, y), or -1 outside the grid."""
    i = int(math.floor((x - grid.ox) / grid.res))
    j = int(math.floor((y - grid.oy) / grid.res))
    if i < 0 or j < 0 or i >= grid.width or j >= grid.height:
        return -1
    return j * grid.width + i


def cell_center(grid: Grid, idx: int) -> Point:
    i, j = idx % grid.width, idx // grid.width
    return (grid.ox + (i + 0.5) * grid.res, grid.oy + (j + 0.5) * grid.res)


def is_blocked(grid: Grid, idx: int) -> bool:
    return grid.data[idx] != 0


def nearest_free(
    grid: Grid,
    x: float,
    y: float,
    max_dist: float,
    allowed: Optional[Sequence[bool]] = None,
) -> int:
    """Free cell whose centre is nearest to (x, y) and at most max_dist away, else -1.

    The search area is a circle. If `allowed` is given, the cell must be True there too.
    """
    res, w = grid.res, grid.width
    i_lo = max(int(math.floor((x - max_dist - grid.ox) / res)), 0)
    i_hi = min(int(math.floor((x + max_dist - grid.ox) / res)), w - 1)
    j_lo = max(int(math.floor((y - max_dist - grid.oy) / res)), 0)
    j_hi = min(int(math.floor((y + max_dist - grid.oy) / res)), grid.height - 1)
    limit = max_dist * max_dist
    best, best_sqr = -1, float("inf")
    for j in range(j_lo, j_hi + 1):
        for i in range(i_lo, i_hi + 1):
            idx = j * w + i
            if grid.data[idx] != 0 or (allowed is not None and not allowed[idx]):
                continue
            px, py = cell_center(grid, idx)
            sqr = (px - x) ** 2 + (py - y) ** 2
            if sqr <= limit and sqr < best_sqr:
                best, best_sqr = idx, sqr
    return best


def reachable_from(grid: Grid, start: int) -> List[bool]:
    """Flag every free cell that can be reached from `start` (4-connected)."""
    w, h = grid.width, grid.height
    visited = [False] * (w * h)
    visited[start] = True
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        cx, cz = cell % w, cell // w
        for nx, nz in ((cx + 1, cz), (cx - 1, cz), (cx, cz + 1), (cx, cz - 1)):
            if nx < 0 or nz < 0 or nx >= w or nz >= h:
                continue
            ni = nz * w + nx
            if visited[ni] or is_blocked(grid, ni):
                continue
            visited[ni] = True
            queue.append(ni)
    return visited


# ---------------- inflation ----------------
def inflate(grid: Grid, radius: float) -> Grid:
    """New grid in which every cell closer than `radius` to an obstacle is blocked too.

    The distance runs from the cell centre to the obstacle cell's edge, so a robot
    centre exactly `radius` away is allowed, and a radius of half a cell or less
    changes nothing. The input grid is not modified.
    """
    if radius <= 0.0:
        return grid
    reach = int(math.ceil(radius / grid.res + 0.5))
    offsets = [
        (di, dj)
        for dj in range(-reach, reach + 1)
        for di in range(-reach, reach + 1)
        if (di or dj)
        and math.hypot(max(abs(di) - 0.5, 0.0), max(abs(dj) - 0.5, 0.0)) * grid.res
        < radius
    ]
    if not offsets:
        return grid
    w, h = grid.width, grid.height
    data = list(grid.data)
    for idx, value in enumerate(grid.data):
        if value == 0:
            continue
        i, j = idx % w, idx // w
        for di, dj in offsets:
            ni, nj = i + di, j + dj
            if 0 <= ni < w and 0 <= nj < h:
                data[nj * w + ni] = OCCUPIED
    return Grid(data, w, h, grid.res, grid.ox, grid.oy)


# ---------------- A* ----------------
def _octile(grid: Grid, a: int, b: int) -> float:
    w = grid.width
    dx = abs(a % w - b % w)
    dz = abs(a // w - b // w)
    return (dx + dz) + (SQRT2 - 2.0) * min(dx, dz)


def astar(grid: Grid, start: int, goal: int) -> Optional[List[int]]:
    """8-connected A* (octile heuristic): cell indices from start to goal, or None.

    A diagonal step is refused when either side cell is blocked (no corner cutting).
    """
    w, h = grid.width, grid.height
    n = w * h
    g = [float("inf")] * n
    parent = [-1] * n
    closed = [False] * n
    g[start] = 0.0
    open_heap = [(_octile(grid, start, goal), start)]  # (f, cell): smallest f first

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if closed[current]:
            continue  # stale duplicate entry
        if current == goal:
            path = []
            while current != -1:
                path.append(current)
                current = parent[current]
            path.reverse()
            return path

        closed[current] = True
        cx, cz = current % w, current // w
        for dz in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dz == 0:
                    continue
                nx, nz = cx + dx, cz + dz
                if nx < 0 or nz < 0 or nx >= w or nz >= h:
                    continue
                ni = nz * w + nx
                if is_blocked(grid, ni) or closed[ni]:
                    continue
                diagonal = dx != 0 and dz != 0
                if diagonal and (
                    is_blocked(grid, cz * w + nx) or is_blocked(grid, nz * w + cx)
                ):
                    continue
                new_g = g[current] + (SQRT2 if diagonal else 1.0)
                if new_g < g[ni]:
                    g[ni] = new_g
                    parent[ni] = current
                    heapq.heappush(open_heap, (new_g + _octile(grid, ni, goal), ni))
    return None


# ---------------- line of sight and smoothing ----------------
def cells_on_segment(grid: Grid, a: Point, b: Point) -> Iterator[Tuple[int, int]]:
    """Yield (i, j) of every cell the segment a-b passes through, in order.

    A segment through a grid corner also yields the two cells beside that corner, so
    nothing it touches is missed. Cells outside the grid are yielded as well.
    """
    res = grid.res
    u, v = (a[0] - grid.ox) / res, (a[1] - grid.oy) / res
    du, dv = (b[0] - a[0]) / res, (b[1] - a[1]) / res
    i, j = int(math.floor(u)), int(math.floor(v))
    left_i = int(math.floor(u + du)) - i  # signed number of vertical lines crossed
    left_j = int(math.floor(v + dv)) - j  # signed number of horizontal lines crossed
    step_i, step_j = (1 if left_i > 0 else -1), (1 if left_j > 0 else -1)
    left_i, left_j = abs(left_i), abs(left_j)
    # Position along the segment (0 to 1) of the next crossing, and the gap after it.
    t_i = ((i + (du > 0)) - u) / du if left_i else math.inf
    t_j = ((j + (dv > 0)) - v) / dv if left_j else math.inf
    dt_i = 1.0 / abs(du) if left_i else math.inf
    dt_j = 1.0 / abs(dv) if left_j else math.inf

    yield i, j
    while left_i or left_j:
        if left_i and left_j and abs(t_i - t_j) < CORNER_EPS:
            yield i + step_i, j
            yield i, j + step_j
            i, j = i + step_i, j + step_j
            left_i, left_j = left_i - 1, left_j - 1
            t_i, t_j = t_i + dt_i, t_j + dt_j
        elif left_i and (not left_j or t_i < t_j):
            i, left_i, t_i = i + step_i, left_i - 1, t_i + dt_i
        else:
            j, left_j, t_j = j + step_j, left_j - 1, t_j + dt_j
        yield i, j


def line_of_sight(grid: Grid, a: Point, b: Point) -> bool:
    """True when the straight segment a-b touches only free cells inside the grid."""
    w, h = grid.width, grid.height
    return all(
        0 <= i < w and 0 <= j < h and grid.data[j * w + i] == 0
        for i, j in cells_on_segment(grid, a, b)
    )


def smooth(grid: Grid, points: List[Point]) -> List[Point]:
    """Keep only the points needed so every straight segment stays in free cells."""
    result = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not line_of_sight(grid, points[i], points[j]):
            j -= 1
        result.append(points[j])
        i = j
    return result


# ---------------- the whole plan ----------------
def plan(
    grid: Grid,
    start: Point,
    goal: Point,
    snap_distance: float,
    inflation_radius: float = 0.0,
) -> List[Point]:
    """Corner points of a smoothed A* path from start to goal; raise NoPath if none.

    The map is inflated by inflation_radius first. A start or goal that falls in a
    blocked cell is snapped to the nearest free cell within snap_distance (the goal
    only to one the robot can reach). Unsnapped ends keep their exact position.
    """
    if not all(math.isfinite(v) for v in (*start, *goal)):
        raise NoPath("start or goal is not a finite number")
    grid = inflate(grid, inflation_radius)

    start_cell = nearest_free(grid, start[0], start[1], snap_distance)
    if start_cell < 0:
        raise NoPath("no free cell within %.2f m of the robot" % snap_distance)
    goal_cell = nearest_free(
        grid, goal[0], goal[1], snap_distance, reachable_from(grid, start_cell)
    )
    if goal_cell < 0:
        raise NoPath("no reachable free cell within %.2f m of the goal" % snap_distance)
    cells = astar(grid, start_cell, goal_cell)
    if cells is None:
        raise NoPath("A* found no route")  # not expected after the flood fill

    points = [cell_center(grid, c) for c in cells]
    if start_cell == cell_index(grid, start[0], start[1]):
        points[0] = (start[0], start[1])
    else:
        # The robot stands in blocked space: it first drives to the free cell.
        points.insert(0, (start[0], start[1]))
    if goal_cell == cell_index(grid, goal[0], goal[1]):
        points[-1] = (goal[0], goal[1])
    return smooth(grid, points)


def ascii_map(grid: Grid, points: List[Point]) -> str:
    """Picture of the grid and a path: S start, * path, G goal, # obstacle, . free.

    Same orientation as map_viewer.py: the top is +x and the left is +y.
    """
    w, h = grid.width, grid.height
    marks = {}
    for a, b in zip(points, points[1:]):
        for i, j in cells_on_segment(grid, a, b):
            if 0 <= i < w and 0 <= j < h:
                marks[j * w + i] = "*"
    marks[cell_index(grid, *points[0])] = "S"
    marks[cell_index(grid, *points[-1])] = "G"
    lines = []
    for i in range(w - 1, -1, -1):
        line = ""
        for j in range(h - 1, -1, -1):
            idx = j * w + i
            line += marks.get(idx, "#" if is_blocked(grid, idx) else ".")
        lines.append(line)
    return "\n".join(lines)
