"""Shared test helpers: put scripts/ on sys.path and build grids from ASCII art."""
import math
import os
import sys
from typing import List

SCRIPTS = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts")
)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from grid_planner import Grid  # noqa: E402


def make_grid(
    rows: List[str], res: float = 1.0, ox: float = 0.0, oy: float = 0.0
) -> Grid:
    """Build a Grid from ASCII rows. '#' is blocked, '.' is free.

    The FIRST row is the TOP of the picture (largest y), so cell (i, j) is
    rows[height - 1 - j][i] and its centre is
    (ox + (i + .5) * res, oy + (j + .5) * res).
    """
    height, width = len(rows), len(rows[0])
    data = [0] * (width * height)
    for r, row in enumerate(rows):
        for i, ch in enumerate(row):
            if ch == "#":
                data[(height - 1 - r) * width + i] = 100
    return Grid(data, width, height, res, ox, oy)


def blocked_cells(grid: Grid) -> set:
    """Set of (i, j) of every blocked cell."""
    return {
        (idx % grid.width, idx // grid.width)
        for idx, value in enumerate(grid.data)
        if value != 0
    }


def rooms(width_m: float, height_m: float, res: float, obstacles) -> Grid:
    """A Grid of width_m x height_m metres. `obstacles` are (x0, y0, x1, y1) rectangles in metres."""
    width, height = int(round(width_m / res)), int(round(height_m / res))
    data = [0] * (width * height)
    for x0, y0, x1, y1 in obstacles:
        for j in range(max(int(round(y0 / res)), 0), min(int(round(y1 / res)), height)):
            for i in range(max(int(round(x0 / res)), 0), min(int(round(x1 / res)), width)):
                data[j * width + i] = 100
    return Grid(data, width, height, res)


def clearance(grid: Grid, x: float, y: float, reach: float = 0.8) -> float:
    """Distance from the point to the nearest blocked cell's edge, or `reach` if none is that close."""
    res = grid.res
    i0, i1 = int((x - reach) / res) - 1, int((x + reach) / res) + 1
    j0, j1 = int((y - reach) / res) - 1, int((y + reach) / res) + 1
    best = reach
    for j in range(max(j0, 0), min(j1, grid.height - 1) + 1):
        for i in range(max(i0, 0), min(i1, grid.width - 1) + 1):
            if grid.data[j * grid.width + i] == 0:
                continue
            dx = max(abs(x - (i + 0.5) * res) - res / 2.0, 0.0)
            dy = max(abs(y - (j + 0.5) * res) - res / 2.0, 0.0)
            best = min(best, math.hypot(dx, dy))
    return best
