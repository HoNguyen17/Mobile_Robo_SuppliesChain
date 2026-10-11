"""Shared test helpers: put scripts/ on sys.path and build grids from ASCII art."""
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
