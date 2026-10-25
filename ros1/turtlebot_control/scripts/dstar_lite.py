"""D* Lite incremental path planning and incremental grid inflation.

Pure Python, no ROS imports. Implements Koenig & Likhachev D* Lite (backward search,
8-connected, octile heuristic) with incremental obstacle inflation and adaptive radii fallback.
"""
import heapq
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, FrozenSet, Iterable, List, Optional, Sequence, Set, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grid_planner import (  # noqa: E402
    Grid,
    InflatedMaps,
    NoPath,
    Point,
    cell_center,
    cell_index,
    nearest_free,
    reachable_from,
    smooth,
)

OCCUPIED = 100

# Exact tie-breaking: paths stay as optimal as astar's, because two different path
# costs differ by roughly 3e-4 m or more for paths under 1000 cells, while the
# rounding error is under 4e-7 m.
STRAIGHT = 10**9
DIAGONAL = 1414213562  # round(sqrt(2) * STRAIGHT)


# ---------------- 1. disc offsets ----------------
def disc_offsets(res: float, radius: float) -> List[Tuple[int, int]]:
    """List of (di, dj) cell offsets within obstacle inflation `radius`, including (0, 0).

    Matches the exact geometry of grid_planner.inflate:
      edge = hypot(max(|di|-0.5, 0), max(|dj|-0.5, 0)) * res
    An offset is included if edge < radius (and (0, 0) is always included).
    """
    if res <= 0.0:
        raise ValueError("grid resolution must be positive")
    offsets = [(0, 0)]
    if radius <= 0.0:
        return offsets
    reach = int(math.ceil(radius / res + 0.5))
    for dj in range(-reach, reach + 1):
        for di in range(-reach, reach + 1):
            if di == 0 and dj == 0:
                continue
            edge = math.hypot(max(abs(di) - 0.5, 0.0), max(abs(dj) - 0.5, 0.0)) * res
            if edge < radius:
                offsets.append((di, dj))
    return offsets


def expand_lidar_hit(
    grid: Grid,
    ci: int,
    cj: int,
    depth_cell: Optional[Tuple[int, int]],
    filter_static: bool,
) -> Tuple[List[Tuple[int, int]], bool]:
    """Expand a lidar hit into dilation (3x3) and extrusion cells, optionally filtering static hits.

    Args:
        grid: Raw static map (Grid).
        ci: Grid cell x-index of the lidar hit.
        cj: Grid cell y-index of the lidar hit.
        depth_cell: Optional (di, dj) cell extruded into obstacle volume.
        filter_static: If True, drop the hit if any cell in the in-bounds 3x3 neighbourhood is occupied.

    Returns:
        (cells_to_add, dropped): list of cells to add to dynamic overlay, and boolean dropped flag.
    """
    w, h = grid.width, grid.height
    if not (0 <= ci < w and 0 <= cj < h):
        return ([], False)

    if filter_static:
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                ni, nj = ci + di, cj + dj
                if 0 <= ni < w and 0 <= nj < h:
                    if grid.data[nj * w + ni] != 0:
                        return ([], True)

    cells: List[Tuple[int, int]] = []
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            ni, nj = ci + di, cj + dj
            if 0 <= ni < w and 0 <= nj < h:
                cells.append((ni, nj))

    if depth_cell is not None:
        di_depth, dj_depth = depth_cell
        if 0 <= di_depth < w and 0 <= dj_depth < h:
            cells.append((di_depth, dj_depth))

    return (cells, False)


def path_near_points(
    corners: Optional[Sequence[Point]],
    points: Sequence[Point],
    robot_xy: Point,
    radius: float,
    max_range: float = 3.5,
) -> bool:
    """Check if any obstacle point within max_range of robot is within radius of planned path."""
    if not corners or len(corners) < 2:
        return False

    rx, ry = robot_xy
    for k in range(len(corners) - 1):
        ax, ay = corners[k]
        bx, by = corners[k + 1]
        seg_dx = bx - ax
        seg_dy = by - ay
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 1e-4:
            continue

        for ox, oy in points:
            if math.hypot(ox - rx, oy - ry) > max_range:
                continue
            t = max(0.0, min(1.0, ((ox - ax) * seg_dx + (oy - ay) * seg_dy) / (seg_len * seg_len)))
            proj_x = ax + t * seg_dx
            proj_y = ay + t * seg_dy
            if math.hypot(ox - proj_x, oy - proj_y) < radius:
                return True
    return False


# ---------------- 2. incremental inflation ----------------
class IncrementalInflation:
    """Maintains an inflated grid incrementally under sparse dynamic obstacle updates.

    Static obstacles are pre-inflated and never modified. Dynamic obstacles are tracked
    with one cover count per cell (flat list). The resulting view.data matches
    inflate(overlay_obstacles(raw, centres), radius).
    """

    def __init__(self, static_inflated: Grid, radius: float) -> None:
        self.static_grid = static_inflated
        self.radius = radius
        self.width = static_inflated.width
        self.height = static_inflated.height
        self.res = static_inflated.res
        self.ox = static_inflated.ox
        self.oy = static_inflated.oy
        self._offsets = disc_offsets(self.res, radius)
        self._static_bytes = bytes(OCCUPIED if v != 0 else 0 for v in static_inflated.data)
        self.blocked = bytearray(self._static_bytes)
        self.view = Grid(
            self.blocked, self.width, self.height, self.res, self.ox, self.oy
        )
        # Flat cover counts: how many dynamic cells have this cell inside their disc.
        self._cover = [0] * (self.width * self.height)
        # The same offsets as linear index steps, valid away from the grid border.
        self._lin = [dj * self.width + di for di, dj in self._offsets]
        self._reach = max(max(abs(di), abs(dj)) for di, dj in self._offsets)
        self._current_cells: Set[Tuple[int, int]] = set()
        self.last_added: int = 0
        self.last_removed: int = 0

    def sync(self, cells: Iterable[Tuple[int, int]]) -> List[int]:
        """Make the dynamic overlay exactly `cells` and return indices of flipped cells."""
        w, h = self.width, self.height
        valid = {(i, j) for i, j in cells if 0 <= i < w and 0 <= j < h}
        added = valid - self._current_cells
        removed = self._current_cells - valid
        self._current_cells = valid
        self.last_added = len(added)
        self.last_removed = len(removed)

        cover, lin, reach, slow = self._cover, self._lin, self._reach, self._offsets
        candidates: List[int] = []  # cells whose count crossed 0 <-> 1 at least once

        for ci, cj in added:
            if reach <= ci < w - reach and reach <= cj < h - reach:
                base = cj * w + ci
                for off in lin:
                    idx = base + off
                    c = cover[idx]
                    cover[idx] = c + 1
                    if not c:
                        candidates.append(idx)
            else:
                for di, dj in slow:
                    ni, nj = ci + di, cj + dj
                    if 0 <= ni < w and 0 <= nj < h:
                        idx = nj * w + ni
                        c = cover[idx]
                        cover[idx] = c + 1
                        if not c:
                            candidates.append(idx)

        for ci, cj in removed:
            if reach <= ci < w - reach and reach <= cj < h - reach:
                base = cj * w + ci
                for off in lin:
                    idx = base + off
                    c = cover[idx]
                    cover[idx] = c - 1
                    if c == 1:
                        candidates.append(idx)
            else:
                for di, dj in slow:
                    ni, nj = ci + di, cj + dj
                    if 0 <= ni < w and 0 <= nj < h:
                        idx = nj * w + ni
                        c = cover[idx]
                        cover[idx] = c - 1
                        if c == 1:
                            candidates.append(idx)

        flipped: List[int] = []
        static, blocked = self._static_bytes, self.blocked
        for idx in set(candidates):
            now = OCCUPIED if (cover[idx] > 0 or static[idx]) else 0
            if blocked[idx] != now:
                blocked[idx] = now
                flipped.append(idx)
        return flipped


# ---------------- 3. D* Lite core ----------------
class DStarLite:
    """8-connected backward D* Lite search with octile heuristic and no corner-cutting."""

    def __init__(self, grid: Grid, start_cell: int, goal_cell: int) -> None:
        self.grid = grid
        self.width = grid.width
        self.height = grid.height
        self.start = start_cell
        self.goal = goal_cell
        self.last = start_cell
        self.km = 0
        self.expanded = 0

        n = self.width * self.height
        self.g = [float("inf")] * n
        self.rhs = [float("inf")] * n
        self.queued: Dict[int, Tuple[float, float]] = {}
        self.heap: List[Tuple[Tuple[float, float], int]] = []

        self.rhs[self.goal] = 0
        k = self._calc_key(self.goal)
        self.queued[self.goal] = k
        heapq.heappush(self.heap, (k, self.goal))

    def _h(self, a: int, b: int) -> int:
        w = self.width
        dx = abs(a % w - b % w)
        dy = abs(a // w - b // w)
        return STRAIGHT * (dx + dy) + (DIAGONAL - 2 * STRAIGHT) * min(dx, dy)

    def _calc_key(self, u: int) -> Tuple[float, float]:
        m = min(self.g[u], self.rhs[u])
        return (m + self._h(self.start, u) + self.km, m)

    def _calc_rhs(self, u: int) -> float:
        if u == self.goal:
            return 0
        data = self.grid.data
        if data[u] != 0:
            return float("inf")
        w, h = self.width, self.height
        cx, cy = u % w, u // w
        g = self.g
        best = float("inf")

        if 0 < cx < w - 1 and 0 < cy < h - 1:
            # Fast path for non-border cells: direct indexing, no bounds checks
            # 4 straight neighbours
            v = u - 1
            if data[v] == 0:
                val = STRAIGHT + g[v]
                if val < best:
                    best = val
            v = u + 1
            if data[v] == 0:
                val = STRAIGHT + g[v]
                if val < best:
                    best = val
            v = u - w
            if data[v] == 0:
                val = STRAIGHT + g[v]
                if val < best:
                    best = val
            v = u + w
            if data[v] == 0:
                val = STRAIGHT + g[v]
                if val < best:
                    best = val

            # 4 diagonal neighbours with no-corner-cutting check
            v = u - w - 1
            if data[v] == 0 and data[u - w] == 0 and data[u - 1] == 0:
                val = DIAGONAL + g[v]
                if val < best:
                    best = val
            v = u - w + 1
            if data[v] == 0 and data[u - w] == 0 and data[u + 1] == 0:
                val = DIAGONAL + g[v]
                if val < best:
                    best = val
            v = u + w - 1
            if data[v] == 0 and data[u + w] == 0 and data[u - 1] == 0:
                val = DIAGONAL + g[v]
                if val < best:
                    best = val
            v = u + w + 1
            if data[v] == 0 and data[u + w] == 0 and data[u + 1] == 0:
                val = DIAGONAL + g[v]
                if val < best:
                    best = val
            return best
        else:
            # General path for border cells
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        v = ny * w + nx
                        if data[v] != 0:
                            continue
                        diag = dx != 0 and dy != 0
                        if diag and (data[cy * w + nx] != 0 or data[ny * w + cx] != 0):
                            continue
                        cost = DIAGONAL if diag else STRAIGHT
                        val = cost + g[v]
                        if val < best:
                            best = val
            return best

    def _neighbours(self, u: int) -> Sequence[int]:
        w, h = self.width, self.height
        cx, cy = u % w, u // w
        if 0 < cx < w - 1 and 0 < cy < h - 1:
            return (
                u - 1,
                u + 1,
                u - w,
                u + w,
                u - w - 1,
                u - w + 1,
                u + w - 1,
                u + w + 1,
            )
        res = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    res.append(ny * w + nx)
        return res

    def update_vertex(self, u: int) -> None:
        if u != self.goal:
            self.rhs[u] = self._calc_rhs(u)
        if self.g[u] != self.rhs[u]:
            k = self._calc_key(u)
            self.queued[u] = k
            heapq.heappush(self.heap, (k, u))
        else:
            self.queued.pop(u, None)

    def cells_changed(self, cells: Iterable[int]) -> None:
        affected = set()
        for c in cells:
            affected.add(c)
            for n in self._neighbours(c):
                affected.add(n)
        for u in affected:
            self.update_vertex(u)

    def move_start(self, cell: int) -> None:
        self.km += self._h(self.last, cell)
        self.last = cell
        self.start = cell

    def compute(self) -> bool:
        while self.heap:
            # Drop stale heap entries from the top before comparing
            top_k, top_u = self.heap[0]
            if self.queued.get(top_u) != top_k:
                heapq.heappop(self.heap)
                continue

            start_k = self._calc_key(self.start)
            if not (top_k < start_k or self.rhs[self.start] != self.g[self.start]):
                break

            k_old, u = heapq.heappop(self.heap)
            if self.queued.get(u) != k_old:
                continue
            k_new = self._calc_key(u)
            if k_old < k_new:
                self.queued[u] = k_new
                heapq.heappush(self.heap, (k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.expanded += 1
                self.g[u] = self.rhs[u]
                self.queued.pop(u, None)
                for v in self._neighbours(u):
                    self.update_vertex(v)
            else:
                self.expanded += 1
                self.g[u] = float("inf")
                self.update_vertex(u)
                for v in self._neighbours(u):
                    self.update_vertex(v)
            if len(self.heap) > 4 * len(self.queued) + 1000:
                self.heap = [(k, u) for u, k in self.queued.items()]
                heapq.heapify(self.heap)
        return self.g[self.start] != float("inf")

    def path_cells(self) -> Optional[List[int]]:
        if self.g[self.start] == float("inf"):
            return None
        w = self.width
        data = self.grid.data
        curr = self.start
        path = [curr]
        limit = self.width * self.height
        seen = {curr}
        while curr != self.goal:
            cx, cy = curr % w, curr // w
            best_v = None
            best_cost = float("inf")
            for v in self._neighbours(curr):
                if data[v] != 0:
                    continue
                nx, ny = v % w, v // w
                diag = nx != cx and ny != cy
                if diag and (data[cy * w + nx] != 0 or data[ny * w + cx] != 0):
                    continue
                c = (DIAGONAL if diag else STRAIGHT) + self.g[v]
                if c < best_cost:
                    best_cost = c
                    best_v = v
            if best_v is None or best_v in seen or len(path) > limit:
                return None
            curr = best_v
            seen.add(curr)
            path.append(curr)
        return path


# ---------------- 4. DStarLitePlanner ----------------
@dataclass
class PlanStats:
    radius: float
    level: int
    rebuilt: bool
    expanded: int
    changed_cells: int
    ms: float
    cpu_ms: float = 0.0
    build_ms: float = 0.0
    sync_ms: float = 0.0
    cells_ms: float = 0.0
    compute_ms: float = 0.0
    path_ms: float = 0.0
    format_ms: float = 0.0
    raw_added: int = 0
    raw_removed: int = 0


@dataclass
class _SearchState:
    goal: Point
    snap_distance: float
    radii: Tuple[float, ...]
    level: int
    inc: IncrementalInflation
    dstar: DStarLite
    start_cell: int
    goal_cell: int
    last_probe: float


class DStarLitePlanner:
    """Incremental replanner with D* Lite, multi-stage adaptive inflation, and probe widening."""

    def __init__(self, maps: InflatedMaps, widen_every: float = 5.0) -> None:
        self.maps = maps
        self.widen_every = widen_every
        self.obstacle_cells: FrozenSet[Tuple[int, int]] = frozenset()
        self.stats: Optional[PlanStats] = None
        self._state: Optional[_SearchState] = None
        self._lock = threading.Lock()
        self._obs_lock = threading.Lock()

    def set_obstacles(self, cells: Iterable[Tuple[int, int]]) -> None:
        with self._obs_lock:
            self.obstacle_cells = frozenset(cells)

    def view_snapshot(self) -> Optional[Grid]:
        with self._lock:
            if self._state is None:
                return None
            inc = self._state.inc
            return Grid(bytes(inc.blocked), inc.width, inc.height, inc.res, inc.ox, inc.oy)

    def plan(
        self,
        start: Point,
        goal: Point,
        snap_distance: float,
        radii: Sequence[float],
        now: float = 0.0,
    ) -> List[Point]:
        # Read the obstacle frozenset under the small lock, then work without holding it
        with self._obs_lock:
            obs = self.obstacle_cells

        with self._lock:
            t0 = time.monotonic()
            cpu0 = time.thread_time()
            if not all(math.isfinite(v) for v in (*start, *goal)):
                raise NoPath("start or goal is not a finite number")

            radii_tuple = tuple(radii)
            state = self._state
            can_reuse = (
                state is not None
                and state.goal == goal
                and state.snap_distance == snap_distance
                and state.radii == radii_tuple
            )

            # Probe widening if operating at a narrower fallback radius
            if can_reuse and state is not None and state.level > 0:
                if now - state.last_probe >= self.widen_every:
                    t_bld0 = time.perf_counter()
                    wider_lvl = state.level - 1
                    wider_r = radii_tuple[wider_lvl]
                    w_inc = IncrementalInflation(self.maps.get(wider_r), wider_r)
                    w_inc.sync(obs)
                    w_s = nearest_free(w_inc.view, start[0], start[1], snap_distance)
                    if w_s >= 0:
                        w_reach = reachable_from(w_inc.view, w_s)
                        w_g = nearest_free(w_inc.view, goal[0], goal[1], snap_distance, w_reach)
                        if w_g >= 0:
                            w_dsl = DStarLite(w_inc.view, w_s, w_g)
                            build_ms = (time.perf_counter() - t_bld0) * 1000.0

                            t_comp0 = time.perf_counter()
                            comp_ok = w_dsl.compute()
                            compute_ms = (time.perf_counter() - t_comp0) * 1000.0

                            if comp_ok:
                                t_path0 = time.perf_counter()
                                cells = w_dsl.path_cells()
                                path_ms = (time.perf_counter() - t_path0) * 1000.0

                                if cells is not None:
                                    self._state = _SearchState(
                                        goal=goal,
                                        snap_distance=snap_distance,
                                        radii=radii_tuple,
                                        level=wider_lvl,
                                        inc=w_inc,
                                        dstar=w_dsl,
                                        start_cell=w_s,
                                        goal_cell=w_g,
                                        last_probe=now,
                                    )
                                    t_fmt0 = time.perf_counter()
                                    path = self._format_path(w_inc.view, cells, start, goal, w_s, w_g)
                                    format_ms = (time.perf_counter() - t_fmt0) * 1000.0

                                    self.stats = PlanStats(
                                        radius=wider_r,
                                        level=wider_lvl,
                                        rebuilt=True,
                                        expanded=w_dsl.expanded,
                                        changed_cells=0,
                                        ms=(time.monotonic() - t0) * 1000.0,
                                        cpu_ms=(time.thread_time() - cpu0) * 1000.0,
                                        build_ms=build_ms,
                                        sync_ms=0.0,
                                        cells_ms=0.0,
                                        compute_ms=compute_ms,
                                        path_ms=path_ms,
                                        format_ms=format_ms,
                                        raw_added=w_inc.last_added,
                                        raw_removed=w_inc.last_removed,
                                    )
                                    return path
                    state.last_probe = now

            # Incremental reuse attempt
            if can_reuse and state is not None:
                t_sync0 = time.perf_counter()
                flipped = state.inc.sync(obs)
                sync_ms = (time.perf_counter() - t_sync0) * 1000.0

                if state.inc.view.data[state.goal_cell] == 0:
                    t_cells0 = time.perf_counter()
                    state.dstar.cells_changed(flipped)
                    cells_ms = (time.perf_counter() - t_cells0) * 1000.0

                    new_start = nearest_free(state.inc.view, start[0], start[1], snap_distance)
                    if new_start < 0:
                        # Fall through to narrower radii instead of raising NoPath
                        start_lvl = state.level + 1
                    else:
                        if new_start != state.start_cell:
                            state.dstar.move_start(new_start)
                            state.start_cell = new_start
                        prev_exp = state.dstar.expanded

                        t_comp0 = time.perf_counter()
                        comp_ok = state.dstar.compute()
                        compute_ms = (time.perf_counter() - t_comp0) * 1000.0

                        if comp_ok:
                            t_path0 = time.perf_counter()
                            cells = state.dstar.path_cells()
                            path_ms = (time.perf_counter() - t_path0) * 1000.0

                            if cells is not None:
                                t_fmt0 = time.perf_counter()
                                path = self._format_path(
                                    state.inc.view,
                                    cells,
                                    start,
                                    goal,
                                    state.start_cell,
                                    state.goal_cell,
                                )
                                format_ms = (time.perf_counter() - t_fmt0) * 1000.0

                                self.stats = PlanStats(
                                    radius=radii_tuple[state.level],
                                    level=state.level,
                                    rebuilt=False,
                                    expanded=state.dstar.expanded - prev_exp,
                                    changed_cells=len(flipped),
                                    ms=(time.monotonic() - t0) * 1000.0,
                                    cpu_ms=(time.thread_time() - cpu0) * 1000.0,
                                    build_ms=0.0,
                                    sync_ms=sync_ms,
                                    cells_ms=cells_ms,
                                    compute_ms=compute_ms,
                                    path_ms=path_ms,
                                    format_ms=format_ms,
                                    raw_added=state.inc.last_added,
                                    raw_removed=state.inc.last_removed,
                                )
                                return path
                        # Route blocked at this level: fall back to narrower radii
                        start_lvl = state.level + 1
                else:
                    # Goal became blocked by dynamic overlay: rebuild and re-snap
                    start_lvl = 0
            else:
                start_lvl = 0

            # Fresh build or fallback across radii
            for lvl in range(start_lvl, len(radii_tuple)):
                t_bld0 = time.perf_counter()
                r = radii_tuple[lvl]
                inc = IncrementalInflation(self.maps.get(r), r)
                inc.sync(obs)
                s_cell = nearest_free(inc.view, start[0], start[1], snap_distance)
                if s_cell < 0:
                    continue
                reach = reachable_from(inc.view, s_cell)
                g_cell = nearest_free(inc.view, goal[0], goal[1], snap_distance, reach)
                if g_cell < 0:
                    continue
                dsl = DStarLite(inc.view, s_cell, g_cell)
                build_ms = (time.perf_counter() - t_bld0) * 1000.0

                t_comp0 = time.perf_counter()
                comp_ok = dsl.compute()
                compute_ms = (time.perf_counter() - t_comp0) * 1000.0

                if comp_ok:
                    t_path0 = time.perf_counter()
                    cells = dsl.path_cells()
                    path_ms = (time.perf_counter() - t_path0) * 1000.0

                    if cells is not None:
                        self._state = _SearchState(
                            goal=goal,
                            snap_distance=snap_distance,
                            radii=radii_tuple,
                            level=lvl,
                            inc=inc,
                            dstar=dsl,
                            start_cell=s_cell,
                            goal_cell=g_cell,
                            last_probe=now,
                        )
                        t_fmt0 = time.perf_counter()
                        path = self._format_path(inc.view, cells, start, goal, s_cell, g_cell)
                        format_ms = (time.perf_counter() - t_fmt0) * 1000.0

                        self.stats = PlanStats(
                            radius=r,
                            level=lvl,
                            rebuilt=True,
                            expanded=dsl.expanded,
                            changed_cells=0,
                            ms=(time.monotonic() - t0) * 1000.0,
                            cpu_ms=(time.thread_time() - cpu0) * 1000.0,
                            build_ms=build_ms,
                            sync_ms=0.0,
                            cells_ms=0.0,
                            compute_ms=compute_ms,
                            path_ms=path_ms,
                            format_ms=format_ms,
                            raw_added=inc.last_added,
                            raw_removed=inc.last_removed,
                        )
                        return path

            raise NoPath("No path found around obstacles")

    def _format_path(
        self,
        grid: Grid,
        cells: List[int],
        start: Point,
        goal: Point,
        start_cell: int,
        goal_cell: int,
    ) -> List[Point]:
        points = [cell_center(grid, c) for c in cells]
        if start_cell == cell_index(grid, start[0], start[1]):
            points[0] = (start[0], start[1])
        else:
            points.insert(0, (start[0], start[1]))
        if goal_cell == cell_index(grid, goal[0], goal[1]):
            points[-1] = (goal[0], goal[1])
        return smooth(grid, points)
