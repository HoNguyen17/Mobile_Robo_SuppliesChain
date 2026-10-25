#!/usr/bin/env python3
"""Benchmark path planners on a synthetic warehouse map.

Note: The grid is synthetic, so the numbers compare planners and are not simulation results.

Three planners are evaluated on identical inputs and trajectories:
  A = overlay + inflate the whole map + plan_inflated (what astar_planner does today)
  B = plan_inflated on IncrementalInflation.view
  C = DStarLitePlanner
"""
import argparse
import csv
import math
import os
import sys
import time
from typing import Dict, List, Optional, Set, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dstar_lite import DStarLitePlanner, IncrementalInflation  # noqa: E402
from grid_planner import (  # noqa: E402
    Grid,
    InflatedMaps,
    cell_center,
    inflate,
    overlay_obstacles,
    plan_inflated,
)


def build_synthetic_warehouse(
    w: int, h: int, res: float = 0.05, scenario: str = "gate"
) -> Tuple[Grid, Tuple[float, float], Tuple[float, float]]:
    """Build a synthetic warehouse with shelf rows (0.9m), aisles (1.6m), and cross aisles."""
    data = [0] * (w * h)

    # Outer perimeter walls
    for i in range(w):
        data[0 * w + i] = 100
        data[(h - 1) * w + i] = 100
    for j in range(h):
        data[j * w + 0] = 100
        data[j * w + (w - 1)] = 100

    # Main aisle at bottom: y < 2.0 m (j < 40)
    # Top clearance aisle: y > h*res - 2.0 m (j > h - 40)
    # Cross aisle at mid-height (j in [h//2 - 10 .. h//2 + 10])
    mid_j = h // 2
    j_cross_lo = mid_j - 10
    j_cross_hi = mid_j + 10
    j_bottom_shelf = 40
    j_top_shelf = h - 40

    # Shelf rows: 0.9 m (18 cells) wide; Aisles: 1.6 m (32 cells) wide -> pitch = 50 cells
    rack_w = 18
    pitch = 50
    for x_start in range(pitch, w - pitch, pitch):
        for i in range(x_start, min(x_start + rack_w, w - 1)):
            for j in range(j_bottom_shelf, j_cross_lo):
                data[j * w + i] = 100
            for j in range(j_cross_hi, j_top_shelf):
                data[j * w + i] = 100

    if scenario == "gate":
        # Static wall 2 cells thick at x about 11.75 m (i = 235, 236)
        # Racks occupy i = 50k .. 50k+17; columns 235..236 are in the aisle between rack 4 (200..217) and rack 5 (250..267)
        wall_cols = (235, 236)
        for i in wall_cols:
            for j in range(h):
                data[j * w + i] = 100

        # Three doors:
        # door A: y 9.4..10.6 m (1.2 m wide, in line with start and goal)
        # door B: y 18.0..19.6 m (top clearance aisle)
        # door C: y 0.4..1.9 m (bottom main aisle)
        door_A = range(int(round(9.4 / res)), int(round(10.6 / res)))
        door_B = range(int(round(18.0 / res)), int(round(19.6 / res)))
        door_C = range(int(round(0.4 / res)), int(round(1.9 / res)))
        for j in list(door_A) + list(door_B) + list(door_C):
            for i in wall_cols:
                data[j * w + i] = 0

        raw = Grid(data, w, h, res)
        start = (2.5, 10.0)
        goal = (27.0, 10.0)
        return raw, start, goal

    raw = Grid(data, w, h, res)
    start = (2.5, 1.0)
    goal = ((w - 60) * res, (h - 20) * res)
    return raw, start, goal


def percentile(data: List[float], p: float) -> float:
    if not data:
        return 0.0
    sorted_d = sorted(data)
    k = (len(sorted_d) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_d[int(k)]
    return sorted_d[int(f)] * (c - k) + sorted_d[int(c)] * (k - f)


def path_length(points: Optional[List[Tuple[float, float]]]) -> float:
    if not points or len(points) < 2:
        return 0.0
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def advance_along_path(
    path: List[Tuple[float, float]],
    curr_pos: Tuple[float, float],
    step_dist: float,
) -> Tuple[float, float]:
    """Advance curr_pos forward along path segments by step_dist."""
    if not path or len(path) < 2:
        return curr_pos

    # Find the closest segment
    best_idx = 0
    best_t = 0.0
    min_dist_sq = float("inf")
    cx, cy = curr_pos
    for k in range(len(path) - 1):
        ax, ay = path[k]
        bx, by = path[k + 1]
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-8:
            continue
        t = max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / seg_len_sq))
        px = ax + t * dx
        py = ay + t * dy
        d_sq = (cx - px) ** 2 + (cy - py) ** 2
        if d_sq < min_dist_sq:
            min_dist_sq = d_sq
            best_idx = k
            best_t = t

    # Advance step_dist forward starting from best_idx at best_t
    rem = step_dist
    ax, ay = path[best_idx]
    bx, by = path[best_idx + 1]
    dx, dy = bx - ax, by - ay
    seg_len = math.hypot(dx, dy)
    curr_dist_on_seg = best_t * seg_len
    if curr_dist_on_seg + rem <= seg_len:
        new_t = (curr_dist_on_seg + rem) / seg_len
        return (ax + new_t * dx, ay + new_t * dy)

    rem -= (seg_len - curr_dist_on_seg)
    for k in range(best_idx + 1, len(path) - 1):
        ax, ay = path[k]
        bx, by = path[k + 1]
        seg_len = math.hypot(bx - ax, by - ay)
        if rem <= seg_len:
            new_t = rem / seg_len
            return (ax + new_t * (bx - ax), ay + new_t * (by - ay))
        rem -= seg_len

    return path[-1]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark path planners on a synthetic warehouse map."
    )
    parser.add_argument("--width", type=int, default=600, help="Grid width in cells")
    parser.add_argument("--height", type=int, default=400, help="Grid height in cells")
    parser.add_argument("--steps", type=int, default=110, help="Simulation steps")
    parser.add_argument("--radius", type=float, default=0.35, help="Inflation radius (m)")
    parser.add_argument("--speed", type=float, default=0.26, help="Robot speed (m/s)")
    parser.add_argument("--scenario", type=str, default="gate", help="Scenario name (gate, synthetic)")
    parser.add_argument("--walker-from", type=int, default=30, help="Start step for dynamic obstacle")
    parser.add_argument("--walker-until", type=int, default=90, help="End step for dynamic obstacle")
    parser.add_argument("--csv", type=str, default="benchmark_planners.csv", help="CSV export filename")
    parser.add_argument("--trace", action="store_true", help="Print per-step diagnostic trace")
    args = parser.parse_args()

    res = 0.05
    dt = 0.5  # s per step
    snap_distance = 2.0
    step_advance = args.speed * dt

    print(f"================================================================================")
    print(f"BENCHMARK: Path Planners Comparison ({args.scenario})")
    print(f"Grid: {args.width}x{args.height} cells ({args.width*res:.1f}x{args.height*res:.1f} m, res={res} m)")
    print(f"Inflation radius: {args.radius:.2f} m | Robot speed: {args.speed:.2f} m/s | Steps: {args.steps}")
    print(f"Dynamic walker present: step {args.walker_from}..{args.walker_until}")
    print(f"================================================================================")

    raw, start_pos, goal_pos = build_synthetic_warehouse(
        args.width, args.height, res, scenario=args.scenario
    )
    maps = InflatedMaps(raw)

    if args.scenario == "gate":
        # Static wall at i = 235, 236 (x ~ 11.75 m)
        wall_cols = (235, 236)
        door_A = range(int(round(9.4 / res)), int(round(10.6 / res)))
        door_B_mid_y = (18.0 + 19.6) / 2.0
        door_C_mid_y = (0.4 + 1.9) / 2.0

        p_open = plan_inflated(maps.get(args.radius), start_pos, goal_pos, snap_distance)
        len_open = path_length(p_open)

        # Verify door B and door C passability
        p_door_B = plan_inflated(maps.get(args.radius), start_pos, (wall_cols[0] * res, door_B_mid_y), snap_distance)
        p_door_C = plan_inflated(maps.get(args.radius), start_pos, (wall_cols[0] * res, door_C_mid_y), snap_distance)

        # Check route with door A closed
        data_closed = list(raw.data)
        for j in door_A:
            for i in wall_cols:
                data_closed[j * args.width + i] = 100
        grid_closed = Grid(data_closed, args.width, args.height, res)
        maps_closed = InflatedMaps(grid_closed)
        p_closed = plan_inflated(maps_closed.get(args.radius), start_pos, goal_pos, snap_distance)
        len_closed = path_length(p_closed)

        print("\n--- Gate Scenario Verification ---")
        print(f"Door A passable: {p_open is not None}")
        print(f"Door B passable: {p_door_B is not None}")
        print(f"Door C passable: {p_door_C is not None}")
        print(f"Route with door A open:   {len_open:.2f} m")
        print(f"Route with door A closed: {len_closed:.2f} m")
        print(f"Route difference:         {len_closed - len_open:.2f} m (>= 12 m: {len_closed - len_open >= 12.0})")

    # Initial plan
    print("\n--- Initial Route Planning ---")
    t0 = time.perf_counter()
    init_path = plan_inflated(maps.get(args.radius), start_pos, goal_pos, snap_distance)
    init_plan_ms = (time.perf_counter() - t0) * 1000.0
    print(f"Initial route planned in {init_plan_ms:.1f} ms with {len(init_path)} corners:")
    for k, (x, y) in enumerate(init_path):
        print(f"  corner {k:2d}: x={x:6.2f} m, y={y:6.2f} m")

    if args.scenario == "gate":
        print(f"\nGate scenario dynamic walker:")
        print(f"Walker (11x11 cells) paces along y inside door A (x=11.75 m, y in [9.4, 10.6] m)")
        print(f"Walker present from step {args.walker_from} to step {args.walker_until}\n")
    else:
        # Find the aisle segment that the route actually uses.
        # We look for a segment where the robot travels mostly vertically (y changes > 2m)
        # which represents traversing an aisle between shelves.
        aisle_seg = None
        for a, b in zip(init_path, init_path[1:]):
            if abs(b[0] - a[0]) < 0.20 and math.hypot(b[0] - a[0], b[1] - a[1]) >= 2.0:
                aisle_seg = (a, b)
                break
        if aisle_seg is None:
            aisle_seg = (init_path[1], init_path[2])

        aisle_x = (aisle_seg[0][0] + aisle_seg[1][0]) / 2.0
        aisle_y_lo = min(aisle_seg[0][1], aisle_seg[1][1])
        aisle_y_hi = max(aisle_seg[0][1], aisle_seg[1][1])
        aisle_y_mid = (aisle_y_lo + aisle_y_hi) / 2.0

        aisle_ci = int(round(aisle_x / res))
        mid_j = int(round(aisle_y_mid / res))
        left_wall = aisle_ci
        while left_wall > 0 and raw.data[mid_j * args.width + left_wall] == 0:
            left_wall -= 1
        right_wall = aisle_ci
        while right_wall < args.width - 1 and raw.data[mid_j * args.width + right_wall] == 0:
            right_wall += 1
        aisle_width_cells = right_wall - left_wall - 1
        aisle_center_i = (left_wall + right_wall) // 2

        print(f"\nSelected aisle along x={aisle_x:.2f} m (y={aisle_y_lo:.2f}..{aisle_y_hi:.2f} m)")
        print(f"Aisle width: {aisle_width_cells} cells ({aisle_width_cells * res:.2f} m)")
        print(f"Aisle center cell i={aisle_center_i}, walls at i={left_wall}..{right_wall}")

        walker_y = aisle_y_mid
        walker_cj = int(round(walker_y / res))
        walker_i_lo = left_wall + 1
        walker_i_hi = right_wall - 1
        walker_j_half = 5
        walker_template = set()
        for di in range(walker_i_lo, walker_i_hi + 1):
            for dj in range(-walker_j_half, walker_j_half + 1):
                walker_template.add((di, walker_cj + dj))

        print(f"Walker fills aisle i={walker_i_lo}..{walker_i_hi}, j={walker_cj - walker_j_half}..{walker_cj + walker_j_half}")
        print(f"Walker template: {len(walker_template)} cells\n")

    # Initialize planners
    planner_C = DStarLitePlanner(maps)
    inc_B = IncrementalInflation(maps.get(args.radius), args.radius)

    trail: List[Tuple[Set[Tuple[int, int]], float]] = []
    curr_robot_pos = start_pos
    prev_obstacle_cells: Set[Tuple[int, int]] = set()

    # Compute A's path first at step 0 to use for driving the robot
    path_A_for_drive = init_path

    # Step logs
    step_records = []
    dstar_search_steps = []

    if args.trace:
        print(f"{'step':>5} {'n_obs':>6} {'robot_x':>8} {'robot_y':>8} {'len_A':>8} {'len_C':>8} {'C_exp':>8} {'near_A':>8}")
        print("-" * 72)

    print("Running benchmark steps...")
    for s in range(args.steps):
        sim_time = s * dt

        # Update dynamic walker trail
        if args.walker_from <= s <= args.walker_until:
            if args.scenario == "gate":
                wy = 10.0 + 0.25 * math.sin(s * 0.2)
                wj = int(round(wy / res))
                wi = 235
                walker_box = {
                    (wi + di, wj + dj)
                    for di in range(-5, 6)
                    for dj in range(-5, 6)
                }
                trail.append((walker_box, sim_time))
            else:
                trail.append((walker_template, sim_time))

        # Expire obstacles older than 6.0 s
        trail = [(box, t) for box, t in trail if t >= sim_time - 6.0]
        active_obstacles: Set[Tuple[int, int]] = set()
        for box, _ in trail:
            active_obstacles.update(box)

        changed = active_obstacles != prev_obstacle_cells
        prev_obstacle_cells = set(active_obstacles)

        # Benchmark Planner A (Raw overlay + full map inflation + plan_inflated)
        t_start = time.perf_counter()
        pts = [cell_center(raw, j * args.width + i) for i, j in active_obstacles]
        overlaid_grid = overlay_obstacles(raw, pts)
        inflated_A = inflate(overlaid_grid, args.radius)
        path_A = plan_inflated(inflated_A, curr_robot_pos, goal_pos, snap_distance)
        ms_A = (time.perf_counter() - t_start) * 1000.0

        # Benchmark Planner B (IncrementalInflation view + plan_inflated)
        t_start = time.perf_counter()
        inc_B.sync(active_obstacles)
        path_B = plan_inflated(inc_B.view, curr_robot_pos, goal_pos, snap_distance)
        ms_B = (time.perf_counter() - t_start) * 1000.0

        # Benchmark Planner C (DStarLitePlanner)
        t_start = time.perf_counter()
        planner_C.set_obstacles(active_obstacles)
        path_C = planner_C.plan(
            curr_robot_pos, goal_pos, snap_distance, [args.radius], now=sim_time
        )
        ms_C = (time.perf_counter() - t_start) * 1000.0

        c_exp = planner_C.stats.expanded if planner_C.stats else 0
        c_rebuilt = planner_C.stats.rebuilt if planner_C.stats else False

        len_A = path_length(path_A)
        len_C = path_length(path_C)
        ratio_CA = len_C / len_A if len_A > 0 else 1.0

        if c_exp > 0:
            dstar_search_steps.append((s, sim_time, c_exp, c_rebuilt, ms_C))

        step_records.append(
            {
                "step": s,
                "time_s": sim_time,
                "robot_x": curr_robot_pos[0],
                "robot_y": curr_robot_pos[1],
                "changed_obstacles": changed,
                "plan_A_ms": ms_A,
                "plan_B_ms": ms_B,
                "plan_C_ms": ms_C,
                "C_expanded": c_exp,
                "C_rebuilt": c_rebuilt,
                "len_A": len_A,
                "len_C": len_C,
                "ratio_CA": ratio_CA,
            }
        )

        if args.trace and s % 5 == 0 and 25 <= s <= 105:
            near_A = False
            if path_A and args.walker_from <= s <= args.walker_until:
                gate_x = 11.75 if args.scenario == "gate" else aisle_x
                gate_y = 10.0 if args.scenario == "gate" else walker_y
                for a, b in zip(path_A, path_A[1:]):
                    seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
                    if seg_len < 0.01:
                        continue
                    n_pts = max(int(seg_len / 0.05), 1)
                    for kk in range(n_pts + 1):
                        t = kk / n_pts
                        px = a[0] + t * (b[0] - a[0])
                        py = a[1] + t * (b[1] - a[1])
                        dist = math.hypot(px - gate_x, py - gate_y)
                        if dist < 0.4:
                            near_A = True
                            break
                    if near_A:
                        break
            print(
                f"{s:5d} {len(active_obstacles):6d} {curr_robot_pos[0]:8.2f} {curr_robot_pos[1]:8.2f} "
                f"{len_A:8.2f} {len_C:8.2f} {c_exp:8d} {'YES' if near_A else 'no':>8}"
            )

        # Advance robot forward along Planner A's path so all planners see the same positions
        curr_robot_pos = advance_along_path(path_A, curr_robot_pos, step_advance)

    # Compute summary statistics (exclude step 0)
    records_after_0 = step_records[1:]
    changed_records = [r for r in records_after_0 if r["changed_obstacles"]]
    unchanged_records = [r for r in records_after_0 if not r["changed_obstacles"]]

    def stat_triplet(records, key):
        vals = [r[key] for r in records]
        return percentile(vals, 50.0), percentile(vals, 95.0), max(vals) if vals else 0.0

    med_A_chg, p95_A_chg, max_A_chg = stat_triplet(changed_records, "plan_A_ms")
    med_B_chg, p95_B_chg, max_B_chg = stat_triplet(changed_records, "plan_B_ms")
    med_C_chg, p95_C_chg, max_C_chg = stat_triplet(changed_records, "plan_C_ms")

    med_A_unchg, p95_A_unchg, max_A_unchg = stat_triplet(unchanged_records, "plan_A_ms")
    med_B_unchg, p95_B_unchg, max_B_unchg = stat_triplet(unchanged_records, "plan_B_ms")
    med_C_unchg, p95_C_unchg, max_C_unchg = stat_triplet(unchanged_records, "plan_C_ms")

    med_A_all, p95_A_all, max_A_all = stat_triplet(records_after_0, "plan_A_ms")
    med_B_all, p95_B_all, max_B_all = stat_triplet(records_after_0, "plan_B_ms")
    med_C_all, p95_C_all, max_C_all = stat_triplet(records_after_0, "plan_C_ms")

    total_expanded = sum(r["C_expanded"] for r in records_after_0)
    ratios = [r["ratio_CA"] for r in records_after_0]
    avg_ratio = sum(ratios) / len(ratios) if ratios else 1.0
    max_ratio = max(ratios) if ratios else 1.0

    print("\n================================================================================")
    print("BENCHMARK RESULTS SUMMARY")
    print("================================================================================")
    print(f"First-plan execution time (step 0, reported separately):")
    print(f"  Planner A: {step_records[0]['plan_A_ms']:8.2f} ms")
    print(f"  Planner B: {step_records[0]['plan_B_ms']:8.2f} ms")
    print(f"  Planner C: {step_records[0]['plan_C_ms']:8.2f} ms")

    print(f"\nSteps WITH Obstacle Changes (n={len(changed_records)}, step 0 excluded):")
    print(f"  Planner A: median={med_A_chg:6.2f} ms | p95={p95_A_chg:6.2f} ms | max={max_A_chg:6.2f} ms")
    print(f"  Planner B: median={med_B_chg:6.2f} ms | p95={p95_B_chg:6.2f} ms | max={max_B_chg:6.2f} ms")
    print(f"  Planner C: median={med_C_chg:6.2f} ms | p95={p95_C_chg:6.2f} ms | max={max_C_chg:6.2f} ms")

    print(f"\nSteps WITHOUT Obstacle Changes (n={len(unchanged_records)}, step 0 excluded):")
    print(f"  Planner A: median={med_A_unchg:6.2f} ms | p95={p95_A_unchg:6.2f} ms | max={max_A_unchg:6.2f} ms")
    print(f"  Planner B: median={med_B_unchg:6.2f} ms | p95={p95_B_unchg:6.2f} ms | max={max_B_unchg:6.2f} ms")
    print(f"  Planner C: median={med_C_unchg:6.2f} ms | p95={p95_C_unchg:6.2f} ms | max={max_C_unchg:6.2f} ms")

    print(f"\nOverall Performance (n={len(records_after_0)}, step 0 excluded):")
    print(f"  Planner A: median={med_A_all:6.2f} ms | p95={p95_A_all:6.2f} ms | max={max_A_all:6.2f} ms")
    print(f"  Planner B: median={med_B_all:6.2f} ms | p95={p95_B_all:6.2f} ms | max={max_B_all:6.2f} ms")
    print(f"  Planner C: median={med_C_all:6.2f} ms | p95={p95_C_all:6.2f} ms | max={max_C_all:6.2f} ms")

    print(f"\nD* Lite Metrics:")
    print(f"  Total expanded cells across run: {total_expanded}")
    print(f"  Path length ratio (D* / A*): mean = {avg_ratio:.4f}, max = {max_ratio:.4f}")

    print("\nSteps where D* Lite searched (expanded > 0):")
    print(f"  {'Step':>6s} | {'Time (s)':>8s} | {'Expanded':>8s} | {'Rebuilt':>7s} | {'D* ms':>8s}")
    print(f"  {'-'*6}---{'-'*8}---{'-'*8}---{'-'*7}---{'-'*8}")
    for s, t_s, exp, reb, ms in dstar_search_steps:
        print(f"  {s:6d} | {t_s:8.1f} | {exp:8d} | {str(reb):>7s} | {ms:8.2f}")

    # Acceptance criteria verification
    print("\n================================================================================")
    print("ACCEPTANCE CRITERIA VERIFICATION")
    print("================================================================================")
    search_after_0 = [row for row in dstar_search_steps if row[0] > 0 and row[2] > 0]
    print(f"1. Steps where D* Lite searched (step > 0, expanded > 0): {len(search_after_0)} row(s)")
    for s, t_s, exp, reb, ms in search_after_0:
        print(f"   Step {s:3d} | Time {t_s:5.1f}s | Expanded {exp:6d} | Rebuilt {str(reb):>5s} | D* {ms:6.2f} ms")

    growth_found = False
    for k in range(1, len(step_records)):
        prev_len = step_records[k - 1]["len_A"]
        curr_len = step_records[k]["len_A"]
        diff = curr_len - prev_len
        if diff >= 12.0:
            st = step_records[k]["step"]
            print(f"2. Step where A's path length grew by >= 12 m: step {st}, length before: {prev_len:.2f} m, length after: {curr_len:.2f} m (growth: {diff:.2f} m)")
            growth_found = True
            break
    if not growth_found:
        print("2. Step where A's path length grew by >= 12 m: None found!")

    a_found_d_not = sum(1 for r in step_records if r["len_A"] > 0 and r["len_C"] == 0)
    print(f"3. D*/A* path length ratio over all steps: mean = {avg_ratio:.4f}, max = {max_ratio:.4f}")
    print(f"   Steps where A found a path and D* did not: {a_found_d_not}")

    # Always write CSV
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(step_records[0].keys()))
            writer.writeheader()
            writer.writerows(step_records)
        print(f"\nBenchmark step log successfully exported to {args.csv}")


if __name__ == "__main__":
    main()
