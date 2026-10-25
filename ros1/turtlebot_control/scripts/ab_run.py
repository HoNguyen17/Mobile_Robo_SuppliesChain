#!/usr/bin/env python3
"""A/B Benchmark Experiment Runner for A* vs D* Lite.

Executes autonomous runs in simulation, records execution traces,
verifies run validity, tracks motion quality metrics, and outputs CSV logs.
Python 3.8 compatible.
"""

import argparse
import csv
import datetime
import json
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

ASTAR_REPLAN_TRIGGER = "Dynamic obstacle detected blocking current planned path ahead! Triggering early replan..."
DSTAR_REPLAN_TRIGGER_PREFIX = "Dynamic obstacle detected blocking current planned path ahead! Triggering early replan..."

CSV_COLUMNS = [
    "run_id",
    "label",
    "planner",
    "scenario",
    "goal_x",
    "goal_y",
    "run_index",
    "outcome",
    "sim_time_to_goal_s",
    "wall_time_s",
    "driven_m",
    "straight_m",
    "displacement_m",
    "path_ratio",
    "stops",
    "collisions",
    "min_clearance_m",
    "plans",
    "early_replans",
    "first_plan_ms",
    "first_plan_rebuilt",
    "replan_ms_median",
    "replan_ms_p95",
    "replan_ms_max",
    "expanded_total",
    "rebuilds",
    "level_max",
    "started_at",
    "ended_at",
    "settle_plans",
    "invalid_reason",
    "first_plan_path_ready_ms",
    "replan_path_ready_median_ms",
    "replan_path_ready_p95_ms",
    "settle_diag",
    "settle_retry",
    "max_xte_m",
    "mean_xte_m",
    "heading_osc",
    "path_switches",
    "plan_ms_total",
    "plan_share_pct",
]

ROBOT_FOOTPRINT_RADIUS_M = 0.257
DEFAULT_START_POSE = (5.01, -0.05)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Compute yaw angle in [-pi, pi] from quaternion components."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wait_for_publisher_connection(
    pub: Any, timeout_wall_s: float = 5.0, rospy_mod: Any = None
) -> bool:
    """Wait up to timeout_wall_s until pub has at least 1 connection."""
    t0 = time.time()
    while time.time() - t0 < timeout_wall_s:
        if rospy_mod and hasattr(rospy_mod, "is_shutdown") and rospy_mod.is_shutdown():
            return False
        if hasattr(pub, "get_num_connections"):
            if pub.get_num_connections() > 0:
                return True
        else:
            return True
        time.sleep(0.05)
    return False


# ---------------- 1. Pure Parsing Functions ----------------

def parse_planned_line(text: str) -> Optional[Dict[str, Any]]:
    """Parse 'Planned: ...' line from either astar_planner or dstar_planner.

    A* format:
      Planned: %d corners, %.1f m, %.0f ms, inflation %.2f m (%d dynamic obstacles)

    D* format:
      Planned: %d corners, %.1f m, %.0f ms, inflation %.2f m (%d dynamic obstacles),
      expanded=%d, rebuilt=%s, level=%d, search=%.0f ms, changed=%d, set_obs=%.0f ms,
      costmap_inflate=%.0f ms, costmap_pub=%.0f ms, path_pub=%.0f ms, path_ready=%.0f ms, ...
    """
    if not text.startswith("Planned:"):
        return None

    try:
        content = text[len("Planned:"):].strip()
        parts = [p.strip() for p in content.split(",")]
        if len(parts) < 4:
            return None

        # 1. corners
        corners = int(parts[0].split()[0])

        # 2. path_len_m
        path_len_m = float(parts[1].split()[0])

        # 3. plan_ms
        plan_ms = float(parts[2].split()[0])

        # 4. inflation and dynamic obstacles
        rest = parts[3]
        inflation_idx = rest.find("inflation")
        obs_idx = rest.find("(")
        if inflation_idx != -1 and obs_idx != -1:
            inflation_str = rest[inflation_idx + len("inflation"):obs_idx].strip().split()[0]
            inflation_m = float(inflation_str)
            dynamic_obs_str = rest[obs_idx + 1:rest.find("dynamic")].strip()
            dynamic_obs = int(dynamic_obs_str)
        else:
            inflation_m = 0.0
            dynamic_obs = 0

        # Check for D* extended fields
        expanded = None
        rebuilt = None
        level = None
        path_ready_ms = None
        search_ms = None

        for p in parts[4:]:
            kv = p.split("=")
            if len(kv) == 2:
                k, v = kv[0].strip(), kv[1].strip()
                if k == "expanded":
                    expanded = int(v)
                elif k == "rebuilt":
                    rebuilt = (v.lower() == "true")
                elif k == "level":
                    level = int(v)
                elif k == "path_ready":
                    path_ready_ms = float(v.split()[0])
                elif k == "search":
                    search_ms = float(v.split()[0])

        is_dstar = expanded is not None or rebuilt is not None

        return {
            "corners": corners,
            "path_len_m": path_len_m,
            "plan_ms": plan_ms,
            "inflation_m": inflation_m,
            "dynamic_obs": dynamic_obs,
            "is_dstar": is_dstar,
            "expanded": expanded,
            "rebuilt": rebuilt,
            "level": level,
            "path_ready_ms": path_ready_ms,
            "search_ms": search_ms,
        }
    except Exception:
        return None


def parse_early_replan_line(text: str) -> bool:
    """Return True if text matches early replan trigger message."""
    return ASTAR_REPLAN_TRIGGER in text or DSTAR_REPLAN_TRIGGER_PREFIX in text


# ---------------- 2. Pure Aggregation & Metric Functions ----------------

def percentile(data: Sequence[float], p: float) -> float:
    """Compute p-th percentile of a sequence (0 <= p <= 100)."""
    if not data:
        return 0.0
    sorted_d = sorted(data)
    k = (len(sorted_d) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_d[int(k)]
    return sorted_d[int(f)] * (c - k) + sorted_d[int(c)] * (k - f)


def compute_driven_distance(poses: Sequence[Any]) -> float:
    """Compute total path length in metres from sequence of coordinates."""
    if not poses or len(poses) < 2:
        return 0.0
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        if len(a) >= 3 and len(b) >= 3:
            ax, ay = a[1], a[2]
            bx, by = b[1], b[2]
        else:
            ax, ay = a[0], a[1]
            bx, by = b[0], b[1]
        total += math.hypot(bx - ax, by - ay)
    return total


def compute_displacement(
    poses: Sequence[Any],
    outcome: str,
) -> Optional[float]:
    """Compute displacement in metres from first pose to last pose for succeeded runs.

    Returns Euclidean distance in metres if outcome == 'succeeded' and at least 1 pose exists.
    Returns None otherwise.
    """
    if str(outcome).strip().lower() != "succeeded" or not poses:
        return None
    first = poses[0]
    last = poses[-1]
    fx = first[1] if len(first) >= 3 else first[0]
    fy = first[2] if len(first) >= 3 else first[1]
    lx = last[1] if len(last) >= 3 else last[0]
    ly = last[2] if len(last) >= 3 else last[1]
    return math.hypot(lx - fx, ly - fy)


def compute_path_ratio(
    driven_m: float,
    displacement_m: Optional[float],
) -> Optional[float]:
    """Compute path_ratio = driven_m / displacement_m (>= 1.0 by construction).

    Returns path ratio float if displacement_m is provided and valid.
    Returns None if displacement_m is None.
    """
    if displacement_m is None:
        return None
    if displacement_m > 0.05:
        return max(1.0, driven_m / displacement_m)
    return 1.0


def compute_plan_share_pct(
    plan_ms_total: Optional[Union[float, str]],
    sim_time_to_goal_s: Optional[float],
    outcome: str,
) -> Optional[float]:
    """Compute plan_share_pct = 100 * plan_ms_total / 1000 / sim_time_to_goal_s for succeeded runs.

    Returns None (blank in CSV) if outcome != 'succeeded', no plan parsed, or sim_time <= 0.
    """
    if str(outcome).strip().lower() != "succeeded":
        return None
    if plan_ms_total is None or plan_ms_total == "":
        return None
    if sim_time_to_goal_s is None or sim_time_to_goal_s <= 0.0:
        return None
    try:
        ms = float(plan_ms_total)
        return round(100.0 * (ms / 1000.0) / float(sim_time_to_goal_s), 2)
    except (ValueError, TypeError):
        return None


def compute_stops(
    poses: Sequence[Any],
    start_sim_time: float,
    min_duration: float = 1.5,
    speed_thresh: float = 0.02,
    yaw_rate_thresh: float = 0.05,
) -> int:
    """Count stops where linear speed < speed_thresh AND |yaw_rate| < yaw_rate_thresh
    for >= min_duration, ignoring t < start_sim_time + 1.0s.

    poses: list of (sim_time_s, x, y, yaw) or (sim_time_s, x, y)
    """
    if not poses or len(poses) < 2:
        return 0

    valid_poses = [p for p in poses if p[0] >= start_sim_time + 1.0]
    if len(valid_poses) < 2:
        return 0

    stops = 0
    stopped_duration = 0.0
    in_stop = False

    for p1, p2 in zip(valid_poses, valid_poses[1:]):
        t1, x1, y1 = p1[0], p1[1], p1[2]
        yaw1 = p1[3] if len(p1) > 3 else 0.0
        t2, x2, y2 = p2[0], p2[1], p2[2]
        yaw2 = p2[3] if len(p2) > 3 else 0.0

        dt = t2 - t1
        if dt <= 0.0:
            continue
        dist = math.hypot(x2 - x1, y2 - y1)
        speed = dist / dt

        dyaw = (yaw2 - yaw1 + math.pi) % (2.0 * math.pi) - math.pi
        yaw_rate = abs(dyaw / dt)

        if speed < speed_thresh and yaw_rate < yaw_rate_thresh:
            stopped_duration += dt
            if stopped_duration >= min_duration and not in_stop:
                stops += 1
                in_stop = True
        else:
            stopped_duration = 0.0
            in_stop = False

    return stops


def compute_min_clearance(
    scans: Sequence[Sequence[float]],
    footprint_radius: float = ROBOT_FOOTPRINT_RADIUS_M,
    min_range: float = 0.12,
) -> float:
    """Compute minimum clearance in metres from laser scans minus footprint radius."""
    min_dist = float("inf")
    for scan in scans:
        for r in scan:
            if not math.isnan(r) and not math.isinf(r) and r >= min_range:
                if r < min_dist:
                    min_dist = r
    if math.isinf(min_dist):
        return 0.0
    return max(0.0, min_dist - footprint_radius)


def point_to_segment_dist(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    """Shortest distance from point (px, py) to line segment (ax, ay)-(bx, by)."""
    dx = bx - ax
    dy = by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def point_to_polyline_dist(
    px: float, py: float, polyline: Sequence[Tuple[float, float]]
) -> float:
    """Shortest distance from point (px, py) to a polyline of (x, y) waypoints."""
    if not polyline:
        return 0.0
    if len(polyline) == 1:
        return math.hypot(px - polyline[0][0], py - polyline[0][1])
    min_d = float("inf")
    for a, b in zip(polyline, polyline[1:]):
        d = point_to_segment_dist(px, py, a[0], a[1], b[0], b[1])
        if d < min_d:
            min_d = d
    return min_d


def compute_cross_track_errors(
    poses: Sequence[Any],
    paths: Sequence[Tuple[float, Sequence[Tuple[float, float]]]],
) -> Tuple[Optional[float], Optional[float]]:
    """Compute (max_xte_m, mean_xte_m) across recorded poses against active path polylines."""
    if not poses or not paths:
        return None, None

    sorted_paths = sorted(paths, key=lambda x: x[0])
    xtes = []  # type: List[float]

    for p in poses:
        t = p[0] if len(p) >= 3 else 0.0
        px = p[1] if len(p) >= 3 else p[0]
        py = p[2] if len(p) >= 3 else p[1]
        # Active path is the latest path received at or before t
        active_poly = None
        for path_t, poly in sorted_paths:
            if path_t <= t:
                active_poly = poly
            else:
                break
        if active_poly is None:
            active_poly = sorted_paths[0][1]

        if active_poly:
            xtes.append(point_to_polyline_dist(px, py, active_poly))

    if not xtes:
        return None, None
    return max(xtes), (sum(xtes) / float(len(xtes)))


def compute_heading_oscillations(
    cmd_vels: Sequence[Any], thresh: float = 0.2
) -> int:
    """Count sign changes of angular.z with |w| > thresh rad/s."""
    last_sign = 0
    oscillations = 0
    for cv in cmd_vels:
        ang_z = cv[2] if isinstance(cv, (tuple, list)) else float(cv)
        if ang_z > thresh:
            sign = 1
        elif ang_z < -thresh:
            sign = -1
        else:
            continue
        if last_sign != 0 and sign != last_sign:
            oscillations += 1
        last_sign = sign
    return oscillations


def aggregate_plan_stats(
    planned_logs: Sequence[Dict[str, Any]],
    path_ready_logs: Optional[Sequence[float]] = None,
    is_dstar: Optional[bool] = None,
) -> Dict[str, Any]:
    """Aggregate first plan, replan, and search metrics from parsed planned lines."""
    if not planned_logs:
        return {
            "plans": 0,
            "first_plan_ms": "",
            "first_plan_rebuilt": "",
            "replan_ms_median": "",
            "replan_ms_p95": "",
            "replan_ms_max": "",
            "expanded_total": "",
            "rebuilds": "",
            "level_max": "",
            "first_plan_path_ready_ms": "",
            "replan_path_ready_median_ms": "",
            "replan_path_ready_p95_ms": "",
            "plan_ms_total": "",
        }

    first = planned_logs[0]
    first_ms = first["plan_ms"]
    first_rebuilt = str(first["rebuilt"]) if first.get("rebuilt") is not None else ""
    first_path_ready = (
        round(first["path_ready_ms"], 1)
        if first.get("path_ready_ms") is not None
        else ""
    )

    plan_ms_total_val = round(sum(p["plan_ms"] for p in planned_logs), 1)

    replans = [p["plan_ms"] for p in planned_logs[1:]]
    if replans:
        replan_med = round(percentile(replans, 50), 1)
        replan_p95 = round(percentile(replans, 95), 1)
        replan_max = round(max(replans), 1)
    else:
        replan_med = ""
        replan_p95 = ""
        replan_max = ""

    replan_prs = [
        p["path_ready_ms"]
        for p in planned_logs[1:]
        if p.get("path_ready_ms") is not None
    ]
    if replan_prs:
        replan_pr_med = round(percentile(replan_prs, 50), 1)
        replan_pr_p95 = round(percentile(replan_prs, 95), 1)
    else:
        replan_pr_med = ""
        replan_pr_p95 = ""

    if is_dstar is None:
        is_dstar = any(p.get("is_dstar", False) for p in planned_logs)

    if is_dstar:
        expanded_total_val = sum(p.get("expanded") or 0 for p in planned_logs)
        rebuilds_count_val = sum(1 for p in planned_logs if p.get("rebuilt") is True)
        levels = [p["level"] for p in planned_logs if p.get("level") is not None]
        level_max_val = max(levels) if levels else 0
    else:
        expanded_total_val = ""
        rebuilds_count_val = ""
        level_max_val = ""

    return {
        "plans": len(planned_logs),
        "first_plan_ms": round(first_ms, 1),
        "first_plan_rebuilt": first_rebuilt,
        "replan_ms_median": replan_med,
        "replan_ms_p95": replan_p95,
        "replan_ms_max": replan_max,
        "expanded_total": expanded_total_val,
        "rebuilds": rebuilds_count_val,
        "level_max": level_max_val,
        "first_plan_path_ready_ms": first_path_ready,
        "replan_path_ready_median_ms": replan_pr_med,
        "replan_path_ready_p95_ms": replan_pr_p95,
        "plan_ms_total": plan_ms_total_val,
    }


def check_csv_header(csv_filepath: str) -> None:
    """Check if existing CSV file has the exact CSV_COLUMNS header. Raise ValueError if mismatch."""
    if os.path.exists(csv_filepath) and os.path.getsize(csv_filepath) > 0:
        with open(csv_filepath, "r", newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            existing_header = next(reader, None)
            if existing_header is not None and existing_header != CSV_COLUMNS:
                raise ValueError(
                    f"CSV header mismatch in {csv_filepath}! Please move the old CSV aside.\n"
                    f"Expected: {CSV_COLUMNS}\n"
                    f"Found:    {existing_header}"
                )


def write_csv_row(csv_filepath: str, row: Dict[str, Any]) -> None:
    """Append row dictionary to csv_filepath, writing header if file does not exist.

    Raises:
        ValueError if the file exists and its header does not match CSV_COLUMNS.
    """
    check_csv_header(csv_filepath)
    file_exists = os.path.exists(csv_filepath) and os.path.getsize(csv_filepath) > 0
    with open(csv_filepath, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        if not file_exists:
            writer.writeheader()
        writer.writerow({col: row.get(col, "") for col in CSV_COLUMNS})


def detect_planner(active_nodes: Sequence[str]) -> Tuple[str, str]:
    """Detect active planner node from active_nodes list.

    Returns:
        (planner_type, node_name) where planner_type is 'astar' or 'dstar'.
    Raises:
        RuntimeError if 0 or >1 planners are detected.
    """
    found = []
    for node in active_nodes:
        base = node.strip("/").split("/")[-1]
        if base == "astar_planner":
            found.append(("astar", node))
        elif base == "dstar_planner":
            found.append(("dstar", node))

    if len(found) == 0:
        raise RuntimeError(f"No planner detected in active nodes: {active_nodes}")
    if len(found) > 1:
        raise RuntimeError(f"Multiple planners detected: {found}. Only 1 allowed.")
    return found[0]


# ---------------- 3. Simulation Runner & State Tracking ----------------

class RunRecorder:
    """Buffers ROS messages during one benchmark leg."""

    def __init__(self) -> None:
        self.poses: List[Tuple[float, float, float, float]] = []  # (sim_time, x, y, yaw)
        self.cmd_vels: List[Tuple[float, float, float]] = []  # (sim_time, lin_x, ang_z)
        self.paths: List[Tuple[float, List[Tuple[float, float]]]] = []  # (sim_time, [(x, y), ...])
        self.scans: List[List[float]] = []
        self.collisions: List[Dict[str, Any]] = []
        self.planned_logs: List[Dict[str, Any]] = []
        self.early_replans: int = 0
        self.leg_result: Optional[Tuple[str, str]] = None  # (outcome, reason)
        self.latest_ack: Optional[Dict[str, Any]] = None
        self.latest_ack_sim_time: Optional[float] = None
        self.recording: bool = False
        self.settling: bool = False
        self.settle_moved: bool = False
        self.settle_plans: int = 0
        self.ack_sim_time: Optional[float] = None
        self.last_settle_pose: Optional[Tuple[float, float, float, float]] = None
        self.last_settle_diag_pose_sim_time: Optional[float] = None
        self.expected_node: Optional[str] = None
        self.node_mismatch: bool = False

        # Settle diagnostics tracking
        self.settle_poses: List[List[float]] = []
        self.settle_max_lin_x: float = 0.0
        self.settle_max_ang_z: float = 0.0
        self.settle_paths_count: int = 0
        self.settle_leg_results: List[Any] = []
        self.settle_rosouts: List[str] = []

    def reset_for_run(self, expected_node: str) -> None:
        self.poses = []
        self.cmd_vels = []
        self.paths = []
        self.scans = []
        self.collisions = []
        self.planned_logs = []
        self.early_replans = 0
        self.leg_result = None
        self.recording = True
        self.settling = False
        self.expected_node = expected_node
        self.node_mismatch = False
        self.last_settle_pose = None
        self.last_settle_diag_pose_sim_time = None

    def start_settle(self, ack_sim_time: Optional[float] = None) -> None:
        self.settling = True
        self.settle_moved = False
        self.settle_plans = 0
        self.ack_sim_time = ack_sim_time if ack_sim_time is not None else self.latest_ack_sim_time
        self.last_settle_pose = None
        self.last_settle_diag_pose_sim_time = None
        self.settle_poses = []
        self.settle_max_lin_x = 0.0
        self.settle_max_ang_z = 0.0
        self.settle_paths_count = 0
        self.settle_leg_results = []
        self.settle_rosouts = []

    def stop_settle(self) -> None:
        self.settling = False

    def get_settle_diag(self) -> Dict[str, Any]:
        return {
            "poses": self.settle_poses,
            "max_lin_x": round(self.settle_max_lin_x, 4),
            "max_ang_z": round(self.settle_max_ang_z, 4),
            "planned_paths": self.settle_paths_count,
            "leg_results": self.settle_leg_results,
            "rosouts": self.settle_rosouts,
        }

    def on_pose(self, x: float, y: float, sim_time: float, yaw: float = 0.0) -> None:
        if self.recording:
            self.poses.append((sim_time, x, y, yaw))
        if self.settling:
            # F3: store poses downsampled to at most 4 per second of sim time (>= 0.249 s interval)
            if self.last_settle_diag_pose_sim_time is None or (sim_time - self.last_settle_diag_pose_sim_time) >= 0.249:
                self.settle_poses.append([round(sim_time, 3), round(x, 3), round(y, 3)])
                self.last_settle_diag_pose_sim_time = sim_time

            # The movement check keeps using every pose
            if self.ack_sim_time is not None and sim_time < (self.ack_sim_time + 0.3):
                # Poses stamped earlier than ack_time + 0.3 s are ignored for movement check
                return
            self.last_settle_pose = (x, y, sim_time, yaw)
            if math.hypot(x - DEFAULT_START_POSE[0], y - DEFAULT_START_POSE[1]) > 0.30:
                self.settle_moved = True

    def on_cmd_vel(self, lin_x: float, ang_z: float, sim_time: float) -> None:
        if self.recording:
            self.cmd_vels.append((sim_time, lin_x, ang_z))
        if self.settling:
            if abs(lin_x) > self.settle_max_lin_x:
                self.settle_max_lin_x = abs(lin_x)
            if abs(ang_z) > self.settle_max_ang_z:
                self.settle_max_ang_z = abs(ang_z)

    def on_planned_path(self, waypoints: List[Tuple[float, float]], sim_time: float) -> None:
        if self.recording:
            self.paths.append((sim_time, waypoints))
        if self.settling:
            self.settle_paths_count += 1

    def on_scan(self, ranges: Sequence[float]) -> None:
        if self.recording:
            self.scans.append(list(ranges))

    def on_collision(self, payload: Dict[str, Any]) -> None:
        if self.recording:
            self.collisions.append(payload)

    def on_leg_result(self, outcome: str, reason: str) -> None:
        if self.recording and self.leg_result is None:
            self.leg_result = (outcome, reason)
        if self.settling:
            self.settle_leg_results.append({"outcome": outcome, "reason": reason})

    def on_rosout(self, node_name: str, msg_text: str) -> None:
        if self.settling:
            base_node = node_name.strip("/").split("/")[-1]
            if any(k in base_node for k in ("astar_planner", "dstar_planner", "path_follower")):
                self.settle_rosouts.append(f"[{base_node}] {msg_text}")
            if parse_planned_line(msg_text) is not None:
                self.settle_plans += 1
            return
        if not self.recording:
            return
        if parse_early_replan_line(msg_text):
            self.early_replans += 1
            return

        parsed = parse_planned_line(msg_text)
        if parsed is not None:
            if self.expected_node and node_name != self.expected_node:
                self.node_mismatch = True
            parsed["node_name"] = node_name
            self.planned_logs.append(parsed)

    def on_ack(self, payload: Dict[str, Any], sim_time: Optional[float] = None) -> None:
        self.latest_ack = payload
        if sim_time is not None:
            self.latest_ack_sim_time = sim_time
            self.ack_sim_time = sim_time


def wait_for_sim_time(rospy_mod: Any, target_sim_time: float, timeout_wall_s: float = 30.0) -> bool:
    """Wait until sim time reaches target_sim_time."""
    t_start = time.time()
    rate = rospy_mod.Rate(20)
    while not rospy_mod.is_shutdown():
        if rospy_mod.Time.now().to_sec() >= target_sim_time:
            return True
        if time.time() - t_start > timeout_wall_s:
            return False
        rate.sleep()
    return False


def run_ab_experiment(
    scenario: str = "S-04",
    goal_xy: Tuple[float, float] = (-3.75, -3.13),
    repeats: int = 5,
    settle_s: float = 7.0,
    timeout_s: float = 120.0,
    out_dir: str = "~/ab_runs",
    custom_label: Optional[str] = None,
    rospy_mod: Any = None,
) -> List[Dict[str, Any]]:
    """Execute the A/B testing matrix and record CSV results."""
    if rospy_mod is None:
        import rospy as rospy_mod  # noqa: F401

    import rosnode
    from geometry_msgs.msg import PoseStamped, Twist
    from rosgraph_msgs.msg import Clock, Log
    from nav_msgs.msg import OccupancyGrid, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Empty, String

    planner_type, planner_node = detect_planner(rosnode.get_node_names())
    label = custom_label or planner_type
    out_dir = os.path.expanduser(out_dir)
    csv_file = os.path.join(out_dir, f"{label}_runs.csv")

    # Check header if output CSV exists BEFORE any reset or simulation touch
    check_csv_header(csv_file)

    rospy_mod.loginfo(
        "A/B Runner: planner=%s (%s), scenario=%s, repeats=%d, out=%s",
        planner_type,
        planner_node,
        scenario,
        repeats,
        csv_file,
    )

    recorder = RunRecorder()
    latest_pose: Optional[Tuple[float, float, float, float]] = None  # (x, y, stamp, yaw)
    clock_received = False
    map_received = False
    scan_received = False

    def handle_clock(msg: Clock) -> None:
        nonlocal clock_received
        clock_received = True

    def handle_map(_msg: OccupancyGrid) -> None:
        nonlocal map_received
        map_received = True

    def handle_pose(msg: PoseStamped) -> None:
        nonlocal latest_pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        t = msg.header.stamp.to_sec() if hasattr(msg.header, "stamp") else rospy_mod.Time.now().to_sec()
        latest_pose = (x, y, t, yaw)
        recorder.on_pose(x, y, t, yaw)

    def handle_cmd_vel(msg: Twist) -> None:
        t = rospy_mod.Time.now().to_sec()
        recorder.on_cmd_vel(msg.linear.x, msg.angular.z, t)

    def handle_planned_path(msg: Path) -> None:
        t = (
            msg.header.stamp.to_sec()
            if hasattr(msg.header, "stamp") and msg.header.stamp.to_sec() > 0
            else rospy_mod.Time.now().to_sec()
        )
        wps = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        recorder.on_planned_path(wps, t)

    def handle_scan(msg: LaserScan) -> None:
        nonlocal scan_received
        scan_received = True
        recorder.on_scan(msg.ranges)

    def handle_collision(msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        recorder.on_collision(data)

    def handle_leg_result(msg: String) -> None:
        try:
            data = json.loads(msg.data)
            outcome = data.get("outcome", "aborted")
            reason = data.get("reason", "")
        except Exception:
            outcome = "aborted"
            reason = msg.data
        recorder.on_leg_result(outcome, reason)

    def handle_rosout(msg: Log) -> None:
        recorder.on_rosout(msg.name, msg.msg)

    last_ack_warn_time = [0.0]

    def handle_ack(msg: String) -> None:
        try:
            data = json.loads(msg.data)
            sim_t = rospy_mod.Time.now().to_sec()
            recorder.on_ack(data, sim_t)
        except Exception as exc:
            now_w = time.time()
            if now_w - last_ack_warn_time[0] >= 5.0:
                last_ack_warn_time[0] = now_w
                if hasattr(rospy_mod, "logwarn_throttle"):
                    rospy_mod.logwarn_throttle(5.0, "Malformed /sim/ack JSON: %s", exc)
                elif hasattr(rospy_mod, "logwarn"):
                    rospy_mod.logwarn("Malformed /sim/ack JSON: %s", exc)

    # Setup Publishers and Subscribers before the loop
    pub_reset = rospy_mod.Publisher("/sim/reset_scenario", String, queue_size=1)
    pub_goal = rospy_mod.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    pub_cancel = rospy_mod.Publisher("/nav/cancel", Empty, queue_size=1)

    rospy_mod.Subscriber("/clock", Clock, handle_clock)
    rospy_mod.Subscriber("/map", OccupancyGrid, handle_map)
    rospy_mod.Subscriber("/robot/pose", PoseStamped, handle_pose)
    rospy_mod.Subscriber("/cmd_vel", Twist, handle_cmd_vel)
    rospy_mod.Subscriber("/planned_path", Path, handle_planned_path)
    rospy_mod.Subscriber("/scan", LaserScan, handle_scan)
    rospy_mod.Subscriber("/sim/collision", String, handle_collision)
    rospy_mod.Subscriber("/nav/leg_result", String, handle_leg_result)
    rospy_mod.Subscriber("/rosout", Log, handle_rosout)
    rospy_mod.Subscriber("/sim/ack", String, handle_ack)

    # 1. Wait until prerequisites arrive (30 s limit)
    rospy_mod.loginfo("Waiting for /clock, /map, /robot/pose, /scan (30 s limit)...")
    t_wait_start = time.time()
    rate = rospy_mod.Rate(10)
    while not rospy_mod.is_shutdown():
        if clock_received and map_received and (latest_pose is not None) and scan_received:
            break
        if time.time() - t_wait_start > 30.0:
            raise RuntimeError(
                f"Prerequisites timeout! clock={clock_received}, map={map_received}, "
                f"pose={latest_pose is not None}, scan={scan_received}"
            )
        # Use wall-clock sleep while awaiting /clock so rate.sleep() does not block forever on sim time 0
        if clock_received and hasattr(rate, "sleep"):
            try:
                rate.sleep()
            except Exception:
                time.sleep(0.05)
        else:
            time.sleep(0.05)

    rows: List[Dict[str, Any]] = []

    # 2. Per-run execution loop
    for run_idx in range(1, repeats + 1):
        if rospy_mod.is_shutdown():
            break

        run_id = f"{label}_{scenario}_r{run_idx}_{int(time.time())}"
        started_at = datetime.datetime.now().isoformat()
        rospy_mod.loginfo("=== Starting Run %d / %d (run_id: %s) ===", run_idx, repeats, run_id)

        # Check connection for pub_reset
        if not wait_for_publisher_connection(pub_reset, timeout_wall_s=5.0, rospy_mod=rospy_mod):
            rospy_mod.logerr("Run %d: /sim/reset_scenario publisher not connected!", run_idx)
            empty_diag_str = json.dumps(
                {"poses": [], "max_lin_x": 0.0, "max_ang_z": 0.0, "planned_paths": 0, "leg_results": [], "rosouts": []},
                separators=(',', ':')
            )
            row = {col: "" for col in CSV_COLUMNS}
            row.update({
                "run_id": run_id,
                "label": label,
                "planner": planner_type,
                "scenario": scenario,
                "goal_x": goal_xy[0],
                "goal_y": goal_xy[1],
                "run_index": run_idx,
                "outcome": "invalid",
                "settle_plans": 0,
                "invalid_reason": "publisher not connected",
                "settle_diag": empty_diag_str,
                "settle_retry": 0,
                "started_at": started_at,
                "ended_at": datetime.datetime.now().isoformat(),
            })
            write_csv_row(csv_file, row)
            rows.append(row)
            continue

        # A & B: Reset and settle with one self-healing retry (TASK C)
        settle_retry = 0
        diag_attempts = []  # type: List[Dict[str, Any]]
        settle_failed = False
        ack_failure_reason = None

        for attempt in (1, 2):
            if attempt == 2:
                settle_retry = 1
                rospy_mod.logwarn("Run %d: settle movement detected, self-healing attempt 2...", run_idx)
                pub_cancel.publish(Empty())
                sim_now = rospy_mod.Time.now().to_sec()
                wait_for_sim_time(rospy_mod, sim_now + 2.0, timeout_wall_s=6.0)

            # Publish reset and wait for ack
            recorder.latest_ack = None
            pub_reset.publish(scenario)
            t_ack_start = time.time()
            ack_ok = False
            ack_sim_time = None
            while time.time() - t_ack_start < 5.0 and not rospy_mod.is_shutdown():
                if recorder.latest_ack and recorder.latest_ack.get("cmd") == "reset_scenario":
                    if recorder.latest_ack.get("ok"):
                        ack_ok = True
                        ack_sim_time = (
                            recorder.latest_ack_sim_time
                            if recorder.latest_ack_sim_time is not None
                            else rospy_mod.Time.now().to_sec()
                        )
                        break
                time.sleep(0.05)

            if not ack_ok:
                ack_failure_reason = "ack not ok" if (recorder.latest_ack and not recorder.latest_ack.get("ok")) else "ack timeout"
                rospy_mod.logerr("Run %d: /sim/ack failed (%s) on attempt %d!", run_idx, ack_failure_reason, attempt)
                settle_failed = True
                break

            # Settle wait
            recorder.start_settle(ack_sim_time=ack_sim_time)
            sim_now = rospy_mod.Time.now().to_sec()
            wait_for_sim_time(rospy_mod, sim_now + settle_s, timeout_wall_s=settle_s * 3.0)
            recorder.stop_settle()
            diag_attempts.append(recorder.get_settle_diag())

            if not recorder.settle_moved:
                settle_failed = False
                break
            else:
                settle_failed = True

        # Build settle_diag compact JSON string
        if len(diag_attempts) == 1:
            settle_diag_str = json.dumps(diag_attempts[0], separators=(',', ':'))
        elif len(diag_attempts) > 1:
            settle_diag_str = json.dumps(
                {"attempt1": diag_attempts[0], "attempt2": diag_attempts[1]},
                separators=(',', ':')
            )
        else:
            settle_diag_str = json.dumps(
                {"poses": [], "max_lin_x": 0.0, "max_ang_z": 0.0, "planned_paths": 0, "leg_results": [], "rosouts": []},
                separators=(',', ':')
            )

        if ack_failure_reason:
            row = {col: "" for col in CSV_COLUMNS}
            row.update({
                "run_id": run_id,
                "label": label,
                "planner": planner_type,
                "scenario": scenario,
                "goal_x": goal_xy[0],
                "goal_y": goal_xy[1],
                "run_index": run_idx,
                "outcome": "invalid",
                "settle_plans": 0,
                "invalid_reason": ack_failure_reason,
                "settle_diag": settle_diag_str,
                "settle_retry": settle_retry,
                "started_at": started_at,
                "ended_at": datetime.datetime.now().isoformat(),
            })
            write_csv_row(csv_file, row)
            rows.append(row)
            # Post-run cleanup: publish /nav/cancel and wait 1 s
            pub_cancel.publish(Empty())
            wait_for_sim_time(rospy_mod, rospy_mod.Time.now().to_sec() + 1.0, timeout_wall_s=3.0)
            continue

        if settle_failed:
            rospy_mod.logerr("Run %d: robot moved > 0.3 m during settle!", run_idx)
            row = {col: "" for col in CSV_COLUMNS}
            row.update({
                "run_id": run_id,
                "label": label,
                "planner": planner_type,
                "scenario": scenario,
                "goal_x": goal_xy[0],
                "goal_y": goal_xy[1],
                "run_index": run_idx,
                "outcome": "invalid",
                "settle_plans": recorder.settle_plans,
                "invalid_reason": "settle movement",
                "settle_diag": settle_diag_str,
                "settle_retry": settle_retry,
                "started_at": started_at,
                "ended_at": datetime.datetime.now().isoformat(),
            })
            write_csv_row(csv_file, row)
            rows.append(row)
            # Post-run cleanup: publish /nav/cancel and wait 1 s
            pub_cancel.publish(Empty())
            wait_for_sim_time(rospy_mod, rospy_mod.Time.now().to_sec() + 1.0, timeout_wall_s=3.0)
            continue

        # C. Check robot pose is within 0.3 m of (5.01, -0.05)
        pose_to_check = recorder.last_settle_pose if recorder.last_settle_pose is not None else latest_pose
        if pose_to_check is None:
            dist_to_start = float("inf")
        else:
            dist_to_start = math.hypot(
                pose_to_check[0] - DEFAULT_START_POSE[0],
                pose_to_check[1] - DEFAULT_START_POSE[1],
            )

        if dist_to_start > 0.30:
            rospy_mod.logerr("Run %d: robot pose %.2f m from start pose (limit 0.3 m)!", run_idx, dist_to_start)
            row = {col: "" for col in CSV_COLUMNS}
            row.update({
                "run_id": run_id,
                "label": label,
                "planner": planner_type,
                "scenario": scenario,
                "goal_x": goal_xy[0],
                "goal_y": goal_xy[1],
                "run_index": run_idx,
                "outcome": "invalid",
                "settle_plans": recorder.settle_plans,
                "invalid_reason": "pose off start",
                "settle_diag": settle_diag_str,
                "settle_retry": settle_retry,
                "started_at": started_at,
                "ended_at": datetime.datetime.now().isoformat(),
            })
            write_csv_row(csv_file, row)
            rows.append(row)
            pub_cancel.publish(Empty())
            wait_for_sim_time(rospy_mod, rospy_mod.Time.now().to_sec() + 1.0, timeout_wall_s=3.0)
            continue

        # Check publisher connection for pub_goal and pub_cancel
        if not wait_for_publisher_connection(pub_goal, timeout_wall_s=5.0, rospy_mod=rospy_mod):
            rospy_mod.logerr("Run %d: /move_base_simple/goal publisher not connected!", run_idx)
            row = {col: "" for col in CSV_COLUMNS}
            row.update({
                "run_id": run_id,
                "label": label,
                "planner": planner_type,
                "scenario": scenario,
                "goal_x": goal_xy[0],
                "goal_y": goal_xy[1],
                "run_index": run_idx,
                "outcome": "invalid",
                "settle_plans": recorder.settle_plans,
                "invalid_reason": "publisher not connected",
                "settle_diag": settle_diag_str,
                "settle_retry": settle_retry,
                "started_at": started_at,
                "ended_at": datetime.datetime.now().isoformat(),
            })
            write_csv_row(csv_file, row)
            rows.append(row)
            pub_cancel.publish(Empty())
            wait_for_sim_time(rospy_mod, rospy_mod.Time.now().to_sec() + 1.0, timeout_wall_s=3.0)
            continue

        if not wait_for_publisher_connection(pub_cancel, timeout_wall_s=5.0, rospy_mod=rospy_mod):
            rospy_mod.logerr("Run %d: /nav/cancel publisher not connected!", run_idx)
            row = {col: "" for col in CSV_COLUMNS}
            row.update({
                "run_id": run_id,
                "label": label,
                "planner": planner_type,
                "scenario": scenario,
                "goal_x": goal_xy[0],
                "goal_y": goal_xy[1],
                "run_index": run_idx,
                "outcome": "invalid",
                "settle_plans": recorder.settle_plans,
                "invalid_reason": "publisher not connected",
                "settle_diag": settle_diag_str,
                "settle_retry": settle_retry,
                "started_at": started_at,
                "ended_at": datetime.datetime.now().isoformat(),
            })
            write_csv_row(csv_file, row)
            rows.append(row)
            pub_cancel.publish(Empty())
            wait_for_sim_time(rospy_mod, rospy_mod.Time.now().to_sec() + 1.0, timeout_wall_s=3.0)
            continue

        # D. Start recording & publish goal
        recorder.reset_for_run(expected_node=planner_node)
        start_sim_time = rospy_mod.Time.now().to_sec()
        start_wall_time = time.time()

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy_mod.Time.now()
        goal_msg.pose.position.x = goal_xy[0]
        goal_msg.pose.position.y = goal_xy[1]
        goal_msg.pose.orientation.w = 1.0
        pub_goal.publish(goal_msg)

        # E. Wait until leg_result arrives or sim timeout
        timeout_sim_target = start_sim_time + timeout_s
        rate = rospy_mod.Rate(20)
        timed_out = False

        while not rospy_mod.is_shutdown():
            if recorder.leg_result is not None:
                break
            if rospy_mod.Time.now().to_sec() >= timeout_sim_target:
                timed_out = True
                break
            rate.sleep()

        end_sim_time = rospy_mod.Time.now().to_sec()
        end_wall_time = time.time()
        recorder.recording = False

        if timed_out and recorder.leg_result is None:
            rospy_mod.logwarn("Run %d: sim time timeout (%.1f s) reached, cancelling!", run_idx, timeout_s)
            pub_cancel.publish(Empty())
            outcome = "timeout"
        elif recorder.leg_result is not None:
            outcome = recorder.leg_result[0]
        else:
            outcome = "aborted"

        # Check validity criteria
        invalid_reason = ""
        if recorder.node_mismatch:
            outcome = "invalid"
            invalid_reason = "planner node mismatch"
        elif len(recorder.planned_logs) == 0:
            outcome = "invalid"
            invalid_reason = "no Planned line"

        # F. Compute metrics & aggregate stats
        sim_time_to_goal = max(0.0, end_sim_time - start_sim_time)
        wall_time_s = max(0.0, end_wall_time - start_wall_time)
        driven_m = compute_driven_distance(recorder.poses)
        start_pt = recorder.poses[0] if recorder.poses else (0.0, DEFAULT_START_POSE[0], DEFAULT_START_POSE[1], 0.0)
        straight_m = math.hypot(goal_xy[0] - start_pt[1], goal_xy[1] - start_pt[2])
        disp_val = compute_displacement(recorder.poses, outcome)
        if disp_val is not None:
            displacement_m_val = round(disp_val, 3)
            ratio_val = compute_path_ratio(driven_m, disp_val)
            path_ratio_val = round(ratio_val, 3) if ratio_val is not None else ""
        else:
            displacement_m_val = ""
            path_ratio_val = ""

        stops = compute_stops(recorder.poses, start_sim_time=start_sim_time)
        collisions = len(recorder.collisions)
        min_clearance = compute_min_clearance(recorder.scans)

        plan_stats = aggregate_plan_stats(recorder.planned_logs)
        plan_share_val = compute_plan_share_pct(
            plan_stats["plan_ms_total"], sim_time_to_goal, outcome
        )

        # Motion-quality metrics (TASK D)
        max_xte, mean_xte = compute_cross_track_errors(recorder.poses, recorder.paths)
        heading_osc = compute_heading_oscillations(recorder.cmd_vels, thresh=0.2)
        path_switches = len(recorder.paths)

        row = {col: "" for col in CSV_COLUMNS}
        row.update({
            "run_id": run_id,
            "label": label,
            "planner": planner_type,
            "scenario": scenario,
            "goal_x": goal_xy[0],
            "goal_y": goal_xy[1],
            "run_index": run_idx,
            "outcome": outcome,
            "sim_time_to_goal_s": round(sim_time_to_goal, 3),
            "wall_time_s": round(wall_time_s, 3),
            "driven_m": round(driven_m, 3),
            "straight_m": round(straight_m, 3),
            "displacement_m": displacement_m_val,
            "path_ratio": path_ratio_val,
            "stops": stops,
            "collisions": collisions,
            "min_clearance_m": round(min_clearance, 3),
            "plans": plan_stats["plans"],
            "early_replans": recorder.early_replans,
            "first_plan_ms": plan_stats["first_plan_ms"],
            "first_plan_rebuilt": plan_stats["first_plan_rebuilt"],
            "replan_ms_median": plan_stats["replan_ms_median"],
            "replan_ms_p95": plan_stats["replan_ms_p95"],
            "replan_ms_max": plan_stats["replan_ms_max"],
            "expanded_total": plan_stats["expanded_total"],
            "rebuilds": plan_stats["rebuilds"],
            "level_max": plan_stats["level_max"],
            "started_at": started_at,
            "ended_at": datetime.datetime.now().isoformat(),
            "settle_plans": recorder.settle_plans,
            "invalid_reason": invalid_reason,
            "first_plan_path_ready_ms": plan_stats["first_plan_path_ready_ms"],
            "replan_path_ready_median_ms": plan_stats["replan_path_ready_median_ms"],
            "replan_path_ready_p95_ms": plan_stats["replan_path_ready_p95_ms"],
            "settle_diag": settle_diag_str,
            "settle_retry": settle_retry,
            "max_xte_m": round(max_xte, 3) if max_xte is not None else "",
            "mean_xte_m": round(mean_xte, 3) if mean_xte is not None else "",
            "heading_osc": heading_osc if recorder.cmd_vels else "",
            "path_switches": path_switches,
            "plan_ms_total": plan_stats["plan_ms_total"],
            "plan_share_pct": plan_share_val if plan_share_val is not None else "",
        })

        write_csv_row(csv_file, row)
        rows.append(row)
        rospy_mod.loginfo(
            "Run %d complete: outcome=%s, time=%.1fs, driven=%.1fm, plans=%d, xte_mean=%s, osc=%s",
            run_idx,
            outcome,
            sim_time_to_goal,
            driven_m,
            plan_stats["plans"],
            str(row["mean_xte_m"]),
            str(row["heading_osc"]),
        )

        # Post-run cleanup: publish /nav/cancel and wait 1 s before next run (TASK C)
        pub_cancel.publish(Empty())
        wait_for_sim_time(rospy_mod, rospy_mod.Time.now().to_sec() + 1.0, timeout_wall_s=3.0)

    return rows


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run A/B benchmark experiments on navigation stack.")
    parser.add_argument("--scenario", default="S-04", help="Scenario name (default: S-04)")
    parser.add_argument(
        "--goal", nargs=2, type=float, default=[-3.75, -3.13], metavar=("X", "Y"), help="Goal coordinates (default: -3.75 -3.13)"
    )
    parser.add_argument("--repeats", type=int, default=5, help="Number of repetitions (default: 5)")
    parser.add_argument("--settle", type=float, default=7.0, help="Settle time in sim seconds after reset (default: 7.0)")
    parser.add_argument("--timeout", type=float, default=120.0, help="Timeout in sim seconds per run (default: 120.0)")
    parser.add_argument("--out", default="~/ab_runs", help="Output directory for runs CSV (default: ~/ab_runs)")
    parser.add_argument("--label", default=None, help="Custom label for output CSV filename")
    return parser.parse_args(argv)


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(line_buffering=True)
        except Exception:
            pass
    if hasattr(sys.stderr, "reconfigure"):
        try:
            sys.stderr.reconfigure(line_buffering=True)
        except Exception:
            pass
    import rospy
    rospy.init_node("ab_runner", anonymous=True)
    args = parse_args()
    try:
        run_ab_experiment(
            scenario=args.scenario,
            goal_xy=(args.goal[0], args.goal[1]),
            repeats=args.repeats,
            settle_s=args.settle,
            timeout_s=args.timeout,
            out_dir=args.out,
            custom_label=args.label,
            rospy_mod=rospy,
        )
    except Exception as exc:
        rospy.logerr("A/B runner encountered fatal error: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
