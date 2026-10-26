#!/usr/bin/env python3
"""dstar_planner: plans a path when a goal arrives using D* Lite and publishes it on /planned_path.

The planning itself is in dstar_lite.py. This node only connects it to ROS.
The node is registered as "dstar_planner" (not "astar_planner") so that both planners
cannot run simultaneously without an explicit name clash or ROS node replacement.

  in : /map, /robot/pose, /scan, /move_base_simple/goal, /nav/replan, parameter /nav/inflation_radius
  out: /planned_path (latched), /costmap (latched), /nav/leg_result (only when there is no path)

Everything is in the ROS "map" frame, in robot metres (ADR-010, ADR-012).
"""
import math
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

# catkin_make runs this file through a wrapper in devel/lib, so the folder with the
# modules next to it is not on sys.path by itself.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dstar_lite import (  # noqa: E402
    DStarLitePlanner,
    IncrementalInflation,
    expand_lidar_hit,
    path_near_points,
)
from grid_planner import (  # noqa: E402
    Grid,
    InflatedMaps,
    NoPath,
    ascii_map,
    inflate,
    overlay_obstacles,
)
from nav_common import ABORTED, NO_PATH, finite_or, leg_result  # noqa: E402

SNAP_DISTANCE = 2.0  # m, default of ~snap_distance
# The map is raw (ADR-012), so the planner must inflate it. Used while /nav/inflation_radius is
# not set: the footprint reaches 0.257 m from the wheel axis centre, so this leaves about 9 cm.
DEFAULT_INFLATION = 0.35  # m
# s of simulation time: planning from an older pose would start in the wrong place
POSE_MAX_AGE = 1.0
MAX_PICTURE_HEIGHT = 100  # the ASCII picture is skipped for taller grids


class DStarPlanner:
    def __init__(self) -> None:
        self.snap_distance = finite_or(
            rospy.get_param("~snap_distance", SNAP_DISTANCE), SNAP_DISTANCE, 0.0
        )
        self.print_map = rospy.get_param("~print_map", False)  # draw the path as text
        self.maps: Optional[InflatedMaps] = None  # replaced as one object
        self.planner: Optional[DStarLitePlanner] = None
        self.pose: Optional[PoseStamped] = None
        self.latest_goal: Optional[PoseStamped] = None
        self.current_path_corners: Optional[List[Tuple[float, float]]] = None
        self.current_path_radius: Optional[float] = None
        self._param_cache: Dict[str, Tuple[Any, float]] = {}
        self.last_costmap_pub_time: float = 0.0
        self.last_auto_replan_time: float = 0.0
        self.obstacle_cells: Dict[Tuple[int, int], float] = {}  # (i, j) -> timestamp
        self.obstacle_lock = threading.Lock()
        self._live_inc: Optional[IncrementalInflation] = None
        self._live_maps: Optional[InflatedMaps] = None
        self._live_radius: Optional[float] = None

        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=1, latch=True)
        self.costmap_pub = rospy.Publisher("/costmap", OccupancyGrid, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher("/nav/leg_result", String, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self.on_map)
        rospy.Subscriber("/robot/pose", PoseStamped, self.on_pose)
        rospy.Subscriber("/scan", LaserScan, self.on_scan)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_goal)
        rospy.Subscriber("/nav/replan", Empty, self.on_replan)
        rospy.loginfo("dstar_planner ready: send a goal to /move_base_simple/goal")

    def _get_param_cached(self, name: str, default: Any) -> Any:
        now = time.time()
        cached = self._param_cache.get(name)
        if cached is not None:
            val, last_time = cached
            if now - last_time < 1.0:
                return val
        val = rospy.get_param(name, default)
        self._param_cache[name] = (val, now)
        return val

    # ---------------- callbacks ----------------
    def on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        try:
            grid = Grid(
                msg.data,
                info.width,
                info.height,
                info.resolution,
                info.origin.position.x,
                info.origin.position.y,
            )
        except ValueError as exc:  # keep the last good map
            rospy.logwarn_throttle(10.0, "Ignoring a bad /map: %s", exc)
            return
        old = self.maps
        if old is not None and same_map(old.grid, grid):
            return  # Unity re-sends the map every second: keep the inflated copies
        self.maps = InflatedMaps(grid)
        self.planner = DStarLitePlanner(self.maps)
        rospy.loginfo(
            "New map: %d x %d cells of %.2f m.", grid.width, grid.height, grid.res
        )

    def on_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id not in ("", "map"):
            rospy.logwarn_throttle(
                10.0, "Ignoring /robot/pose in frame '%s', not 'map'.", msg.header.frame_id
            )
            return
        self.pose = msg

    def on_scan(self, msg: LaserScan) -> None:
        pose = self.pose
        maps = self.maps
        if pose is None or maps is None:
            return
        age = (rospy.Time.now() - pose.header.stamp).to_sec()
        if age > POSE_MAX_AGE:
            return

        robot_x = pose.pose.position.x
        robot_y = pose.pose.position.y
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        now = rospy.Time.now().to_sec()
        grid = maps.grid
        w, h, res, ox, oy = grid.width, grid.height, grid.res, grid.ox, grid.oy

        filter_static = self._get_param_cached("~filter_static_hits", True)
        use_plan_radius = self._get_param_cached("~blocked_use_plan_radius", True)

        kept_hits = 0
        dropped_hits = 0
        new_cells = []
        for k, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < max(msg.range_min, 0.22) or r > msg.range_max:
                continue
            beam_angle = msg.angle_min + k * msg.angle_increment
            total_angle = robot_yaw + beam_angle
            px = robot_x + r * math.cos(total_angle)
            py = robot_y + r * math.sin(total_angle)
            ci = int(math.floor((px - ox) / res))
            cj = int(math.floor((py - oy) / res))
            if 0 <= ci < w and 0 <= cj < h:
                # Extrude 0.10m deeper along beam vector into obstacle volume
                p_depth_x = px + 0.10 * math.cos(total_angle)
                p_depth_y = py + 0.10 * math.sin(total_angle)
                di_depth = int(math.floor((p_depth_x - ox) / res))
                dj_depth = int(math.floor((p_depth_y - oy) / res))
                depth_cell = (di_depth, dj_depth) if (0 <= di_depth < w and 0 <= dj_depth < h) else None

                hit_cells, dropped = expand_lidar_hit(grid, ci, cj, depth_cell, filter_static)
                if dropped:
                    dropped_hits += 1
                else:
                    kept_hits += 1
                    new_cells.extend(hit_cells)

        with self.obstacle_lock:
            for cell in new_cells:
                self.obstacle_cells[cell] = now
            # Prune obstacles older than memory window (default 6.0s, e.g. moving NPCs)
            obstacle_memory_s = float(self._get_param_cached("~obstacle_memory_s", 6.0))
            expire_time = now - obstacle_memory_s
            self.obstacle_cells = {
                c: t for c, t in self.obstacle_cells.items() if t >= expire_time
            }
            curr_cells = list(self.obstacle_cells.keys())
            curr_obstacle_points = [
                (maps.grid.ox + (ci + 0.5) * maps.grid.res, maps.grid.oy + (cj + 0.5) * maps.grid.res)
                for ci, cj in curr_cells
            ]

        # 1. Real-time live costmap publishing: every 0.5s (2 Hz) if dynamic obstacles exist,
        # inflate and publish to /costmap so RViz shows the obstacle immediately (never too late!)
        if curr_obstacle_points and (now - self.last_costmap_pub_time >= 0.5):
            self.last_costmap_pub_time = now
            t_live0 = time.time()
            self.publish_live_costmap(curr_cells, curr_obstacle_points)
            live_costmap_ms = (time.time() - t_live0) * 1000.0
            rospy.loginfo_throttle(
                5.0,
                "live costmap: %.0f ms (%d cells)",
                live_costmap_ms,
                len(curr_obstacle_points),
            )

        # 2. Early automatic replan: if dynamic obstacle blocks the currently planned path ahead,
        # trigger replan immediately without waiting for robot to get close!
        if use_plan_radius and self.current_path_radius is not None:
            check_radius = max(0.05, self.current_path_radius - 0.02)
        else:
            check_radius = 0.38

        rospy.loginfo_throttle(
            5.0,
            "scan overlay: kept %d, dropped_static %d, filter=%s, check_radius=%.2f",
            kept_hits,
            dropped_hits,
            filter_static,
            check_radius,
        )

        if (
            curr_obstacle_points
            and self.current_path_corners
            and self.latest_goal is not None
            and (now - self.last_auto_replan_time >= 1.0)
        ):
            if self.is_path_blocked(robot_x, robot_y, curr_obstacle_points, radius=check_radius):
                if self._get_param_cached("~trigger_debug", False):
                    self._log_trigger_debug(robot_x, robot_y, curr_obstacle_points, check_radius)
                rospy.loginfo(
                    "Dynamic obstacle detected blocking current planned path ahead! Triggering early replan... (check radius %.2f)",
                    check_radius,
                )
                self.last_auto_replan_time = now
                self.plan_for_goal(self.latest_goal)

    def _log_trigger_debug(
        self,
        rx: float,
        ry: float,
        obstacle_points: List[Tuple[float, float]],
        check_radius: float,
    ) -> None:
        corners = self.current_path_corners
        if not corners or len(corners) < 2 or not obstacle_points:
            return

        point_records = []
        n_within_radius = 0
        max_range = 3.5

        for ox, oy in obstacle_points:
            if math.hypot(ox - rx, oy - ry) > max_range:
                continue

            min_d = float("inf")
            best_k = 0
            best_px = 0.0
            best_py = 0.0

            for k in range(len(corners) - 1):
                ax, ay = corners[k]
                bx, by = corners[k + 1]
                seg_dx = bx - ax
                seg_dy = by - ay
                seg_len = math.hypot(seg_dx, seg_dy)
                if seg_len < 1e-4:
                    continue
                t = max(0.0, min(1.0, ((ox - ax) * seg_dx + (oy - ay) * seg_dy) / (seg_len * seg_len)))
                px = ax + t * seg_dx
                py = ay + t * seg_dy
                d = math.hypot(ox - px, oy - py)
                if d < min_d:
                    min_d = d
                    best_k = k
                    best_px = px
                    best_py = py

            if min_d < float("inf"):
                if min_d < check_radius:
                    n_within_radius += 1
                point_records.append((ox, oy, min_d, best_k, best_px, best_py))

        if not point_records:
            return

        point_records.sort(key=lambda r: r[2])
        best = point_records[0]
        top3 = point_records[:3]
        pts_str = " ".join(f"({r[0]:.2f},{r[1]:.2f},{r[2]:.3f})" for r in top3)

        rospy.loginfo(
            "TRIGGER_DEBUG robot=(%.2f,%.2f) nearest_pt=(%.2f,%.2f) dist_to_polyline=%.3f seg_index=%d n_obstacle_pts=%d n_within_radius=%d radius=%.2f %s",
            rx,
            ry,
            best[4],
            best[5],
            best[2],
            best[3],
            len(obstacle_points),
            n_within_radius,
            check_radius,
            pts_str,
        )

    def publish_live_costmap(
        self,
        cells: List[Tuple[int, int]],
        obstacle_points: Optional[List[Tuple[float, float]]] = None,
    ) -> None:
        if self.maps is None:
            return
        radius = finite_or(
            rospy.get_param("/nav/inflation_radius", DEFAULT_INFLATION),
            DEFAULT_INFLATION,
            0.0,
        )
        if (
            self._live_inc is None
            or self.maps is not self._live_maps
            or radius != self._live_radius
        ):
            self._live_inc = IncrementalInflation(self.maps.get(radius), radius)
            self._live_maps = self.maps
            self._live_radius = radius

        self._live_inc.sync(cells)
        self.publish_costmap(self._live_inc.view)

        if rospy.get_param("~verify_costmap", False):
            # Note: A rare false mismatch is possible if another plan runs in between.
            if obstacle_points is None:
                obstacle_points = [
                    (self.maps.grid.ox + (ci + 0.5) * self.maps.grid.res, self.maps.grid.oy + (cj + 0.5) * self.maps.grid.res)
                    for ci, cj in cells
                ]
            old_grid = inflate(overlay_obstacles(self.maps.grid, obstacle_points), radius)
            old_vals = [100 if c else 0 for c in old_grid.data]
            new_vals = [100 if c else 0 for c in self._live_inc.view.data]
            diff_count = sum(1 for o, n in zip(old_vals, new_vals) if o != n)
            if diff_count == 0:
                rospy.loginfo_throttle(5.0, "costmap check (live): OK")
            else:
                rospy.logerr_throttle(5.0, "costmap check (live): MISMATCH, %d cells differ", diff_count)

    def is_path_blocked(
        self,
        rx: float,
        ry: float,
        obstacle_points: List[Tuple[float, float]],
        radius: float = 0.38,
    ) -> bool:
        """Check if any dynamic obstacle point is within `radius` of the planned path ahead of the robot."""
        return path_near_points(
            self.current_path_corners,
            obstacle_points,
            (rx, ry),
            radius,
            max_range=3.5,
        )

    def on_goal(self, msg: PoseStamped) -> None:
        self.latest_goal = msg
        self.plan_for_goal(msg)

    def on_replan(self, _msg: Empty) -> None:
        if self.latest_goal is not None:
            rospy.loginfo("Replan requested: searching alternative path around obstacles...")
            self.plan_for_goal(self.latest_goal)

    def plan_for_goal(self, msg: PoseStamped) -> None:
        maps, pose, planner = self.maps, self.pose, self.planner
        if maps is None or pose is None or planner is None:
            return self.abort("no /map or no /robot/pose received yet")
        frame = msg.header.frame_id
        if frame not in ("", "map"):
            return self.abort("the goal is in frame '%s', not 'map'" % frame)
        age = (rospy.Time.now() - pose.header.stamp).to_sec()
        if age > POSE_MAX_AGE:
            return self.abort("the last /robot/pose is %.1f s old" % age)

        start = (pose.pose.position.x, pose.pose.position.y)
        goal = (msg.pose.position.x, msg.pose.position.y)
        # Read at every plan: the mission changes it with the category profile.
        radius = finite_or(
            rospy.get_param("/nav/inflation_radius", DEFAULT_INFLATION),
            DEFAULT_INFLATION,
            0.0,
        )

        with self.obstacle_lock:
            t_obs0 = time.time()
            planner.set_obstacles(self.obstacle_cells.keys())
            set_obs_ms = (time.time() - t_obs0) * 1000.0
            obstacle_points = [
                (maps.grid.ox + (ci + 0.5) * maps.grid.res, maps.grid.oy + (cj + 0.5) * maps.grid.res)
                for ci, cj in self.obstacle_cells.keys()
            ]

        t0 = time.time()

        # Multi-stage adaptive inflation: try safe inflation, then tighter if corridor is constricted
        # Guarantee safe clearance: try wide inflation (0.40m), then nominal (0.35m),
        # down to 0.28m floor. Never go below 0.28m so robot (radius 0.21m) never touches obstacle!
        candidate_radii = [max(radius, 0.40), radius]
        if radius > 0.30:
            candidate_radii.append(0.30)
        candidate_radii.append(0.28)
        radii_to_try = []
        for r_val in candidate_radii:
            if r_val not in radii_to_try:
                radii_to_try.append(r_val)

        try:
            corners = planner.plan(
                start,
                goal,
                self.snap_distance,
                radii_to_try,
                now=rospy.Time.now().to_sec(),
            )
        except NoPath:
            return self.abort("No path found around obstacles")

        if corners is None:
            return self.abort("No path found around obstacles")

        stats = planner.stats
        chosen_radius = stats.radius if stats else radius
        expanded = stats.expanded if stats else 0
        rebuilt = stats.rebuilt if stats else True
        level = stats.level if stats else 0
        search_ms = stats.ms if stats else 0.0
        changed = stats.changed_cells if stats else 0
        cpu_ms = stats.cpu_ms if stats else 0.0
        build_ms = stats.build_ms if stats else 0.0
        sync_ms = stats.sync_ms if stats else 0.0
        cells_ms = stats.cells_ms if stats else 0.0
        compute_ms = stats.compute_ms if stats else 0.0
        path_ms = stats.path_ms if stats else 0.0
        format_ms = stats.format_ms if stats else 0.0
        raw_added = stats.raw_added if stats else 0
        raw_removed = stats.raw_removed if stats else 0

        # Publish path first so the robot receives its new path immediately
        t_ppub0 = time.time()
        self.current_path_radius = chosen_radius
        self.publish_path(corners)
        path_pub_ms = (time.time() - t_ppub0) * 1000.0
        path_ready_ms = (time.time() - t0) * 1000.0

        # Costmap publication: publish live-overlay costmap at chosen_radius
        t_cinf0 = time.time()
        chosen_grid = planner.view_snapshot()
        if chosen_grid is None:
            active_grid = overlay_obstacles(maps.grid, obstacle_points)
            chosen_grid = inflate(active_grid, chosen_radius)
        costmap_inflate_ms = (time.time() - t_cinf0) * 1000.0

        if rospy.get_param("~verify_costmap", False):
            # Note: A rare false mismatch is possible if another plan runs in between.
            old_grid = inflate(overlay_obstacles(maps.grid, obstacle_points), chosen_radius)
            old_vals = [100 if c else 0 for c in old_grid.data]
            new_vals = [100 if c else 0 for c in chosen_grid.data]
            diff_count = sum(1 for o, n in zip(old_vals, new_vals) if o != n)
            if diff_count == 0:
                rospy.loginfo_throttle(5.0, "costmap check (post-plan): OK")
            else:
                rospy.logerr_throttle(5.0, "costmap check (post-plan): MISMATCH, %d cells differ", diff_count)

        t_cpub0 = time.time()
        self.publish_costmap(chosen_grid)
        costmap_pub_ms = (time.time() - t_cpub0) * 1000.0

        length = sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(corners, corners[1:])
        )
        rospy.loginfo(
            "Planned: %d corners, %.1f m, %.0f ms, inflation %.2f m (%d dynamic obstacles), expanded=%d, rebuilt=%s, level=%d, search=%.0f ms, changed=%d, set_obs=%.0f ms, costmap_inflate=%.0f ms, costmap_pub=%.0f ms, path_pub=%.0f ms, path_ready=%.0f ms, cpu=%.0f ms, build=%.0f ms, sync=%.0f ms, cells=%.0f ms, compute=%.0f ms, path=%.0f ms, format=%.0f ms, raw=+%d/-%d",
            len(corners),
            length,
            (time.time() - t0) * 1000.0,
            chosen_radius,
            len(obstacle_points),
            expanded,
            rebuilt,
            level,
            search_ms,
            changed,
            set_obs_ms,
            costmap_inflate_ms,
            costmap_pub_ms,
            path_pub_ms,
            path_ready_ms,
            cpu_ms,
            build_ms,
            sync_ms,
            cells_ms,
            compute_ms,
            path_ms,
            format_ms,
            raw_added,
            raw_removed,
        )
        if corners[-1] != goal:
            rospy.logwarn(
                "Goal (%.2f, %.2f) is blocked or unreachable: snapped to (%.2f, %.2f).",
                goal[0],
                goal[1],
                corners[-1][0],
                corners[-1][1],
            )
        for k, (x, y) in enumerate(corners):
            rospy.loginfo("  corner %d: x=%.2f y=%.2f", k, x, y)
        if self.print_map and maps.grid.height <= MAX_PICTURE_HEIGHT:
            rospy.loginfo("\n%s", ascii_map(chosen_grid, corners))

    # ---------------- output ----------------
    def abort(self, why: str) -> None:
        rospy.logwarn("No path: %s", why)
        self.result_pub.publish(String(data=leg_result(ABORTED, NO_PATH)))

    def publish_costmap(self, grid: Grid) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.info.resolution = grid.res
        msg.info.width = grid.width
        msg.info.height = grid.height
        msg.info.origin.position.x = grid.ox
        msg.info.origin.position.y = grid.oy
        msg.info.origin.orientation.w = 1.0
        msg.data = [100 if cell else 0 for cell in grid.data]
        self.costmap_pub.publish(msg)

    def publish_path(self, corners) -> None:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for k, (x, y) in enumerate(corners):
            if k < len(corners) - 1:
                yaw = math.atan2(corners[k + 1][1] - y, corners[k + 1][0] - x)
            elif k > 0:
                yaw = math.atan2(y - corners[k - 1][1], x - corners[k - 1][0])
            else:
                yaw = 0.0
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self.current_path_corners = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        self.path_pub.publish(path)


def same_map(a: Grid, b: Grid) -> bool:
    """True when two /map messages describe the same grid."""
    return (
        (a.width, a.height, a.res, a.ox, a.oy) == (b.width, b.height, b.res, b.ox, b.oy)
        and tuple(a.data) == tuple(b.data)
    )


if __name__ == "__main__":
    rospy.init_node("dstar_planner")
    DStarPlanner()
    rospy.spin()
