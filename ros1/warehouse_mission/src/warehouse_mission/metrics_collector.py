"""Metrics collector for UC6 Warehouse Robot.

Conforms to docs/architecture.md §10.
"""

import math
from typing import Optional, List
import logging

from warehouse_mission.models import (
    Task,
    RunSummary,
    RunEvent,
    Pose2D,
    LEG_SHELF,
    LEG_DROPOFF,
    EVENT_COLLISION,
    EVENT_REPLAN,
    EVENT_RECOVERY,
)

logger = logging.getLogger(__name__)

ROBOT_FOOTPRINT_RADIUS_M = 0.257


class MetricsCollector:
    """Collects episode performance metrics according to architecture.md §10."""

    def __init__(self, planner_name: str = "astar"):
        self.planner_name = planner_name
        self.current_task: Optional[Task] = None
        self.current_leg: Optional[str] = None
        self.is_active: bool = False

        self.start_sim_time: float = 0.0
        self.start_pose: Optional[Pose2D] = None
        self.last_pose: Optional[Pose2D] = None
        self.last_pose_time: float = 0.0

        self.collisions: int = 0
        self.replans: int = 0
        self.recoveries: int = 0
        self.plans_in_current_leg: int = 0
        self.min_clearance_m: float = float("inf")
        self.path_length_m: float = 0.0

        # Speeds during drop-off leg
        self.dropoff_distances: float = 0.0
        self.dropoff_duration: float = 0.0

        self.events: List[RunEvent] = []

    def start_run(self, task: Task, start_x: float, start_y: float, sim_time_s: float):
        """Initialise metrics tracking for a new episode."""
        self.current_task = task
        self.current_leg = None
        self.is_active = True

        self.start_sim_time = sim_time_s
        self.start_pose = Pose2D(start_x, start_y)
        self.last_pose = Pose2D(start_x, start_y)
        self.last_pose_time = sim_time_s

        self.collisions = 0
        self.replans = 0
        self.recoveries = 0
        self.plans_in_current_leg = 0
        self.min_clearance_m = float("inf")
        self.path_length_m = 0.0

        self.dropoff_distances = 0.0
        self.dropoff_duration = 0.0

        self.events = []

    def mark_leg(self, leg_name: str):
        """Mark start of a leg (shelf or dropoff). Resets per-leg plan count."""
        self.current_leg = leg_name
        self.plans_in_current_leg = 0

    def on_collision(self, other_tag: str, sim_time_s: float, x: float = 0.0, y: float = 0.0):
        """Handle a collision event from /sim/collision."""
        if not self.is_active:
            return
        self.collisions += 1
        ev = RunEvent(
            event_type=EVENT_COLLISION,
            sim_time_s=sim_time_s - self.start_sim_time,
            payload={"other_tag": other_tag, "x": x, "y": y},
        )
        self.events.append(ev)

    def on_planned_path(self, sim_time_s: float):
        """Handle a new path from /planned_path."""
        if not self.is_active:
            return
        self.plans_in_current_leg += 1
        if self.plans_in_current_leg > 1:
            self.replans += 1
            ev = RunEvent(
                event_type=EVENT_REPLAN,
                sim_time_s=sim_time_s - self.start_sim_time,
                payload={"leg": self.current_leg},
            )
            self.events.append(ev)

    def on_nav_event(self, event_type: str, sim_time_s: float, payload: Optional[dict] = None):
        """Handle an event from /nav/event."""
        if not self.is_active:
            return
        if event_type == "recovery":
            self.recoveries += 1
            ev = RunEvent(
                event_type=EVENT_RECOVERY,
                sim_time_s=sim_time_s - self.start_sim_time,
                payload=payload or {},
            )
            self.events.append(ev)

    def on_scan(self, ranges: List[float]):
        """Update minimum clearance from /scan."""
        if not self.is_active or not ranges:
            return
        # Filter valid lidar readings > 0.12 m
        valid_ranges = [r for r in ranges if not math.isnan(r) and not math.isinf(r) and r >= 0.12]
        if valid_ranges:
            clearance = min(valid_ranges) - ROBOT_FOOTPRINT_RADIUS_M
            if clearance < self.min_clearance_m:
                self.min_clearance_m = clearance

    def on_pose(self, x: float, y: float, sim_time_s: float):
        """Update odometer and speed calculation from /robot/pose."""
        if not self.is_active:
            return
        curr_pose = Pose2D(x, y)
        if self.last_pose is not None:
            dist = curr_pose.distance_to(self.last_pose)
            self.path_length_m += dist

            dt = sim_time_s - self.last_pose_time
            if self.current_leg == LEG_DROPOFF and dt > 0:
                self.dropoff_distances += dist
                self.dropoff_duration += dt

        self.last_pose = curr_pose
        self.last_pose_time = sim_time_s

    def finish_run(
        self, outcome: str, fail_reason: str = "", sim_time_s: float = 0.0
    ) -> RunSummary:
        """Complete the run and return the computed RunSummary."""
        self.is_active = False
        duration_s = max(0.0, sim_time_s - self.start_sim_time)

        # Baseline: start -> shelf -> dropoff
        baseline_m = 0.0
        start_x = self.start_pose.x if self.start_pose else float("nan")
        start_y = self.start_pose.y if self.start_pose else float("nan")

        if self.current_task and self.start_pose:
            d1 = self.start_pose.distance_to(self.current_task.shelf_pose)
            d2 = self.current_task.shelf_pose.distance_to(self.current_task.dropoff_pose)
            baseline_m = d1 + d2

        dropoff_speed = float("nan")
        if self.dropoff_duration > 0.0:
            dropoff_speed = self.dropoff_distances / self.dropoff_duration

        min_clr = self.min_clearance_m if self.min_clearance_m != float("inf") else float("nan")

        task_id = self.current_task.task_id if self.current_task else 0
        scenario_id = self.current_task.scenario_id if self.current_task else "S-00"

        summary = RunSummary(
            task_id=task_id,
            scenario_id=scenario_id,
            planner=self.planner_name,
            outcome=outcome,
            fail_reason=fail_reason,
            start_x=start_x,
            start_y=start_y,
            collisions=self.collisions,
            replans=self.replans,
            recoveries=self.recoveries,
            min_clearance_m=min_clr,
            path_length_m=self.path_length_m,
            baseline_m=baseline_m,
            duration_s=duration_s,
            dropoff_mean_speed_mps=dropoff_speed,
            events=list(self.events),
        )
        return summary
