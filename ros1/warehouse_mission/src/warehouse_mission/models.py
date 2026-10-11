"""Data models and constants for the UC6 Warehouse Robot mission layer.

Conforms to docs/architecture.md and docs/data-model.md.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Any
import math

# Category constants
CATEGORY_STANDARD = "Standard"
CATEGORY_HEAVY = "Heavy"
CATEGORY_FRAGILE = "Fragile"
ALLOWED_CATEGORIES = (CATEGORY_STANDARD, CATEGORY_HEAVY, CATEGORY_FRAGILE)

# Leg constants
LEG_SHELF = "shelf"
LEG_DROPOFF = "dropoff"
ALLOWED_LEGS = (LEG_SHELF, LEG_DROPOFF)

# Outcome constants
OUTCOME_SUCCESS = "success"
OUTCOME_FAIL = "fail"
ALLOWED_OUTCOMES = (OUTCOME_SUCCESS, OUTCOME_FAIL)

# Fail reason constants
FAIL_NAV_ABORTED = "nav_aborted"
FAIL_TIMEOUT = "timeout"
FAIL_ITEM_ERROR = "item_error"
FAIL_BRIDGE_LOST = "bridge_lost"
ALLOWED_FAIL_REASONS = ("", FAIL_NAV_ABORTED, FAIL_TIMEOUT, FAIL_ITEM_ERROR, FAIL_BRIDGE_LOST)

# Event type constants
EVENT_COLLISION = "collision"
EVENT_REPLAN = "replan"
EVENT_RECOVERY = "recovery"
EVENT_WARNING = "warning"
ALLOWED_EVENT_TYPES = (EVENT_COLLISION, EVENT_REPLAN, EVENT_RECOVERY, EVENT_WARNING)


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float = 0.0

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass
class Task:
    task_id: int
    item_id: int
    category: str
    shelf_pose: Pose2D
    dropoff_pose: Pose2D
    scenario_id: str = "S-00"

    def __post_init__(self):
        if self.category not in ALLOWED_CATEGORIES:
            raise ValueError(f"Invalid category '{self.category}', must be one of {ALLOWED_CATEGORIES}")


@dataclass
class RunEvent:
    event_type: str
    sim_time_s: float
    payload: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        if self.event_type not in ALLOWED_EVENT_TYPES:
            raise ValueError(f"Invalid event_type '{self.event_type}', must be one of {ALLOWED_EVENT_TYPES}")


@dataclass
class RunSummary:
    task_id: int
    scenario_id: str
    planner: str
    outcome: str
    fail_reason: str = ""
    start_x: float = float("nan")
    start_y: float = float("nan")
    collisions: int = 0
    replans: int = 0
    recoveries: int = 0
    min_clearance_m: float = float("nan")
    path_length_m: float = 0.0
    baseline_m: float = 0.0
    path_ratio: float = float("nan")
    duration_s: float = 0.0
    dropoff_mean_speed_mps: float = float("nan")
    started_at: str = ""
    ended_at: str = ""
    events: List[RunEvent] = field(default_factory=list)

    def __post_init__(self):
        if self.outcome not in ALLOWED_OUTCOMES:
            raise ValueError(f"Invalid outcome '{self.outcome}', must be one of {ALLOWED_OUTCOMES}")
        if self.fail_reason not in ALLOWED_FAIL_REASONS:
            raise ValueError(f"Invalid fail_reason '{self.fail_reason}', must be one of {ALLOWED_FAIL_REASONS}")
        if self.baseline_m > 0 and not math.isnan(self.path_length_m):
            self.path_ratio = self.path_length_m / self.baseline_m
