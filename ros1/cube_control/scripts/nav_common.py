"""Small things the planner and the follower share. No ROS imports."""
import json
import math
from typing import Any

# /nav/leg_result payload values (docs/architecture.md section 5)
SUCCEEDED = "succeeded"
ABORTED = "aborted"
NO_PATH = "no_path"
CANCELLED = "cancelled"


def leg_result(outcome: str, reason: str = "") -> str:
    """JSON text for a std_msgs/String on /nav/leg_result."""
    return json.dumps({"outcome": outcome, "reason": reason})


def finite_or(value: Any, default: float, minimum: float) -> float:
    """float(value) if it is a finite number of at least `minimum`, else `default`.

    For ROS parameters, which anyone can set to anything at run time.
    """
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) and number >= minimum else default
