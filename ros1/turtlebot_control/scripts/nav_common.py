"""Small things the planner and the follower share. No ROS imports."""
import json
import math
from typing import Any, Callable, Optional

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


def wait_for_clock(
    now: Callable[[], float],
    sleep: Callable[[float], None],
    is_shutdown: Callable[[], bool] = lambda: False,
    on_wait: Optional[Callable[[float], None]] = None,
    poll: float = 0.05,
    report_every: float = 2.0,
) -> bool:
    """Block until the simulation clock runs, i.e. now() > 0 (ADR-009).

    With use_sim_time the clock reads 0 until Unity's first /clock message arrives, and
    rospy.Rate would wait forever. `sleep` must be a wall-clock sleep. `on_wait(seconds)` is
    called every `report_every` seconds of waiting. Returns False if ROS shuts down first.
    """
    waited = 0.0
    next_report = report_every
    while now() <= 0.0:
        if is_shutdown():
            return False
        sleep(poll)
        waited += poll
        if on_wait is not None and waited >= next_report - 1e-9:
            on_wait(waited)
            next_report += report_every
    return True


class ClockWatch:
    """Notices that the simulation clock restarted (Unity pressed Play again)."""

    def __init__(self, tolerance: float = 1.0) -> None:
        self.tolerance = tolerance  # s: a smaller step back is jitter, not a restart
        self._last: Optional[float] = None

    def jumped_back(self, now: float) -> bool:
        """True when `now` is more than `tolerance` before the previous reading."""
        last, self._last = self._last, now
        return last is not None and now < last - self.tolerance
