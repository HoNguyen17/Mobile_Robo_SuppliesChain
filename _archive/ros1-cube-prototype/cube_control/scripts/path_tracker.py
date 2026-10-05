"""Waypoint tracking for path_follower: the path state and the P controller.

No ROS imports, so it runs (and is unit tested) anywhere. ROS callbacks run on their
own threads, so every read or change of the path goes through one lock.
"""
import math
import threading
from dataclasses import dataclass
from typing import Sequence, Tuple

K_LIN = 0.5  # forward speed per metre to the last waypoint (final approach only)
K_ANG = 1.5  # turn rate per radian of heading error
MAX_ANG = 1.0  # rad/s
TURN_FIRST = 0.5  # rad: with a bigger heading error the robot turns on the spot first
FINAL_TOL = 0.1  # m: the goal counts as reached inside this distance

IDLE = "idle"  # no path: nothing to do
DRIVING = "driving"  # follow the command
ARRIVED = "arrived"  # the last waypoint was just reached; the path is cleared
STALE = "stale"  # the pose is too old to steer by: stop and wait

Point = Tuple[float, float]


@dataclass(frozen=True)
class Command:
    linear: float  # m/s
    angular: float  # rad/s
    status: str


_IDLE = Command(0.0, 0.0, IDLE)
_ARRIVED = Command(0.0, 0.0, ARRIVED)
_STALE = Command(0.0, 0.0, STALE)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    """Yaw of anything with x, y, z, w (a geometry_msgs/Quaternion)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathTracker:
    def __init__(self, waypoint_tol: float = 0.3, pose_timeout: float = 0.5) -> None:
        self.waypoint_tol = waypoint_tol  # m, for waypoints before the last
        self.pose_timeout = pose_timeout  # s, the oldest pose that is still trusted
        self._lock = threading.Lock()
        self._waypoints: Tuple[Point, ...] = ()
        self._index = 0

    @property
    def active(self) -> bool:
        return bool(self._waypoints)

    def set_path(self, points: Sequence[Point]) -> None:
        """Follow a new path. An empty path is ignored."""
        waypoints = tuple((float(x), float(y)) for x, y in points)
        if not waypoints:
            return
        with self._lock:
            self._waypoints = waypoints
            # Waypoint 0 is where the robot was when it planned: head for waypoint 1.
            self._index = 1 if len(waypoints) > 1 else 0

    def cancel(self) -> bool:
        """Drop the path. True if there was one."""
        with self._lock:
            was_active = bool(self._waypoints)
            self._waypoints = ()
            return was_active

    def step(
        self, x: float, y: float, yaw: float, pose_age: float, max_lin: float
    ) -> Command:
        """One control step for pose (x, y, yaw), which is `pose_age` seconds old."""
        with self._lock:
            if not self._waypoints:
                return _IDLE
            if not pose_age <= self.pose_timeout or not all(
                math.isfinite(v) for v in (x, y, yaw)
            ):
                return _STALE
            last = len(self._waypoints) - 1
            while True:
                wx, wy = self._waypoints[self._index]
                dist = math.hypot(wx - x, wy - y)
                if self._index == last:
                    if dist < FINAL_TOL:
                        self._waypoints = ()
                        return _ARRIVED
                    break
                if dist >= self.waypoint_tol:
                    break
                self._index += 1  # close enough: chase the next one
            is_last = self._index == last

        heading_error = normalize(math.atan2(wy - y, wx - x) - yaw)
        angular = clamp(K_ANG * heading_error, -MAX_ANG, MAX_ANG)
        if abs(heading_error) > TURN_FIRST:
            linear = 0.0  # pivot on the spot
        elif is_last:
            linear = clamp(K_LIN * dist, 0.0, max_lin)  # slow down towards the goal
        else:
            linear = max_lin  # keep flowing through corners
        return Command(linear, angular, DRIVING)
