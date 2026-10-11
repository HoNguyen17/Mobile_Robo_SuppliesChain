"""Closed-loop check without ROS: plan on a map in robot metres, follow the plan with the real
follower logic, and move a simulated Waffle Pi that behaves like Unity's DiffDriveController
(speed limits, acceleration ramp, 0.5 s watchdog, some wheel slip, pose published at 30 Hz)."""
import math
import unittest

import support  # noqa: F401  (puts scripts/ on sys.path)
from grid_planner import plan
from path_tracker import ARRIVED, PathTracker
from support import clearance, rooms

CONTROL_DT = 0.05  # the follower runs at 20 Hz
PHYSICS_DT = 0.01
POSE_PERIOD = 1.0 / 30.0  # /robot/pose rate
FOOTPRINT_RADIUS = 0.257  # Waffle Pi: farthest corner from the wheel axis centre


def move_towards(value, target, step):
    if abs(target - value) <= step:
        return target
    return value + math.copysign(step, target - value)


class WafflePi:
    """Like DiffDriveController: clamp the command, ramp the speed, stop without commands.

    `slip` scales what the wheels really achieve (ADR-010 measured 0.94 m instead of 0.98 m
    going straight and -152 instead of -172 degrees turning).
    """

    MAX_LIN, MAX_ANG = 0.26, 1.82
    ACC_LIN, ACC_ANG = 1.0, 3.0
    WATCHDOG = 0.5

    def __init__(self, x, y, yaw, slip_lin=0.96, slip_ang=0.88):
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.cmd_time = -1e9
        self.slip_lin, self.slip_ang = slip_lin, slip_ang

    def command(self, linear, angular, now):
        self.cmd = (
            max(-self.MAX_LIN, min(self.MAX_LIN, linear)),
            max(-self.MAX_ANG, min(self.MAX_ANG, angular)),
        )
        self.cmd_time = now

    def step(self, now):
        v, w = self.cmd if now - self.cmd_time <= self.WATCHDOG else (0.0, 0.0)
        self.v = move_towards(self.v, v, self.ACC_LIN * PHYSICS_DT)
        self.w = move_towards(self.w, w, self.ACC_ANG * PHYSICS_DT)
        self.yaw += self.w * self.slip_ang * PHYSICS_DT
        self.x += self.v * self.slip_lin * math.cos(self.yaw) * PHYSICS_DT
        self.y += self.v * self.slip_lin * math.sin(self.yaw) * PHYSICS_DT


def distance_to_polyline(point, corners):
    best = math.inf
    for (ax, ay), (bx, by) in zip(corners, corners[1:]):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0
        if length2 > 0.0:
            t = max(0.0, min(1.0, ((point[0] - ax) * dx + (point[1] - ay) * dy) / length2))
        best = min(best, math.hypot(point[0] - (ax + t * dx), point[1] - (ay + t * dy)))
    return best


def drive(grid, start, goal, yaw=0.0, inflation_radius=0.35, max_lin=0.26, limit_s=300.0, snap=2.0,
          pose_delay=0.0):
    """Plan, then follow the plan until the follower reports arrival.

    Returns a dict: the robot, the trail of (x, y), the corners, the seconds taken, and the worst
    clearance and cross-track error. `pose_delay` is extra latency of /robot/pose, in seconds.
    Fails if the robot does not arrive in time.
    """
    corners = plan(grid, start, goal, snap, inflation_radius)
    tracker = PathTracker()
    tracker.set_path(corners)
    robot = WafflePi(start[0], start[1], yaw)
    trail = [(robot.x, robot.y)]
    samples = [(0.0, (robot.x, robot.y, robot.yaw))]  # what /robot/pose has said so far
    next_pose = POSE_PERIOD
    now, next_control = 0.0, 0.0
    worst_clearance, worst_cross = math.inf, 0.0
    while now < limit_s:
        if now >= next_pose:
            samples.append((now, (robot.x, robot.y, robot.yaw)))
            next_pose += POSE_PERIOD
        if now >= next_control:
            next_control += CONTROL_DT
            pose_time, pose = max((t, p) for t, p in samples if t <= now - pose_delay or t == samples[0][0])
            cmd = tracker.step(pose[0], pose[1], pose[2], now - pose_time, max_lin)
            if cmd.status == ARRIVED:
                return {
                    "robot": robot,
                    "trail": trail,
                    "corners": corners,
                    "seconds": now,
                    "clearance": worst_clearance,
                    "cross_track": worst_cross,
                }
            robot.command(cmd.linear, cmd.angular, now)
        robot.step(now)
        now += PHYSICS_DT
        if round(now / PHYSICS_DT) % 5 == 0:
            trail.append((robot.x, robot.y))
            worst_clearance = min(worst_clearance, clearance(grid, robot.x, robot.y))
            worst_cross = max(worst_cross, distance_to_polyline((robot.x, robot.y), corners))
    raise AssertionError(
        "did not arrive within %.0f s, stopped at (%.2f, %.2f)" % (limit_s, robot.x, robot.y)
    )


def off_goal(result, goal):
    return math.hypot(result["robot"].x - goal[0], result["robot"].y - goal[1])


class ClosedLoopTest(unittest.TestCase):
    def test_crosses_an_open_room_and_stops_at_the_goal(self):
        grid = rooms(10.0, 8.0, 0.05, [])
        goal = (9.0, 7.0)
        result = drive(grid, (1.0, 1.0), goal)
        self.assertLess(off_goal(result, goal), 0.15)
        self.assertLess(result["seconds"], 80.0)  # ~10 m at 0.26 m/s
        self.assertLess(result["cross_track"], 0.10)  # a straight leg stays on its line

    def test_a_robot_facing_away_turns_round_first(self):
        grid = rooms(10.0, 8.0, 0.05, [])
        goal = (9.0, 4.0)
        result = drive(grid, (1.0, 4.0), goal, yaw=math.pi)
        self.assertLess(off_goal(result, goal), 0.15)

    def test_goes_through_the_gap_in_a_wall_without_touching_it(self):
        # A wall at x = 5 with a 1.4 m opening, in a 10 x 8 m room.
        grid = rooms(10.0, 8.0, 0.05, [(4.9, 0.0, 5.1, 3.0), (4.9, 4.4, 5.1, 8.0)])
        goal = (9.0, 1.0)
        result = drive(grid, (1.0, 1.0), goal)
        self.assertLess(off_goal(result, goal), 0.15)
        self.assertGreater(result["clearance"], FOOTPRINT_RADIUS, "the footprint circle touched the wall")

    def test_a_one_metre_corridor_between_boxes_is_passable(self):
        # S-02 style: boxes along both sides leave 1.0 m, so with inflation 0.35 the centre line is 0.3 m wide.
        grid = rooms(10.0, 6.0, 0.05, [(3.0, 0.0, 7.0, 2.5), (3.0, 3.5, 7.0, 6.0)])
        goal = (9.0, 3.0)
        result = drive(grid, (1.0, 3.0), goal)
        self.assertLess(off_goal(result, goal), 0.15)
        self.assertGreater(result["clearance"], FOOTPRINT_RADIUS - 0.02)

    def test_a_zigzag_between_walls_keeps_the_footprint_clear(self):
        # Three walls force sharp turns at 0.15 m from their ends once inflated.
        walls = [(2.5, 0.0, 2.7, 6.0), (5.0, 2.0, 5.2, 8.0), (7.5, 0.0, 7.7, 6.0)]
        grid = rooms(10.0, 8.0, 0.05, walls)
        goal = (9.0, 1.0)
        result = drive(grid, (1.0, 1.0), goal)
        self.assertLess(off_goal(result, goal), 0.15)
        self.assertGreater(result["clearance"], FOOTPRINT_RADIUS, "a corner was cut into the footprint")

    def test_a_late_pose_still_gets_the_robot_there_cleanly(self):
        # 0.15 s of extra latency on /robot/pose, on top of its 30 Hz rate.
        walls = [(2.5, 0.0, 2.7, 6.0), (5.0, 2.0, 5.2, 8.0), (7.5, 0.0, 7.7, 6.0)]
        grid = rooms(10.0, 8.0, 0.05, walls)
        goal = (9.0, 1.0)
        result = drive(grid, (1.0, 1.0), goal, pose_delay=0.15)
        self.assertLess(off_goal(result, goal), 0.15)
        self.assertGreater(result["clearance"], FOOTPRINT_RADIUS)

    def test_a_slower_speed_limit_is_obeyed_all_the_way(self):
        grid = rooms(10.0, 2.0, 0.05, [])
        fast = drive(grid, (1.0, 1.0), (9.0, 1.0), max_lin=0.26)["seconds"]
        slow = drive(grid, (1.0, 1.0), (9.0, 1.0), max_lin=0.10)["seconds"]
        self.assertGreater(slow, 2.0 * fast)  # 2.6x slower limit, final approach aside


if __name__ == "__main__":
    unittest.main()
