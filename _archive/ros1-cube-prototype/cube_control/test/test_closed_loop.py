"""Closed-loop check without ROS: plan on a map, follow the plan with the real
follower logic, and move a simulated robot the way Unity does."""
import math
import unittest

import support  # noqa: F401  (puts scripts/ on sys.path)
from grid_planner import cell_index, plan
from path_tracker import ARRIVED, FINAL_TOL, PathTracker
from support import make_grid

DT = 0.05  # the follower runs at 20 Hz


class Robot:
    """Kinematic unicycle. Like Unity's CubeCmdVelSubscriber: turn first, then drive."""

    def __init__(self, x, y, yaw):
        self.x, self.y, self.yaw = x, y, yaw

    def apply(self, cmd, dt):
        self.yaw += cmd.angular * dt
        self.x += cmd.linear * math.cos(self.yaw) * dt
        self.y += cmd.linear * math.sin(self.yaw) * dt


def wall_with_gap(width=20, height=12, wall_i=10, gap=(9, 10, 11)):
    """A vertical wall at column wall_i with an opening in rows `gap` (j values)."""
    rows = []
    for j in range(height - 1, -1, -1):
        rows.append(
            "".join("#" if i == wall_i and j not in gap else "." for i in range(width))
        )
    return make_grid(rows)


def drive(grid, start, goal, yaw=0.0, inflation_radius=0.0, max_lin=0.5, limit_s=300.0):
    """Plan, then follow the plan until the follower reports arrival.

    Returns (robot, trail of (x, y), seconds). Fails if it does not arrive in time.
    """
    corners = plan(grid, start, goal, 3.0, inflation_radius)
    tracker = PathTracker()
    tracker.set_path(corners)
    robot = Robot(start[0], start[1], yaw)
    trail = [(robot.x, robot.y)]
    for n in range(int(limit_s / DT)):
        cmd = tracker.step(robot.x, robot.y, robot.yaw, 0.0, max_lin)
        if cmd.status == ARRIVED:
            return robot, trail, n * DT
        robot.apply(cmd, DT)
        trail.append((robot.x, robot.y))
    raise AssertionError("did not arrive within %.0f s, stopped at (%.2f, %.2f)" % (
        limit_s, robot.x, robot.y))


class ClosedLoopTest(unittest.TestCase):
    def test_drives_across_an_open_field_to_the_goal(self):
        grid = make_grid(["." * 20] * 20)
        goal = (17.5, 15.5)
        robot, _, seconds = drive(grid, (2.5, 2.5), goal)
        self.assertLess(math.hypot(robot.x - goal[0], robot.y - goal[1]), FINAL_TOL)
        self.assertLess(seconds, 60.0)  # about 20 m at 0.5 m/s

    def test_drives_through_the_gap_in_a_wall_without_touching_it(self):
        grid = wall_with_gap()
        goal = (17.5, 2.5)
        robot, trail, _ = drive(grid, (2.5, 2.5), goal, inflation_radius=1.0)
        self.assertLess(math.hypot(robot.x - goal[0], robot.y - goal[1]), FINAL_TOL)
        self.assertGreater(max(y for _, y in trail), 9.0)  # it went up to the gap
        for x, y in trail:
            self.assertEqual(grid.data[cell_index(grid, x, y)], 0, (x, y))

    def test_a_robot_facing_away_turns_round_and_still_arrives(self):
        grid = wall_with_gap()
        goal = (17.5, 2.5)
        robot, _, _ = drive(grid, (2.5, 2.5), goal, yaw=math.pi, inflation_radius=1.0)
        self.assertLess(math.hypot(robot.x - goal[0], robot.y - goal[1]), FINAL_TOL)

    def test_a_slower_speed_limit_is_obeyed_all_the_way(self):
        grid = make_grid(["." * 20] * 4)
        fast = drive(grid, (1.5, 1.5), (18.5, 1.5), max_lin=0.5)[2]
        slow = drive(grid, (1.5, 1.5), (18.5, 1.5), max_lin=0.1)[2]
        self.assertGreater(slow, 4.0 * fast)  # 5x slower limit, final approach aside


if __name__ == "__main__":
    unittest.main()
