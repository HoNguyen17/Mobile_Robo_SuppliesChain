#!/usr/bin/env python3
"""astar_planner: plans a path when a goal arrives and publishes it on /planned_path.

The planning itself is in grid_planner.py. This node only connects it to ROS.

  in : /map, /cube/pose, /move_base_simple/goal, parameter /nav/inflation_radius
  out: /planned_path (latched), /nav/leg_result (only when there is no path)
"""
import math
import os
import sys
import time
from typing import Optional

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String

# catkin_make runs this file through a wrapper in devel/lib, so the folder with the
# modules next to it is not on sys.path by itself.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grid_planner import Grid, NoPath, ascii_map, plan  # noqa: E402
from nav_common import ABORTED, NO_PATH, finite_or, leg_result  # noqa: E402

SNAP_DISTANCE = 6.0  # m, default of ~snap_distance
MAX_PICTURE_HEIGHT = 100  # the ASCII picture is skipped for taller grids


class AStarPlanner:
    def __init__(self) -> None:
        self.snap_distance = finite_or(
            rospy.get_param("~snap_distance", SNAP_DISTANCE), SNAP_DISTANCE, 0.0
        )
        self.print_map = rospy.get_param("~print_map", True)  # draw the path as text
        self.grid: Optional[Grid] = None
        self.pose: Optional[Pose] = None

        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher("/nav/leg_result", String, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self.on_map)
        rospy.Subscriber("/cube/pose", Pose, self.on_pose)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_goal)
        rospy.loginfo("astar_planner ready: send a goal to /move_base_simple/goal")

    # ---------------- callbacks ----------------
    def on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        try:
            self.grid = Grid(
                msg.data,
                info.width,
                info.height,
                info.resolution,
                info.origin.position.x,
                info.origin.position.y,
            )
        except ValueError as exc:  # keep the last good map
            rospy.logwarn_throttle(10.0, "Ignoring a bad /map: %s", exc)

    def on_pose(self, msg: Pose) -> None:
        self.pose = msg

    def on_goal(self, msg: PoseStamped) -> None:
        grid, pose = self.grid, self.pose
        if grid is None or pose is None:
            return self.abort("no /map or no /cube/pose received yet")
        frame = msg.header.frame_id
        if frame not in ("", "map"):
            return self.abort("the goal is in frame '%s', not 'map'" % frame)

        start = (pose.position.x, pose.position.y)
        goal = (msg.pose.position.x, msg.pose.position.y)
        # Read at every plan: the mission changes it with the category profile.
        radius = finite_or(rospy.get_param("/nav/inflation_radius", 0.0), 0.0, 0.0)

        t0 = time.time()
        try:
            corners = plan(grid, start, goal, self.snap_distance, radius)
        except NoPath as exc:
            return self.abort(str(exc))

        self.publish_path(corners)
        length = sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(corners, corners[1:])
        )
        rospy.loginfo(
            "Planned: %d corners, %.1f m, %.0f ms, inflation %.2f m",
            len(corners),
            length,
            (time.time() - t0) * 1000.0,
            radius,
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
        if self.print_map and grid.height <= MAX_PICTURE_HEIGHT:
            rospy.loginfo("\n%s", ascii_map(grid, corners))

    # ---------------- output ----------------
    def abort(self, why: str) -> None:
        rospy.logwarn("No path: %s", why)
        self.result_pub.publish(String(data=leg_result(ABORTED, NO_PATH)))

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
        self.path_pub.publish(path)


if __name__ == "__main__":
    rospy.init_node("astar_planner")
    AStarPlanner()
    rospy.spin()
