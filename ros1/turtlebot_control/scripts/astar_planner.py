#!/usr/bin/env python3
"""astar_planner: plans a path when a goal arrives and publishes it on /planned_path.

The planning itself is in grid_planner.py. This node only connects it to ROS.

  in : /map, /robot/pose, /move_base_simple/goal, parameter /nav/inflation_radius
  out: /planned_path (latched), /nav/leg_result (only when there is no path)

Everything is in the ROS "map" frame, in robot metres (ADR-010, ADR-012).
"""
import math
import os
import sys
import time
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String

# catkin_make runs this file through a wrapper in devel/lib, so the folder with the
# modules next to it is not on sys.path by itself.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from grid_planner import (  # noqa: E402
    Grid,
    InflatedMaps,
    NoPath,
    ascii_map,
    plan_inflated,
)
from nav_common import ABORTED, NO_PATH, finite_or, leg_result  # noqa: E402

SNAP_DISTANCE = 2.0  # m, default of ~snap_distance
# The map is raw (ADR-012), so the planner must inflate it. Used while /nav/inflation_radius is
# not set: the footprint reaches 0.257 m from the wheel axis centre, so this leaves about 9 cm.
DEFAULT_INFLATION = 0.35  # m
# s of simulation time: planning from an older pose would start in the wrong place
POSE_MAX_AGE = 1.0
MAX_PICTURE_HEIGHT = 100  # the ASCII picture is skipped for taller grids


class AStarPlanner:
    def __init__(self) -> None:
        self.snap_distance = finite_or(
            rospy.get_param("~snap_distance", SNAP_DISTANCE), SNAP_DISTANCE, 0.0
        )
        self.print_map = rospy.get_param("~print_map", False)  # draw the path as text
        self.maps: Optional[InflatedMaps] = None  # replaced as one object
        self.pose: Optional[PoseStamped] = None

        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher("/nav/leg_result", String, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self.on_map)
        rospy.Subscriber("/robot/pose", PoseStamped, self.on_pose)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_goal)
        rospy.loginfo("astar_planner ready: send a goal to /move_base_simple/goal")

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

    def on_goal(self, msg: PoseStamped) -> None:
        maps, pose = self.maps, self.pose
        if maps is None or pose is None:
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

        t0 = time.time()
        try:
            corners = plan_inflated(maps.get(radius), start, goal, self.snap_distance)
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
        if self.print_map and maps.grid.height <= MAX_PICTURE_HEIGHT:
            rospy.loginfo("\n%s", ascii_map(maps.grid, corners))

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


def same_map(a: Grid, b: Grid) -> bool:
    """True when two /map messages describe the same grid."""
    return (
        (a.width, a.height, a.res, a.ox, a.oy) == (b.width, b.height, b.res, b.ox, b.oy)
        and tuple(a.data) == tuple(b.data)
    )


if __name__ == "__main__":
    rospy.init_node("astar_planner")
    AStarPlanner()
    rospy.spin()
