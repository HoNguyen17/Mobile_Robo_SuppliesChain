#!/usr/bin/env python3
"""path_follower: drives along /planned_path and reports how the leg ended.

The path state and the controller are in path_tracker.py. This node only connects
them to ROS.

  in : /cube/pose, /planned_path, /nav/cancel, parameter /nav/max_lin
  out: /cmd_vel, /nav/leg_result
"""
import os
import sys
import time
from typing import Optional, Tuple

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String

# catkin_make runs this file through a wrapper in devel/lib, so the folder with the
# modules next to it is not on sys.path by itself.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_common import (  # noqa: E402
    ABORTED,
    CANCELLED,
    SUCCEEDED,
    finite_or,
    leg_result,
)
from path_tracker import (  # noqa: E402
    ARRIVED,
    DRIVING,
    STALE,
    PathTracker,
    yaw_from_quaternion,
)

MAX_LIN = 0.5  # m/s, used while /nav/max_lin is not set
RATE_HZ = 20.0
POSE_TIMEOUT = 0.5  # s: with an older /cube/pose the robot stops
LATCH_WINDOW = 3.0  # s: right after start, a path older than this node is ignored


class PathFollower:
    def __init__(self) -> None:
        waypoint_tol = finite_or(rospy.get_param("~waypoint_tol", 0.3), 0.3, 0.0)
        self.tracker = PathTracker(waypoint_tol, POSE_TIMEOUT)
        self.latest: Optional[Tuple[Pose, rospy.Time]] = None  # replaced as one object
        self.start_time = rospy.Time.now()
        self.start_wall = time.monotonic()

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.result_pub = rospy.Publisher("/nav/leg_result", String, queue_size=10)
        rospy.Subscriber("/cube/pose", Pose, self.on_pose)
        rospy.Subscriber("/planned_path", Path, self.on_path)
        rospy.Subscriber("/nav/cancel", Empty, self.on_cancel)
        rospy.on_shutdown(self.stop)
        rospy.loginfo(
            "path_follower ready (waypoint_tol=%.2f m). Waiting for a new path...",
            waypoint_tol,
        )

    # ---------------- callbacks ----------------
    def on_pose(self, msg: Pose) -> None:
        self.latest = (msg, rospy.Time.now())

    def on_path(self, msg: Path) -> None:
        # /planned_path is latched, so a path planned before this node started
        # arrives right after it connects. Drop that one. The check only runs in the
        # first seconds: if Unity restarts its clock, new paths must not look old.
        just_started = time.monotonic() - self.start_wall < LATCH_WINDOW
        if just_started and msg.header.stamp < self.start_time:
            rospy.loginfo("Ignoring an old (latched) path from before this start.")
            return
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not points:
            return
        self.tracker.set_path(points)
        rospy.loginfo("New path with %d waypoints.", len(points))

    def on_cancel(self, _msg: Empty) -> None:
        if self.tracker.cancel():
            self.stop()
            self.report(ABORTED, CANCELLED)
            rospy.loginfo("Leg cancelled.")

    # ---------------- control loop ----------------
    def step(self) -> None:
        latest = self.latest
        if latest is None or not self.tracker.active:
            return  # publish nothing: Unity's 0.5 s watchdog keeps the cube still
        pose, received = latest
        # Read at every step: the mission changes it with the category profile.
        max_lin = finite_or(rospy.get_param("/nav/max_lin", MAX_LIN), MAX_LIN, 0.0)
        age = (rospy.Time.now() - received).to_sec()
        pos = pose.position
        cmd = self.tracker.step(
            pos.x, pos.y, yaw_from_quaternion(pose.orientation), age, max_lin
        )

        if cmd.status == DRIVING:
            twist = Twist()
            twist.linear.x = cmd.linear
            twist.angular.z = cmd.angular
            self.cmd_pub.publish(twist)
            rospy.loginfo_throttle(
                2.0,
                "driving: v=%.2f m/s w=%.2f rad/s at x=%.2f y=%.2f",
                cmd.linear,
                cmd.angular,
                pos.x,
                pos.y,
            )
        elif cmd.status == STALE:
            self.stop()
            rospy.logwarn_throttle(2.0, "No fresh /cube/pose: the robot is stopped.")
        elif cmd.status == ARRIVED:
            self.stop()
            self.report(SUCCEEDED)
            rospy.loginfo("Arrived at the goal: x=%.2f y=%.2f", pos.x, pos.y)

    # ---------------- output ----------------
    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def report(self, outcome: str, reason: str = "") -> None:
        self.result_pub.publish(String(data=leg_result(outcome, reason)))


if __name__ == "__main__":
    rospy.init_node("path_follower")
    node = PathFollower()
    rate = rospy.Rate(RATE_HZ)
    try:
        while not rospy.is_shutdown():
            node.step()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
