#!/usr/bin/env python3
"""path_follower: drives along /planned_path and reports how the leg ended.

The path state and the controller are in path_tracker.py. This node only connects
them to ROS.

  in : /robot/pose, /planned_path, /nav/cancel, parameter /nav/max_lin
  out: /cmd_vel, /nav/leg_result

Time is simulation time (ADR-009): the node waits for Unity's first /clock message and
survives Unity pressing Play again, which restarts the clock at 0.
"""
import os
import sys
import time
from typing import Optional

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String

# catkin_make runs this file through a wrapper in devel/lib, so the folder with the
# modules next to it is not on sys.path by itself.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_common import (  # noqa: E402
    ABORTED,
    CANCELLED,
    SUCCEEDED,
    ClockWatch,
    finite_or,
    leg_result,
    wait_for_clock,
)
from path_tracker import (  # noqa: E402
    ARRIVED,
    DRIVING,
    STALE,
    WAYPOINT_TOL,
    PathTracker,
    check_flank_obstacle,
    check_frontal_obstacle,
    yaw_from_quaternion,
)

MAX_LIN = 0.26  # m/s, used while /nav/max_lin is not set: the Waffle Pi's top speed
RATE_HZ = 20.0
POSE_TIMEOUT = 0.5  # s of simulation time: with an older /robot/pose the robot stops
LATCH_WINDOW = 3.0  # s: right after start, a path older than this node is ignored


class PathFollower:
    def __init__(self) -> None:
        waypoint_tol = finite_or(
            rospy.get_param("~waypoint_tol", WAYPOINT_TOL), WAYPOINT_TOL, 0.0
        )
        final_tol = finite_or(
            rospy.get_param("~final_tol", 0.20), 0.20, 0.05
        )
        self.tracker = PathTracker(waypoint_tol, POSE_TIMEOUT, final_tol)
        self.clock = ClockWatch()
        self.latest: Optional[PoseStamped] = None  # replaced as one object
        self.latest_scan: Optional[LaserScan] = None
        self.blocked_since: Optional[float] = None
        self.last_replan_time: float = 0.0
        self.start_time = rospy.Time.now()
        self.start_wall = time.monotonic()

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.result_pub = rospy.Publisher("/nav/leg_result", String, queue_size=10)
        self.replan_pub = rospy.Publisher("/nav/replan", Empty, queue_size=1)
        rospy.Subscriber("/robot/pose", PoseStamped, self.on_pose)
        rospy.Subscriber("/planned_path", Path, self.on_path)
        rospy.Subscriber("/scan", LaserScan, self.on_scan)
        rospy.Subscriber("/nav/cancel", Empty, self.on_cancel)
        rospy.on_shutdown(self.stop)
        rospy.loginfo(
            "path_follower ready (waypoint_tol=%.2f m, final_tol=%.2f m). Waiting for a new path...",
            waypoint_tol,
            final_tol,
        )

    # ---------------- callbacks ----------------
    def on_pose(self, msg: PoseStamped) -> None:
        if msg.header.frame_id not in ("", "map"):
            rospy.logwarn_throttle(
                10.0, "Ignoring /robot/pose in frame '%s', not 'map'.", msg.header.frame_id
            )
            return
        self.latest = msg

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

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
        self.blocked_since = None
        rospy.loginfo("New path with %d waypoints.", len(points))

    def on_cancel(self, _msg: Empty) -> None:
        if self.tracker.cancel():
            self.stop()
            self.blocked_since = None
            self.report(ABORTED, CANCELLED)
            rospy.loginfo("Leg cancelled.")

    # ---------------- control loop ----------------
    def step(self) -> None:
        now = rospy.Time.now()
        if self.clock.jumped_back(now.to_sec()):
            self.on_clock_restart()
        latest = self.latest
        if latest is None or not self.tracker.active:
            return  # publish nothing: DiffDriveController's 0.5 s watchdog stops the wheels

        # Read at every step: the mission changes it with the category profile.
        max_lin = finite_or(rospy.get_param("/nav/max_lin", MAX_LIN), MAX_LIN, 0.0)
        age = (now - latest.header.stamp).to_sec()
        pos = latest.pose.position
        yaw = yaw_from_quaternion(latest.pose.orientation)
        cmd = self.tracker.step(pos.x, pos.y, yaw, age, max_lin)

        if cmd.status == DRIVING:
            v_lin = cmd.linear
            v_ang = cmd.angular

            # If driving forward (not pivoting on the spot), check frontal obstacles
            if v_lin > 0.0:
                close_obstacle = None
                far_obstacle = None
                if self.latest_scan is not None:
                    scan_age = (now - self.latest_scan.header.stamp).to_sec()
                    if 0.0 <= scan_age <= 1.0:
                        scan = self.latest_scan
                        # Zone 2: Close emergency stop obstacle (0.18m to 0.55m)
                        close_obstacle = check_frontal_obstacle(
                            scan.ranges,
                            scan.angle_min,
                            scan.angle_increment,
                            scan.range_min,
                            scan.range_max,
                            stop_dist=0.55,
                            corridor_half_width=0.25,
                            min_dist=0.18,
                        )
                        # Zone 1: Lookahead replan zone (0.55m to 2.80m)
                        if close_obstacle is None:
                            far_obstacle = check_frontal_obstacle(
                                scan.ranges,
                                scan.angle_min,
                                scan.angle_increment,
                                scan.range_min,
                                scan.range_max,
                                stop_dist=2.80,
                                corridor_half_width=0.32,
                                min_dist=0.55,
                            )

                if close_obstacle is not None:
                    # Check if robot is already at final goal standoff (e.g. shelf docking zone)
                    if cmd.is_last and 0.0 <= cmd.remaining_dist <= 0.32:
                        self.stop()
                        self.blocked_since = None
                        self.tracker.arrive()
                        self.report(SUCCEEDED)
                        rospy.loginfo(
                            "Arrived at destination standoff (dist=%.2f m, obstacle=%.2f m). Leg succeeded.",
                            cmd.remaining_dist,
                            close_obstacle,
                        )
                        return

                    if self.blocked_since is None:
                        self.blocked_since = now.to_sec()
                    v_lin = 0.0  # hold forward motion to avoid collision!
                    # Do NOT zero v_ang: keep v_ang = cmd.angular so robot can pivot away!
                    rospy.logwarn_throttle(
                        1.0,
                        "Close frontal obstacle detected at %.2f m! Holding forward velocity...",
                        close_obstacle,
                    )
                    blocked_dur = now.to_sec() - self.blocked_since
                    if blocked_dur >= 0.3 and (now.to_sec() - self.last_replan_time >= 1.0):
                        rospy.logwarn(
                            "Blocked at close range for %.1f s: requesting replan on /nav/replan...",
                            blocked_dur,
                        )
                        self.replan_pub.publish(Empty())
                        self.last_replan_time = now.to_sec()

                elif far_obstacle is not None:
                    self.blocked_since = None
                    # Do not trigger replan if we are approaching the final goal
                    if not (cmd.is_last and 0.0 <= cmd.remaining_dist <= 0.60):
                        if now.to_sec() - self.last_replan_time >= 1.0:
                            rospy.loginfo(
                                "Lookahead obstacle detected ahead at %.2f m! Requesting early replan...",
                                far_obstacle,
                            )
                            self.replan_pub.publish(Empty())
                            self.last_replan_time = now.to_sec()
                else:
                    self.blocked_since = None

                # Flank / side clearance guard: if bypassing an obstacle, ensure side doesn't scrape
                if close_obstacle is None and self.latest_scan is not None:
                    flank = check_flank_obstacle(
                        self.latest_scan.ranges,
                        self.latest_scan.angle_min,
                        self.latest_scan.angle_increment,
                        self.latest_scan.range_min,
                        self.latest_scan.range_max,
                        side_dist=0.28,
                    )
                    if flank is not None:
                        s_dist, s_yr = flank
                        # Obstacle closer than 28cm laterally: apply repulsive steering nudge away
                        bias_w = -0.35 if s_yr > 0 else 0.35
                        v_ang += bias_w
                        v_lin = min(v_lin, 0.12)  # slow down safely while clearing obstacle flank
                        rospy.logwarn_throttle(
                            1.0,
                            "Obstacle flank proximity (dist=%.2f m, y=%.2f m): nudging heading away",
                            s_dist,
                            s_yr,
                        )
            else:
                # Pivoting in place: clear blocked timer so robot can turn freely
                self.blocked_since = None

            twist = Twist()
            twist.linear.x = v_lin
            twist.angular.z = v_ang
            self.cmd_pub.publish(twist)
            rospy.loginfo_throttle(
                2.0,
                "driving: v=%.2f m/s w=%.2f rad/s at x=%.2f y=%.2f",
                v_lin,
                v_ang,
                pos.x,
                pos.y,
            )
        elif cmd.status == STALE:
            self.stop()
            self.blocked_since = None
            rospy.logwarn_throttle(2.0, "No fresh /robot/pose: the robot is stopped.")
        elif cmd.status == ARRIVED:
            self.stop()
            self.blocked_since = None
            self.report(SUCCEEDED)
            rospy.loginfo("Arrived at the goal: x=%.2f y=%.2f", pos.x, pos.y)

    def on_clock_restart(self) -> None:
        """Unity pressed Play again: the pose and the leg belong to the old run."""
        self.latest = None
        self.latest_scan = None
        self.blocked_since = None
        self.last_replan_time = 0.0
        rospy.logwarn("The simulation clock restarted: dropping the old pose and leg.")
        if self.tracker.cancel():
            self.stop()
            self.report(ABORTED, CANCELLED)

    # ---------------- output ----------------
    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def report(self, outcome: str, reason: str = "") -> None:
        self.result_pub.publish(String(data=leg_result(outcome, reason)))


def main() -> None:
    rospy.init_node("path_follower")
    ready = wait_for_clock(
        now=lambda: rospy.Time.now().to_sec(),
        sleep=time.sleep,
        is_shutdown=rospy.is_shutdown,
        on_wait=lambda s: rospy.loginfo("Waiting for Unity's /clock (%.0f s)...", s),
    )
    if not ready:
        return
    node = PathFollower()
    rate = rospy.Rate(RATE_HZ)
    while not rospy.is_shutdown():
        try:
            node.step()
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            rate = rospy.Rate(RATE_HZ)  # the next step() sees the jump and drops the leg
        except rospy.ROSInterruptException:
            break
        except Exception as exc:
            rospy.logerr_throttle(2.0, "Unexpected error in path_follower: %s", exc)


if __name__ == "__main__":
    main()
