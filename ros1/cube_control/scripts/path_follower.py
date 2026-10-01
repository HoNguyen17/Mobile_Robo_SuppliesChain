#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path

K_LIN = 0.5        # proportional gain, forward speed (final approach only)
K_ANG = 1.5        # proportional gain, turning
MAX_LIN = 0.5      # m/s
MAX_ANG = 1.0      # rad/s
TURN_FIRST = 0.5   # rad: if the heading error is bigger, turn in place first
TOLERANCE = 0.1   # m: arrival tolerance at the FINAL waypoint
RATE_HZ = 20.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PathFollower:
    def __init__(self):
        self.waypoint_tol = rospy.get_param('~waypoint_tol', 0.3)  # m, intermediate corners
        self.pose = None
        self.waypoints = []
        self.index = 0
        self.start_time = rospy.Time.now()

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/cube/pose', Pose, self.on_pose)
        rospy.Subscriber('/planned_path', Path, self.on_path)
        rospy.on_shutdown(self.stop)
        rospy.loginfo('path_follower ready (waypoint_tol=%.2f m). Waiting for a new path...',
                      self.waypoint_tol)

    def on_pose(self, msg):
        self.pose = msg

    def on_path(self, msg):
        # /planned_path is latched: a path planned before this node started arrives immediately.
        if msg.header.stamp < self.start_time:
            rospy.loginfo('Ignoring an old (latched) path from before this node started.')
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not pts:
            return
        self.waypoints = pts
        # waypoint 0 is where the robot was when it planned, so head for waypoint 1.
        self.index = 1 if len(pts) > 1 else 0
        rospy.loginfo('New path with %d waypoints, heading for waypoint %d.',
                      len(pts), self.index)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def step(self):
        if self.pose is None or self.index >= len(self.waypoints):
            return  # nothing to do: publish nothing, Unity's 0.5 s watchdog keeps the cube still

        x, y = self.pose.position.x, self.pose.position.y
        yaw = yaw_from_quaternion(self.pose.orientation)
        wx, wy = self.waypoints[self.index]
        dx, dy = wx - x, wy - y
        dist = math.hypot(dx, dy)
        last = (self.index == len(self.waypoints) - 1)

        if last:
            if dist < TOLERANCE:
                self.stop()
                self.waypoints = []
                rospy.loginfo('Arrived at the goal: x=%.2f y=%.2f', x, y)
                return
        elif dist < self.waypoint_tol:
            rospy.loginfo('Reached waypoint %d of %d', self.index, len(self.waypoints) - 1)
            self.index += 1
            return  # the next loop chases the new waypoint

        heading_err = normalize(math.atan2(dy, dx) - yaw)

        cmd = Twist()
        cmd.angular.z = clamp(K_ANG * heading_err, -MAX_ANG, MAX_ANG)
        if abs(heading_err) > TURN_FIRST:
            cmd.linear.x = 0.0                                # pivot on the spot
        elif last:
            cmd.linear.x = clamp(K_LIN * dist, 0.0, MAX_LIN)  # slow down towards the goal
        else:
            cmd.linear.x = MAX_LIN                            # keep flowing through corners
        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(2.0, 'waypoint %d/%d: dist=%.2f m, heading error=%.2f rad',
                               self.index, len(self.waypoints) - 1, dist, heading_err)


if __name__ == '__main__':
    rospy.init_node('path_follower')
    node = PathFollower()
    rate = rospy.Rate(RATE_HZ)
    try:
        while not rospy.is_shutdown():
            node.step()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
