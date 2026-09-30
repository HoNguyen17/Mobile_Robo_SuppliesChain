#!/usr/bin/env python3
# go_to_goal.py - closed-loop controller: reads /cube/pose, publishes /cmd_vel
# until the cube reaches a goal point (x, y) given in the ROS frame.
#
# Parameters (private, set on the command line with _name:=value):
#   _goal_x, _goal_y : goal in metres. If omitted, goal = start position + (2, 2).

import math
import rospy
from geometry_msgs.msg import Pose, Twist

K_LIN = 0.5          # linear gain      (m/s per metre of distance)
K_ANG = 1.5          # angular gain     (rad/s per radian of heading error)
MAX_LIN = 0.5        # m/s
MAX_ANG = 1.0        # rad/s
TURN_FIRST = 0.5     # rad: if heading error is larger, rotate in place
TOLERANCE = 0.1      # m: goal reached inside this distance


def wrap_angle(a):
    """Wrap an angle into [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GoToGoal:
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal = None
        self.reached = False
        if rospy.has_param("~goal_x") and rospy.has_param("~goal_y"):
            self.goal = (rospy.get_param("~goal_x"), rospy.get_param("~goal_y"))
        rospy.Subscriber("/cube/pose", Pose, self.on_pose)

    def on_pose(self, msg):
        x, y = msg.position.x, msg.position.y
        q = msg.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if self.goal is None:                       # no goal given: 2 m diagonal from here
            self.goal = (x + 2.0, y + 2.0)
            rospy.loginfo("goal set to (%.2f, %.2f)", *self.goal)

        dx = self.goal[0] - x
        dy = self.goal[1] - y
        dist = math.hypot(dx, dy)
        heading_err = wrap_angle(math.atan2(dy, dx) - yaw)

        cmd = Twist()
        if dist < TOLERANCE:
            if not self.reached:
                rospy.loginfo("goal reached (distance %.2f m)", dist)
                self.reached = True
        else:
            self.reached = False
            cmd.angular.z = clamp(K_ANG * heading_err, -MAX_ANG, MAX_ANG)
            if abs(heading_err) < TURN_FIRST:
                cmd.linear.x = clamp(K_LIN * dist, 0.0, MAX_LIN)
            rospy.loginfo_throttle(0.5, "dist=%.2f m  heading_err=%.1f deg",
                                   dist, math.degrees(heading_err))
        self.pub.publish(cmd)


def main():
    rospy.init_node("go_to_goal")
    GoToGoal()
    rospy.spin()


if __name__ == "__main__":
    main()
