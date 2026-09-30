#!/usr/bin/env python3
# pose_listener.py - subscribes to /cube/pose and prints x, y and heading (yaw).

import math
import rospy
from geometry_msgs.msg import Pose


def on_pose(msg):
    # Convert the quaternion orientation into a single heading angle (yaw)
    q = msg.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    rospy.loginfo("cube: x=%.2f  y=%.2f  yaw=%.1f deg",
                  msg.position.x, msg.position.y, math.degrees(yaw))


def main():
    rospy.init_node("pose_listener")
    rospy.Subscriber("/cube/pose", Pose, on_pose)   # call on_pose for every message
    rospy.spin()                                    # keep the node alive


if __name__ == "__main__":
    main()
