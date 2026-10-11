#!/usr/bin/env python3
# cmd_vel_publisher.py - drives a square by publishing geometry_msgs/Twist on /cmd_vel.

import math
import rospy
from geometry_msgs.msg import Twist


def publish_for(pub, rate, linear, angular, duration):
    """Publish the same command at 10 Hz for `duration` seconds."""
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    end = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end and not rospy.is_shutdown():
        pub.publish(cmd)
        rate.sleep()


def main():
    rospy.init_node("cmd_vel_publisher")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)

    speed = 0.5                               # m/s
    turn_rate = 0.5                           # rad/s
    forward_time = 4.0                        # s  -> 2 m per side
    turn_time = (math.pi / 2) / turn_rate     # s  -> exactly 90 degrees

    rospy.sleep(1.0)                          # give the connection a moment
    for side in range(4):
        rospy.loginfo("side %d: forward", side + 1)
        publish_for(pub, rate, speed, 0.0, forward_time)
        rospy.loginfo("side %d: turn left", side + 1)
        publish_for(pub, rate, 0.0, turn_rate, turn_time)

    publish_for(pub, rate, 0.0, 0.0, 0.5)     # explicit stop
    rospy.loginfo("done")


if __name__ == "__main__":
    main()
