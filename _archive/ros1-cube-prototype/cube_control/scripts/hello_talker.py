#!/usr/bin/env python3
# hello_talker.py - the simplest possible ROS node: it publishes a String twice a second.

import rospy
from std_msgs.msg import String


def main():
    # 1. Register this program with roscore under the name "hello_talker"
    rospy.init_node("hello_talker")

    # 2. Announce that we publish String messages on the topic "ros_hello"
    pub = rospy.Publisher("ros_hello", String, queue_size=10)

    # 3. A Rate object lets us loop at a fixed frequency (2 Hz here)
    rate = rospy.Rate(2)

    count = 0
    while not rospy.is_shutdown():      # runs until you press Ctrl+C
        msg = String()
        msg.data = "hello from ROS #%d" % count
        pub.publish(msg)                # send it
        rospy.loginfo("published: %s", msg.data)
        count += 1
        rate.sleep()                    # wait so the loop runs at 2 Hz


if __name__ == "__main__":
    main()
