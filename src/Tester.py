#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, String

if __name__ == "__main__":

    rospy.init_node("Tester")
    rate = rospy.Rate(1)

    pubPoints = rospy.Publisher("/points", Int32, queue_size = 32)
    cnt = 0

    while not rospy.is_shutdown():

        pubPoints.publish(cnt)
        cnt += 1

        rate.sleep()
