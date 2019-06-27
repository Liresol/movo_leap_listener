#!/usr/bin/env python
import sys
import rospy
from listener import LeapListener
if __name__ == '__main__':
    #ll = LeapListener(sys.argv[1])
    ll = LeapListener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("E");
