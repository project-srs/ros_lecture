#!/usr/bin/env python3
import rospy

def timerCallback(event):
    rospy.loginfo("timer callback")

def timer():
    rospy.init_node('timer')
    rospy.sleep(2.)
    rospy.loginfo("sleep 2s")
    
    rospy.Timer(rospy.Duration(2.0), timerCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        timer()
    except rospy.ROSInterruptException: pass