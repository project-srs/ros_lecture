#!/usr/bin/env python3
import rospy
import rosparam
from std_msgs.msg import String

def talker():
    rospy.init_node('talker')
    word = rospy.get_param("~content", "default")
    pub = rospy.Publisher('chatter', String, queue_size=10)

    r = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        str = "send: %s" % word
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass