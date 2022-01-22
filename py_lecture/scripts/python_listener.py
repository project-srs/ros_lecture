#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("recieved %s", data.data)
    now = rospy.Time.now()
    rospy.loginfo("now: %f", now.to_sec())
    
def listener():
    rospy.init_node('listener')
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()
        
if __name__ == '__main__':
    listener()