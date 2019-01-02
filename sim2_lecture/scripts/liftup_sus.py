#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('liftup_sus')
    pub0 = rospy.Publisher('sus0', Float64, queue_size=10)
    pub1 = rospy.Publisher('sus1', Float64, queue_size=10)
    pub2 = rospy.Publisher('sus2', Float64, queue_size=10)

    rospy.sleep(5.0)
    HZ=50
    r = rospy.Rate(HZ)
    i = 0
    liftup_step=100
    while not rospy.is_shutdown():
        command_pos= (liftup_step-i) * (0.02/liftup_step)
        pub0.publish(command_pos)
        pub1.publish(command_pos)
        pub2.publish(command_pos)
        #print(command_pos)
        if command_pos<=0:
            break
        i+=1
        r.sleep()
    rospy.loginfo("Liftup Done")


