#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TwistPublisher:
    def __init__(self):
        rospy.init_node('listener')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/web/joy", Joy, self.callback)
        rospy.spin()

    def callback(self, data):
        rospy.loginfo("recieved %f", data.axes[0])
        cmd = Twist()
        cmd.linear.x = 0.5 * data.axes[1]
        cmd.angular.z = -1.0 * data.axes[0]
        self.pub.publish(cmd)


if __name__ == '__main__':
    twist_publisher = TwistPublisher()