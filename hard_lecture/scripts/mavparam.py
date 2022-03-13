#!/usr/bin/env python3

from mavros_msgs.srv import *
import rospy

if __name__ == "__main__":
    rospy.init_node('mavpram_setter')

    rospy.sleep(10.0)
    rospy.wait_for_service('/mavros/param/pull')
    pull_param = rospy.ServiceProxy('/mavros/param/pull', ParamPull)
    res0 = pull_param(False)
    if not res0.success:
        rospy.logwarn('load mavparam fail')

    rospy.sleep(5.0)
    rospy.wait_for_service('/mavros/param/set')
    set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
    value = ParamSet
    value.integer = 20
    value.real = 0.0
    res1 = set_param('SR0_POSITION',value)
    res2 = set_param('SR0_EXTRA1',value)
    res3 = set_param('SR0_RC_CHAN',value)
    if not (res1.success and res2.success and res3.success):
        rospy.logwarn('set mavparam fail')

    rospy.loginfo("SR0_xxx will be effective from the next startup")
