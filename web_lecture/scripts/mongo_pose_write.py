#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == '__main__':
    rospy.init_node("mongo_pose_write")

    msg_store = MessageStoreProxy(database="srs", collection="pose1")
    p = Pose(Point(0, 1, 2), Quaternion(0, 0, 0, 1))

    try:
        p_id = msg_store.insert_named("named_pose", p)
        p_id = msg_store.insert(p)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e