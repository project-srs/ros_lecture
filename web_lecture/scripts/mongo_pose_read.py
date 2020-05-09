#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
import StringIO

if __name__ == '__main__':
    rospy.init_node("mongo_pose_write")

    msg_store = MessageStoreProxy(database="srs", collection="pose1")

    try:
        for item in msg_store.query(Pose._type):
            print item[0].position

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e