#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import Image
from web_lecture.msg import StringStamp
from web_lecture.srv import StringStampList, StringStampListResponse
from mongodb_store.message_store import MessageStoreProxy
from cv_bridge import CvBridge, CvBridgeError
import base64
import cv2

class MongoImage:
    def __init__(self):
        rospy.Service('shot', Empty, self.shotCallback)
        rospy.Service('delete', Empty, self.deleteCallback)
        rospy.Service('request', StringStampList, self.requestCallback)
        rospy.Subscriber("/image_raw", Image, self.imageCallback)
        self.last_image = Image()
        self.scale = 0.5
        self.bridge = CvBridge()
        self.last_base64 = None
        self.msg_store = MessageStoreProxy(database="srs", collection="image_stamp")

    def imageCallback(self, msg):
        self.last_image = msg

    def shotCallback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(self.last_image, "bgr8")
        height = frame.shape[0]
        width = frame.shape[1]
        frame2 = cv2.resize(frame , (int(width*self.scale), int(height*self.scale)))
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        result, frame3 = cv2.imencode(".jpg", frame2, encode_param)
        mongo_data = StringStamp()
        mongo_data.stamp = rospy.get_rostime()
        mongo_data.data = base64.b64encode(frame3)
        try:
            p_id = self.msg_store.insert(mongo_data)            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return EmptyResponse()

    def deleteCallback(self, msg):
        list = self.msg_store.query(StringStamp._type)
        for item in list:
            self.msg_store.delete(str(item[1]["_id"]))
        return EmptyResponse()

    def requestCallback(self, msg):
        mongo_list = self.msg_store.query(StringStamp._type)
        output = []
        try:
            for item in mongo_list:
                output.append(item[0])
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
        return StringStampListResponse(output)

if __name__ == '__main__':
    rospy.init_node('mongo_image')
    mongo_image = MongoImage()
    rospy.spin()
