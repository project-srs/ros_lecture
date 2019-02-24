#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridgeDemo:
    def __init__(self):
        self.node_name = "cv_bridge_demo_compressed"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("input_image", CompressedImage, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher('output_image', Image, queue_size=1)

    def image_callback(self, ros_image_compressed):
        try:
            np_arr = np.fromstring(ros_image_compressed.data, np.uint8)
            input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError, e:
            print e
        
        output_image = self.process_image(input_image)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "mono8"))
        
        cv2.imshow(self.node_name, output_image)   
        cv2.waitKey(1)
                          
    def process_image(self, frame):
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        grey = cv2.blur(grey, (7, 7))
        edges = cv2.Canny(grey, 15.0, 30.0)
        return edges
        
    def cleanup(self):
        cv2.destroyAllWindows()   
    
if __name__ == '__main__':
    cvBridgeDemo()
    rospy.spin()