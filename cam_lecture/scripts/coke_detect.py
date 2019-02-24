#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridgeDemo:
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("input_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        input_image = np.array(frame, dtype=np.uint8)
        
        rects = self.process_image(input_image, True)
                
        print(rects)
        cv2.waitKey(1)
                          
    def process_image(self, image, debug=False):
        # hsv filter
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([155,50,50])
        upper_blue = np.array([180,255,255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        if debug:
            display = cv2.bitwise_and(image, image, mask= mask)
            cv2.imshow("hsv filter", display)   
        
        # morphology processing
        kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
        mask = cv2.dilate(mask,kernel,iterations = 1)
        mask = cv2.erode(mask,kernel,iterations = 1)
        
        if debug:
            display = cv2.bitwise_and(image, image, mask= mask)
            cv2.imshow("morphology processing", display)   
        
        # make contour
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if debug:
            display = np.zeros(mask.shape, dtype=np.uint8)
            for c in contours:
                for elem in c:
                    display[elem[0,1],elem[0,0]]=255
            cv2.imshow("make contours", display)   
        
        # make region
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(rect)
            
        if debug:
            display=image.copy()
            for rect in rects:
                cv2.rectangle(display, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (0, 255, 0), thickness=5)
            cv2.imshow("make region", display)   

        return rects

    def cleanup(self):
        cv2.destroyAllWindows()   
    
if __name__ == '__main__':
    cvBridgeDemo()
    rospy.spin()