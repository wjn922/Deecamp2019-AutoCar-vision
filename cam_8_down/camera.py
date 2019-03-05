#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import rospy

from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Foo():

    def __init__(self): 
        rospy.init_node('foo', anonymous=True)    
	self.img_pub = rospy.Publisher('Detection/image', Image, queue_size=2)
	
        self.cvb = CvBridge()
        rospy.Subscriber('raspicam_node/image/camdown_8', CompressedImage, self.callback)
 

    def callback(self, imgmsg):
        img = self.cvb.compressed_imgmsg_to_cv2(imgmsg)

	msg = self.cvb.cv2_to_imgmsg(img, encoding="bgr8")
        self.img_pub.publish(msg)

        #cv2.imshow('camdown_8', img)
        #cv2.waitKey(1)
       
if __name__ == '__main__':
    try:
        foo = Foo()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()






























