#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import rospy

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Foo():

    def __init__(self):    
        self.cvb = CvBridge()
        rospy.Subscriber('raspicam_node/image/camup_8', CompressedImage, self.callback)
        rospy.init_node('foo', anonymous=True) 

    def callback(self, imgmsg):
        img = self.cvb.compressed_imgmsg_to_cv2(imgmsg)
        cv2.imshow('camup_8', img)
        cv2.waitKey(1)
       
if __name__ == '__main__':
    try:
        foo = Foo()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()






