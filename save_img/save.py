#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import cv2
import os
import time
import rospy

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class Foo():

    
    def __init__(self):  
    	self.count = 0
    	self.interval = 4
    	self.save_path = "/home/pro/Documents/move_pic/seventh"
  	self.path_init()
        self.cvb = CvBridge()
        rospy.Subscriber('raspicam_node/image/camdown_8', CompressedImage, self.callback)
        rospy.init_node('foo', anonymous=True)

    def callback(self, imgmsg):
        img = self.cvb.compressed_imgmsg_to_cv2(imgmsg)
	if self.count % self.interval == 0:
            path=self.save_path +'/'+ str(self.count)+".jpg"
            print path
            cv2.imwrite(path,img)
        cv2.imshow('img', img)
        cv2.waitKey(1)
        #if cv2.waitKey(1) & 0xff == ord("s"):
           # path='/home/liu/Desktop/'+time.strftime("%H:%M:%S", time.localtime())+".jpg"
            #print path
            #cv2.imwrite( path,img)
	self.count += 1 # = (self.count+1) % self.interval
        #print(self.count)
    def path_init(self):
        if not os.path.exists(self.save_path):
            os.mkdir(self.save_path)
       
if __name__ == '__main__':
    try:
        foo = Foo()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()






