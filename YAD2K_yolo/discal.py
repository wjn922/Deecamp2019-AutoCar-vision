# -*- coding: utf-8 -*-
"""
Created on Fri Jan 25 04:23:01 2019

@author: 22591
"""
from __future__ import division
import numpy as np
''' find cameras' pixel focal lngth '''
    # the pixel focal lngth (unit:pixel)
    
def find_cam(type):
    # 1:7up
    if type == "7up":
        return 606
    
    # 2:7down
    elif type == "7down":
        return 606
    
    # 3:8up
    elif type == "8up":
        return 606
    
    # 4:8down
    elif type == "8down":
        return 606
    
    else:
        return -1


''' the real world length (unit:mm) '''
def find_obj(type):
    #delta  x
    
    #1:stop
    if type == "stop":
        #hight or width
        return 100
    
    #2:slow
    elif type == "slow":
        #width=48
        return 48
    
    #3:work
    elif type == "obstacle":
        #width
        return 210
    
    #4:Grid
    elif type == "grid":
        #width
        return 24
    
    #5:Car
    elif type == "car":
        #width
        return 180
    
    else:
        return -1


''' left_x, right_x, camera type, object type '''
def cal_dis(Left, Right, obj, cam="7down"):
    Delta = np.abs(Left - Right)
    CamFocalLength = find_cam(cam)
    ObjRealLength = find_obj(obj)
    if (CamFocalLength != -1 and ObjRealLength != -1):
        Dis = CamFocalLength * ObjRealLength / Delta
        return Dis
    else:
        return -1

