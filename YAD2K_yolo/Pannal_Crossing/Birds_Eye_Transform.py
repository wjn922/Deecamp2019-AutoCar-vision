#-*- coding: UTF-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from PIL import ImageDraw
from pylab import *
import math
import cv2 

def birds_eye_transform(pic_original,size_of_view=[-0.4,0.26,0.8,0.7],pix_per_m = 500 ):
    # Test Homography
    H = np.array([[ -7.03243543e+02,   1.46724803e+04,   6.04356954e+03],
     [  4.40193417e+04,   2.27653450e+04,   2.94329938e+02],
     [ -1.73453984e+00,   7.46733443e+01,   1.00000000e+00]])


    #pic_gray = cv2.cvtColor(pic_original, cv2.COLOR_RGB2GRAY)
    pic_gray = cv2.cvtColor(pic_original, cv2.COLOR_RGB2HLS)[:,:,1]

    Birds_eye = np.zeros([int(pix_per_m*size_of_view[2]), int(pix_per_m*size_of_view[3])])
    Dead_space = np.zeros([int(pix_per_m*size_of_view[2]), int(pix_per_m*size_of_view[3])]).astype('bool')

    X1 = np.ones([int(pix_per_m*size_of_view[2]),int(pix_per_m*size_of_view[3]),3])
    i = np.arange(int(pix_per_m*size_of_view[2]))*1.0/pix_per_m+size_of_view[0]
    j = np.arange(int(pix_per_m*size_of_view[3]))*1.0/pix_per_m+size_of_view[1]
    I,J = np.meshgrid(i,j)
    X1[:,:,0]=I.T
    X1[:,:,1]=J.T

    U = np.ones([int(pix_per_m*size_of_view[2]),int(pix_per_m*size_of_view[3]),3])

    U[:,:,0] = np.round([np.sum(H[0,:] *X1[:,:],axis=2)] /np.sum(H[2,:] *X1[:,:],axis=2),decimals=0)
    U[:,:,1] = np.round([np.sum(H[1,:] *X1[:,:],axis=2)] /np.sum(H[2,:] *X1[:,:],axis=2),decimals=0)
    U=U.astype('int')

    Dead_space= ~(((0<=U[:,:,0]) & (U[:,:,0]<=pic_gray.shape[0]-1)) & ((0<=U[:,:,1]) & (U[:,:,1]<=pic_gray.shape[1]-1)))
    
    for i in range(int(pix_per_m*size_of_view[2])):
        for j in range(int(pix_per_m*size_of_view[3])):
            if ~Dead_space[i,j]:
                Birds_eye[i,j] = pic_gray[U[i,j,0],U[i,j,1]]
          
    return Birds_eye,Dead_space
