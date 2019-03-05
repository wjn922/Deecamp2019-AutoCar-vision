#-*- coding: UTF-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import skimage
from skimage import morphology
import sklearn
from PIL import Image
from PIL import ImageDraw
from pylab import *
import math
import cv2 



# 定义旋转rotate函数
def rotate(image, angle, center=None, scale=1.0):
    # 获取图像尺寸
    (h, w) = image.shape[:2]
    # 若未指定旋转中心，则将图像中心设为旋转中心
    if center is None:
        center = (w / 2, h / 2)
    # 执行旋转
    M = cv2.getRotationMatrix2D(center, angle, scale)
    rotated = cv2.warpAffine(image, M, (w, h))
    # 返回旋转后的图像
    return rotated

# 定义平移translate函数
def translate(image, x, y):
    # 定义平移矩阵
    M = np.float32([[1, 0, x], [0, 1, y]])
    shifted = cv2.warpAffine(image, M, (image.shape[1], image.shape[0]))
    # 返回转换后的图像
    return shifted

# 定义缩放resize函数
def resize_img(image, width=None, height=None, inter=cv2.INTER_AREA):
    # 初始化缩放比例，并获取图像尺寸
    dim = None
    (h, w) = image.shape[:2]
    # 如果宽度和高度均为0，则返回原图
    if width is None and height is None:
        return image
    # 宽度是0
    if width is None:
        # 则根据高度计算缩放比例
        r = height / float(h)
        dim = (int(w * r), height)
    # 如果高度为0
    else:
        # 根据宽度计算缩放比例
        r = width / float(w)
        dim = (width, int(h * r))
    # 缩放图像
    resized = cv2.resize(image, dim, interpolation=inter)
    # 返回缩放后的图像
    return resized