#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import numpy as np
import os
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import glob
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError





#################################################################
# Step 1 : Calculate camera distortion coefficients
#################################################################
def getCameraCalibrationCoefficients(chessboardname, nx, ny):
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((ny * nx, 3), np.float32)
    objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob(chessboardname)
    if len(images) > 0:
        print("images num for calibration : ", len(images))
    else:
        print("No image for calibration.")
        return

    ret_count = 0
    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_size = (img.shape[1], img.shape[0])
        # Finde the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

        # If found, add object points, image points
        if ret == True:
            ret_count += 1
            objpoints.append(objp)
            imgpoints.append(corners)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)
    print('Do calibration successfully')
    return ret, mtx, dist, rvecs, tvecs


#################################################################
# Step 2 : Undistort image
#################################################################
def undistortImage(distortImage, mtx, dist):
    return cv2.undistort(distortImage, mtx, dist, None, mtx)


#################################################################
# Step 3 : lane detection thresh
#################################################################


def white_select(img, thresh=[80, 255]):
    R = img[:,:,0]
    binary = np.zeros_like(R)
    binary[(R > thresh[0]) & (R <= thresh[1])] = 1
    return binary


def directional_gradient(img,direction='x',thresh=[0,255]):
    '''
    使用Opencv Sobel算子来求方向梯度
    img:Grayscale
    direction:x or y axis
    thresh:apply threshold on pixel intensity of gradient image
    output is binary image
    '''
    if direction=='x':
        sobel = cv2.Sobel(img,cv2.CV_64F,1,0)
    elif direction=='y':
        sobel = cv2.Sobel(img,cv2.CV_64F,0,1)
    sobel_abs = np.absolute(sobel)#absolute value
    scaled_sobel = np.uint8(sobel_abs*255/np.max(sobel_abs))
    binary_output = np.zeros_like(sobel)
    binary_output[(scaled_sobel>=thresh[0])&(scaled_sobel<=thresh[1])] = 1
    return binary_output


def color_binary(img,dst_format='HLS',ch=2,ch_thresh=[0,255]):
    '''
    Color thresholding on channel ch
    img:RGB
    dst_format:destination format(HLS or HSV)
    ch_thresh:pixel intensity threshold on channel ch
    output is binary image
    '''
    if dst_format =='HSV':
        img = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
        ch_binary = np.zeros_like(img[:,:,int(ch-1)])
        ch_binary[(img[:,:,int(ch-1)]>=ch_thresh[0])&(img[:,:,int(ch-1)]<=ch_thresh[1])] = 1
    else:
        img = cv2.cvtColor(img,cv2.COLOR_RGB2HLS)
        ch_binary = np.zeros_like(img[:,:,int(ch-1)])
        ch_binary[(img[:,:,int(ch-1)]>=ch_thresh[0])&(img[:,:,int(ch-1)]<=ch_thresh[1])] = 1
    return ch_binary


def roi_mask(img, vertices):
    mask = np.zeros_like(img)
    mask_color = 1
    cv2.fillPoly(mask, vertices, mask_color)
    masked_img = cv2.bitwise_and(img, mask)
    return masked_img




def birdView(img,M):
    '''
    Transform image to birdeye view
    img:binary image
    M:transformation matrix
    return a wraped image
    '''
    img_sz = (img.shape[1],img.shape[0])
    img_warped = cv2.warpPerspective(img,M,img_sz,flags = cv2.INTER_LINEAR)
    return img_warped


def perspective_transform(src_pts,dst_pts):
    '''
    perspective transform
    args:source and destiantion points
    return M and Minv
    '''
    M = cv2.getPerspectiveTransform(src_pts,dst_pts)
    Minv = cv2.getPerspectiveTransform(dst_pts,src_pts)
    return {'M':M,'Minv':Minv}


#################################################################
# Step 5 : Detect lane lines through moving window
#################################################################
def find_line(binary_warped):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # Choose the number of sliding windows
    nwindows = 10
    # Set height of windows
    window_height = np.int(binary_warped.shape[0] / nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Set the width of the windows +/- margin
    margin = 40
    # Set minimum number of pixels found to recenter window
    minpix = 20
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Fit a linear order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 1)
    right_fit = np.polyfit(righty, rightx, 1)

    flag = 1        # 是车道线
    if rightx_base - leftx_base < 150 or abs(left_fit[0]) > 0.4 or abs(right_fit[0]) > 0.4 \
            or leftx_base > 320 or rightx_base < 320:
        flag = 1    # 不是车道线

    return left_fit, right_fit, left_lane_inds, right_lane_inds, flag


#################################################################
# Step 6 : Track lane lines based the latest lane line result
#################################################################
def find_line_by_previous(binary_warped, left_fit, right_fit):
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = ((nonzerox > (left_fit[0] * (nonzeroy ** 2) + left_fit[1] * nonzeroy +
                                   left_fit[2] - margin)) & (nonzerox < (left_fit[0] * (nonzeroy ** 2) +
                                                                         left_fit[1] * nonzeroy + left_fit[
                                                                             2] + margin)))

    right_lane_inds = ((nonzerox > (right_fit[0] * (nonzeroy ** 2) + right_fit[1] * nonzeroy +
                                    right_fit[2] - margin)) & (nonzerox < (right_fit[0] * (nonzeroy ** 2) +
                                                                           right_fit[1] * nonzeroy + right_fit[
                                                                               2] + margin)))

    # Again, extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    # Fit a linear order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 1)
    right_fit = np.polyfit(righty, rightx, 1)
    return left_fit, right_fit, left_lane_inds, right_lane_inds


#################################################################
# Step 7 : Draw lane line result on undistorted image
#################################################################
def draw_area(undist, binary_warped, Minv, left_fit, right_fit, flag):
    if flag == 1:
        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        # left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        # right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
        left_fitx = left_fit[0] * ploty + left_fit[1]
        right_fitx = right_fit[0] * ploty + right_fit[1]

        # Create an image to draw the lines on
        warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, Minv, (undist.shape[1], undist.shape[0]))
        # Combine the result with the original image
        result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
    else:
        return undist
    return result


#################################################################
# Step 8 : Calculate distance from center
#################################################################
def calculate_curv_and_pos(binary_warped, left_fit, right_fit, flag):
    if flag == 1:
        # Define y-value where we want radius of curvature
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        # leftx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        # rightx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
        leftx = left_fit[0] * ploty + left_fit[1]
        rightx = right_fit[0] * ploty + right_fit[1]

        '''
        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 0.5 / 480  # meters per pixel in y dimension
        xm_per_pix = 0.3 / 240  # meters per pixel in x dimension
        y_eval = np.max(ploty)
        # Fit new polynomials to x,y in world space  linear model
        left_fit_cr = np.polyfit(ploty * ym_per_pix, leftx * xm_per_pix, 1)
        right_fit_cr = np.polyfit(ploty * ym_per_pix, rightx * xm_per_pix, 1)
        
         # Calculate the new radii of curvature
        left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * left_fit_cr[0])
        right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * right_fit_cr[0])
    
        curvature = ((left_curverad + right_curverad) / 2)
        # print(curvature)
        '''

        lane_width = np.absolute(leftx[479] - rightx[479])  # 480 y pixels
        lane_xm_per_pix = 0.3 / lane_width
        veh_pos = (((leftx[479] + rightx[479]) * lane_xm_per_pix) / 2.)
        cen_pos = ((binary_warped.shape[1] * lane_xm_per_pix) / 2.)
        distance_from_center = (cen_pos - veh_pos) * 1000   # mm
    else:
        distance_from_center = float('inf')
    return distance_from_center


def draw_values(img, distance_from_center):
    font = cv2.FONT_HERSHEY_SIMPLEX

    if distance_from_center > 0:
        pos_flag = 'right'
    else:
        pos_flag = 'left'

    center_text = "Vehicle is %.3fmm %s of center" % (abs(distance_from_center), pos_flag)
    cv2.putText(img, center_text, (20, 32), font, 0.8, (255, 255, 255), 2)
    return img



def callback_image(img):

    #print img.
    #mtx = np.load("mtx.npy")
    #dist = np.load("dist.npy")

    IMG = CvBridge().imgmsg_to_cv2(img)


    # 1 Do RGB
    distort_image = cv2.cvtColor(IMG, cv2.COLOR_BGR2RGB)

    # 2 Do undistortion
    #distort_image = undistortImage(distort_image, mtx, dist)

    # 3 Do sobel
    gray = cv2.cvtColor(distort_image, cv2.COLOR_RGB2GRAY)
    sobel = directional_gradient(gray, direction='x', thresh=[30, 255])
    # 4 Do hls
    # hls = color_binary(undistort_image, dst_format='HLS', ch=3, ch_thresh=[50,255])
    hls = white_select(distort_image, thresh=[80, 155])
    # 5 Do Combined
    combined_binary = np.zeros_like(sobel)
    combined_binary[((sobel == 1) | (hls == 1))] = 1

    # 6 Do roi_mask
    roi_vtx = np.array([[(0, distort_image.shape[0]), (0, 300), (320, 200), (640, 300), \
                            (distort_image.shape[1], distort_image.shape[0])]])
    masked_image = roi_mask(combined_binary, roi_vtx)

    # 7 Do WarpImage
    src = np.float32([[0, 480], [225, 280], [415, 280], [640, 480]])
    dst = np.float32([[200, 480], [200, 0], [440, 0], [440, 480]])
    transform_matrix = perspective_transform(src, dst)
    warped_image = birdView(masked_image * 1.0, transform_matrix['M'])

    # 8 Detect Lane
    left_fit = []
    right_fit = []
    ploty = []
    left_fit, right_fit, left_lane_inds, right_lane_inds, flag = find_line(warped_image)

    # 9 draw
    result = draw_area(distort_image, combined_binary, transform_matrix['Minv'], left_fit, right_fit, flag)

    # 10 calcurate bias
    dist_from_center = calculate_curv_and_pos(warped_image, left_fit, right_fit, flag)##
    result_image = draw_values(result, dist_from_center)

    # show a frame
    cv2.imshow("capture", result_image)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break
    
    #cv2.imshow('lane_img', IMG)
    cv2.waitKey(1)

    #cv2.destroyAllWindows()



if __name__ == '__main__':
    '''
    # 0 Do calibration
    nx = 7
    ny = 6
    ret, mtx, dist, rvecs, tvecs = getCameraCalibrationCoefficients('camera_cal/calibration*.jpg', nx, ny)
    '''
    print "start"
    rospy.init_node('lane_detection', anonymous=True)
    rospy.Subscriber('laneDetection/image', Image, callback_image)
    rospy.spin()

