#-*- coding: UTF-8 -*-
from __future__ import division

# External Library:
from Pannal_Crossing.Birds_Eye_Transform import birds_eye_transform
from Pannal_Crossing.Image_Manipulation import *

# Zebra Crossing Detection

def is_zebra_crossing(line, length_of_line):
    n_points = line.shape[0]
    n_threshold = int(0.01/length_of_line*n_points)
    i=1
    n_valid_segment = 0
    start_segment =0
    color = line[0]
    while (i<n_points-1):
        while (i<n_points-2 )&(line[i+1]==line[i]) : 
            i+=1
        if (i-start_segment>=n_threshold):
            n_valid_segment+=1
        i+=1
        start_segment = i
    if (n_valid_segment>=12): return True
    else: return False

def zebra_crossing_detection(view_real,size_of_view=(-0.4,0.26,0.8,0.7),pix_per_m = 500, valid_distance=0.8):
    
    img_bird,dead_space = birds_eye_transform(view_real,size_of_view=[-0.4,0.26,0.8,0.7],pix_per_m = 500)
    
    img_bird[img_bird > 70] = 255
    img_bird[img_bird <= 70] = 0
    
    # Create randomly lines between two lines that may cut the zebra crossing
    n_lines =100
    P1_x = int((size_of_view[2]/2-0.2)*pix_per_m)
    P2_x = int((size_of_view[2]/2+0.2)*pix_per_m)
    P1_y_set = np.random.randint(int((0.3-size_of_view[1])*pix_per_m),int((valid_distance-size_of_view[1])*pix_per_m),size=n_lines)
    P2_y_set = np.random.randint(int((0.3-size_of_view[1])*pix_per_m),int((valid_distance-size_of_view[1])*pix_per_m),size=n_lines)

    P1_y_set[0]=150
    P2_y_set[0]=150

    # For each line, put the points in one ndarray of 1X 0.4*pix_per_m
    n_valid_lines =0
    dis =0
    for i in range(n_lines):
        n_points =int(0.4*pix_per_m)
        line = np.zeros(n_points)
        line_X = np.round(np.linspace(P1_x,P2_x,n_points)).astype('int')
        line_Y = np.round(np.linspace(P1_y_set[i],P2_y_set[i],n_points)).astype('int')
        line = img_bird[line_X,line_Y]
        if (is_zebra_crossing(line,sqrt((P1_x-P2_x)**2+(P1_y_set[i]-P2_y_set[i])**2)/pix_per_m)):
            dis+=((P1_y_set[i]+P2_y_set[i])*1./2/500+size_of_view[1])
            n_valid_lines += 1
    
    # If two lines have expectant features, we think that there is a zebra crossing line in the view
    if (n_valid_lines>2):
        return (True,dis/n_valid_lines)
    else:
        return (False,0)


if __name__ =='__main__':

    filebox=os.listdir("Data_Set\Encounter2") #
    len1=len(filebox)

    for i in range(len1):
        file_path = 'Data_Set\Encounter2\%s' %filebox[i]
        img_view = cv2.imread(file_path)
        white = np.zeros(img_view.shape)
        sign = resize_img(cv2.imread("Pannal_Crossing.jpg"),width=60,height=60)
        detected,dist=zebra_crossing_detection(img_view)
        # Draw the lane onto the warped blank image
        if detected:
            img_view[10:10+sign.shape[0],550:550+sign.shape[1],:]=sign[:,:,:]
            img_view[10+sign.shape[0]:35+sign.shape[0],550:550+sign.shape[1],:]=255
            font = cv2.FONT_HERSHEY_SIMPLEX
            img_view = cv2.putText(img_view, '%.2fm' %dist , (550,30+sign.shape[0]), font, 0.6, (0, 0, 0), 2)
        cv2.imwrite('Data_Set\Encounter2\detected_%s'%filebox[i], img_view)