#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Twist,TransformStamped,QuaternionStamped
from sensor_msgs.msg import Imu
from tf.msg import tfMessage
import math as M

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


###################################################################
#para:
#orb_slam:
yaw = 0 ; yaw_init = 0  
x = 0   ; x_init = 0  
y = 0   ; y_init = 0 
left_right_porta = 0
####
#turtlebot3:
imu_yaw = 0  ; imu_yaw_init = 0  
turtle_x = 0 ; turtle_x_init = 0  

realWorld_x = 0
realWorld_y = 0

###
symbol_yaw = -1
symbol_x   = -1
symbol_y   = -1

start_point = 0
end_point   = 0.2282
real_distance = 22.5

slam_realWorld_scale = abs(real_distance / (end_point - start_point))
print slam_realWorld_scale

slam_realWorld_startpoint = symbol_y*slam_realWorld_scale*start_point
slam_realWorld_end_point  = symbol_y*slam_realWorld_scale*end_point
print slam_realWorld_startpoint
print slam_realWorld_end_point

#start_value = slam_realWorld_startpoint ; end_value = slam_realWorld_end_point
start_value = 0  ; end_value = 15
move_speed = 0;rotate_speed = 0;tar_yaw = 0

####
turnleft_featureValue_threshold = 0.25
turnright_featureValue_threshold = 1.0 * 1 / turnleft_featureValue_threshold
print  turnright_featureValue_threshold
####



###############################################################
data_init_flag = 1
data_init_cnt  = 0
move_move_flag = 0
def callback_pose(pose):
    global data_init_flag ; global data_init_cnt ; global move_move_flag
    global left_right_porta 
    global yaw         ; global yaw_init ; global symbol_yaw  
    global x           ; global x_init   ; global symbol_x   
    global y           ; global y_init   ; global symbol_y  
    global realWorld_x ; global realWorld_y
    
    if data_init_flag == 1:
        move_move_flag = 0
        yaw_init = symbol_yaw*pose.orientation.z
        x_init   = symbol_x  *pose.position.x
        y_init   = symbol_y  *pose.position.y

        data_init_cnt = data_init_cnt + 1
        print data_init_cnt
        if data_init_cnt >= 10:
            data_init_flag = 0

    if data_init_flag == 0:
        yaw = (symbol_yaw*pose.orientation.z - yaw_init)*180/3.1415926
        #x   = (symbol_x*pose.position.x - x_init) * M.cos(yaw) + (symbol_y*pose.position.y - y_init) * M.sin(yaw)
        #y   = (symbol_y*pose.position.y - y_init) * M.cos(yaw) - (symbol_x*pose.position.x - x_init) * M.sin(yaw)
        x   = symbol_x*pose.position.x - x_init
        y   = symbol_y*pose.position.y - y_init
        move_move_flag = 1

    left_right_porta  = pose.position.z
 
    print yaw
    #print x
    print y
    print "\n"


odom_flag = 1 ; odom_cnt = 0
def callback_tf(tf):
    global odom_flag ; global odom_cnt 
    global turtle_x ; global turtle_x_init
 
    if odom_flag == 1:
        turtle_x_init = tf.transforms[0].transform.translation.x 
        odom_cnt = odom_cnt + 1
        if odom_cnt >= 10:
            odom_flag = 0
    if odom_flag == 0:
        turtle_x = tf.transforms[0].transform.translation.x - turtle_x_init
 

#####################################################################
def tf_coordination():
    global slam_realWorld_scale
    global turtle_x ; global yaw
    global realWorld_x ; global realWorld_y

    realWorld_odom = slam_realWorld_scale * turtle_x
    #realWorld_x = M.sin(yaw) * realWorld_odom
    #realWorld_y = M.cos(yaw) * realWorld_odom

    #print realWorld_x
    #print realWorld_y
    #print "\n"


forward_flag  = 0 ; backward_flag = 0
def loopMove_test(twist):
    global move_move_flag
    global yaw ; global realWorld_y
    global forward_flag ; global backward_flag 
    global start_value  ; global end_value

    global move_speed 
    global rotate_speed ; rotate_speed_max = 0.15 ; rotate_speed_pk = 0.5

    if move_move_flag == 1:
        rotate_speed = -rotate_speed_pk * (0 - yaw)
        if abs(rotate_speed) > rotate_speed_max:
            if rotate_speed < 0:
                rotate_speed = -rotate_speed_max
            else:
                rotate_speed = rotate_speed_max

        if forward_flag == 1 and backward_flag == 0:
            twist.linear.x = move_speed
            twist.angular.z = rotate_speed 
            control_cmd.publish(twist)

        if forward_flag == 0 and backward_flag == 1:
            twist.linear.x = -move_speed
            twist.angular.z = rotate_speed 
            control_cmd.publish(twist)

        if realWorld_y >= end_value :
            forward_flag = 0
            backward_flag= 1
        
        if realWorld_y <= start_value :
            forward_flag = 1
            backward_flag= 0      
callback_tf

process_flag = 1 
p1_y = 0.87
p2_y = 1
p3_y = 1.04
p4_y = 0.5

def main_navigation(twist):
    global process_flag 
    global p1_y ; global p2_y ; global p3_y ; global p4_y
    
    global move_move_flag
    global yaw ; global x ; global y
    global forward_flag ; global backward_flag 
    global start_value  ; global end_value

    global move_speed ; global tar_yaw
    global rotate_speed ; rotate_speed_max = 0.1 ; rotate_speed_pk = 0.05

    #if yaw < angle_90_init:
    #    real_yaw = yaw
    #else:
    #    real_yaw = abs(angle_90_init-yaw)

    if move_move_flag == 1:
        #if start_moving_flag == 1 and process_flag == 0 and abs(y) < 0.01:
        #    process_flag = 1
        #    start_moving_flag = 0

        #if start_moving_flag == 0 and process_flag == 0 and abs(y) >= 0.01:
        #    start_moving_flag = 1

        if process_flag == 1:
            if y <= p1_y:
                move_speed = 0.04
                tar_yaw    = 0
            else:
                move_speed = 0
                tar_yaw    = 90
                process_flag = 2

        if process_flag == 2:
            if abs(tar_yaw - yaw) <= 1 :
                process_flag = 3
             
        if process_flag == 3:
            if y <= p2_y:
                move_speed = 0.04
                tar_yaw    = 90
            else:
                move_speed = 0
                tar_yaw    = 0
                process_flag = 4            
            
        if process_flag == 4:
            if abs(tar_yaw - yaw) <= 1 :
                process_flag = 5

        if process_flag == 5:
            if y <= p3_y:
                move_speed = 0.04
                tar_yaw    = 0
            else:
                move_speed = 0
                tar_yaw    = 90
                process_flag = 10
        
        if process_flag == 10:
            if abs(tar_yaw - yaw) <= 1 :
                process_flag = 11
        
        if process_flag == 11:
            if y >= p4_y:
                move_speed = -0.04
                tar_yaw    = 90
            else:
                move_speed = 0
                tar_yaw    = 90
                process_flag = 12




        if process_flag == 6:
            if abs(tar_yaw - yaw) <= 5 :
                process_flag = 7
        
        if process_flag == 7:
            if y <= p4_y:
                move_speed = 0.04
                tar_yaw    = 0
            else:
                move_speed = 0
                tar_yaw    = 90
                process_flag = 8

        if process_flag == 8:
            if abs(tar_yaw - yaw) <= 5 :
                process_flag = 9

        if process_flag == 9:
            if y <= p5_y:
                move_speed = 0.04
                tar_yaw    = 0
            else:
                move_speed = 0
                tar_yaw    = 0
                process_flag = 10


        if process_flag >= 1:
            rotate_speed = -rotate_speed_pk * (tar_yaw - yaw)
            if abs(rotate_speed) > rotate_speed_max:
                if rotate_speed < 0:
                    rotate_speed = -rotate_speed_max
                else:
                    rotate_speed = rotate_speed_max

            twist.linear.x = move_speed
            twist.angular.z = rotate_speed 
            control_cmd.publish(twist)
        

###################################################################

if __name__ == '__main__':

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('control', anonymous=True)

    rospy.Subscriber("VSLAM_info", Pose, callback_pose)
    rospy.Subscriber("tf", tfMessage, callback_tf)

    control_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=100)


    try:
        print "start_move"

        while not rospy.is_shutdown():
            #tf_coordination()
            twist = Twist()
        
            #loopMove_test(twist)
            main_navigation(twist)

            time.sleep(0.01)
            

    except:
        print "Stop"

    finally:
        twist = Twist()
        control_cmd.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)