#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

def callback_pose(pose):
    yaw = pose.orientation.z
    x   = pose.position.x
    y   = pose.position.y
 

 

if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('control', anonymous=True)

    rospy.Subscriber("VSLAM_info", Pose, callback_pose)

    control_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=100)
 
 
    try:
        print "start_move"

        while not rospy.is_shutdown():
            twist = Twist()
   
            twist.linear.x = 0
            twist.angular.z = 0
 
            control_cmd.publish(twist)

    except:
        print "Stop"

    finally:
        twist = Twist()
        control_cmd.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)