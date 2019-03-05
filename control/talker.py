#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('VSLAM_info', PoseStamped, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "VSLAM_info"
        pose.pose.position.x = 1
        pose.pose.position.y = 1
        pose.pose.position.z = 0.25
        rospy.loginfo(pose)

        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass