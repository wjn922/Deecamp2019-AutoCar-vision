#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include<iostream>
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "emmm");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Pose>("VSLAM_info", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    cout << "hehe" << endl;
    geometry_msgs::Pose p1;
    p1.position.x = 1;
    p1.position.y = 2;
    p1.position.z = 3;
 
    pub.publish(p1);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
 
