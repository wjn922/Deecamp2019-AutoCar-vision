 

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/PoseStamped.h>
#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    
    ros::Publisher chatter_pub = nodeHandler.advertise<geometry_msgs::PoseStamped>("VSLAM_info", 100);
    ros::Subscriber sub = nodeHandler.subscribe("/foo/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::Rate loop_rate(1000);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

geometry_msgs::PoseStamped p;

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    p.pose.position.x = 1;
    p.pose.position.y = 2;
    p.pose.position.z = 3;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    chatter_pub.publish(p);

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}


