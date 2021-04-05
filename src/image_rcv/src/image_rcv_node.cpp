#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "std_msgs/Empty.h"

using namespace std;
using namespace cv;


static const std::string OPENCV_WINDOW = "Image window";


//void imageCb(const sensor_msgs::ImageConstPtr& msg);

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
int count = 0;

    char t[100];

    sprintf(t,"%08d",count);    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  //bgr8
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    cv::imshow(OPENCV_WINDOW, frame);
    cv::imwrite(std::string("/home/control/camera/")+t+std::string(".png"),frame);
    count++;
    cv::waitKey(1);
}


int main(int argc, char** argv)
{
    static int count = 0;
    ros::init(argc, argv, "image_converter");

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber image_sub_;


    image_sub_ = it_.subscribe("/bebop2/image_raw", 1, imageCb);
    


    ros::Rate rate(10.0);

    //ImageConverter ic;
    while(ros::ok())
    {

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
