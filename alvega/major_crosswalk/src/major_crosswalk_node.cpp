#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <ros/package.h>
#include "std_msgs/Bool.h"


#define DEBUG 1

ros::Publisher crossWalk_pub;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;

        #if DEBUG
            cv::imshow("bgr",img_bgr);
            uint8_t k = cv::waitKey(1);
        #endif
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "major_crosswalk");
    ros::NodeHandle nh("~");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/major_tracking_camera/image_raw", 1, imageCallback);

    crossWalk_pub = nh.advertise<std_msgs::Bool>("/crossWalk", 1);

    ros::spin();
}
