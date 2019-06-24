#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <stdlib.h>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/xfeatures2d.hpp"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    try
    {
        cv::Mat img_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::imshow("img_bgr", img_bgr);
        uint8_t k = cv::waitKey(1);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "major_traffic_sign_node");
    ros::NodeHandle nh;

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/major_top_camera/image_raw", 1, imageCallback);

    ros::spin();

}
