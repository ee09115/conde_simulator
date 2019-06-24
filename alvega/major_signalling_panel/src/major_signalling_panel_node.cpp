#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <ros/package.h>


ros::Publisher semaphore_pub;



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;

        std::string signalling_panel_data("false");


        if (signalling_panel_data != "false")
        {
            /// Publish Semaphore Info
            std_msgs::String msg;
            msg.data.clear();
            msg.data = signalling_panel_data;
            ros::Rate loop_rate(50);
            semaphore_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();

        }

        cv::imshow("rgb",img_rgb);

        uint8_t k = cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "major_signalling_panel");
    ros::NodeHandle nh;

    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/major_top_camera/image_raw", 1, imageCallback);

    semaphore_pub = nh.advertise<std_msgs::String>("/signalling_panel_msg", 1);

    ros::spin();
}
