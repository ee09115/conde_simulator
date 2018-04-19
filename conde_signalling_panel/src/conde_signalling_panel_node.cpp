#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"

ros::Publisher semaphore_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;

        std::string semaphore_data("false");

        if (semaphore_data != "false")
        {
            /// Publish Semaphore Info
            std_msgs::String msg;
            msg.data.clear();
            msg.data = semaphore_data;
            ros::Rate loop_rate(200);
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

    ros::init(argc, argv, "conde_signalling_panel");
    ros::NodeHandle nh;
    cv::namedWindow("rgb");
//    cv::namedWindow("gray");
//    cv::namedWindow("bw");
//    cv::namedWindow("semaphore");

//    cv::namedWindow("green");
    //cv::namedWindow("right");
    //cv::namedWindow("up");
    //cv::namedWindow("down");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/conde_camera_signalling_panel/image_raw", 1, imageCallback);

    semaphore_pub = nh.advertise<std_msgs::String>("/conde_signalling_panel_info", 1);

//    template_cross = cv::imread("/home/valter/catkin_conde_ws/src/conde/conde_tracking/cross_templ.jpg", CV_LOAD_IMAGE_GRAYSCALE);

//    if (!template_cross.empty())
//        cv::imshow("semaphore",template_cross);
//    else
//    {
//        std::cout << "Error on reading" << std::endl;
//        exit(0);
//    }

    ros::spin();
    cv::destroyWindow("view");

}
