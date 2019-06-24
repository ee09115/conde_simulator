#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <ros/package.h>
#include "std_msgs/Bool.h"


#include "tracking_crosswalk.h"

#define IPM_POINT_X 93
#define IPM_POINT_Y 117

#define DEBUG 1

ros::Publisher crossWalk_pub;

tracking_crosswalk crossWalkHist;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat img_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::Size gaussFilterSize;
        int val = ceil(img_bgr.cols/50);
        ((val % 2) > 0) ? gaussFilterSize.height = val : gaussFilterSize.height = val + 1;
        gaussFilterSize.width = gaussFilterSize.height;
        cv::GaussianBlur(img_bgr, img_bgr, gaussFilterSize, 0, 0);

//        cv::resize(img_bgr, img_bgr, cv::Size(), 0.25, 0.25);

        cv::Mat img_gray; cv::cvtColor(img_bgr, img_gray, cv::COLOR_BGR2GRAY);

        cv::Mat img_bw; cv::threshold(img_gray, img_bw, 180, 255, 0);



        int crossWalkFound = crossWalkHist.findCrossWalk(img_bw);
        static int stateCrossWalk = 0;

        if ( (stateCrossWalk == 0) && (crossWalkFound == 1) )
        {
            std_msgs::Bool ok;
            ok.data = true;
            crossWalk_pub.publish(ok);
            stateCrossWalk = 1;
        }
        else if( (stateCrossWalk == 1) && (crossWalkFound == 0) )
        {
            std_msgs::Bool ok;
            ok.data = false;
            crossWalk_pub.publish(ok);
            stateCrossWalk = 0;
        }

        #if DEBUG
            cv::imshow("bgr",img_bgr);
            cv::imshow("bw",img_bw);
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

    int flag_real;
    if(!(nh.getParam("real", flag_real))){
        std::cerr << "Parameter flag_real not found" << std::endl;
        std::cerr << "Usage example: rosrun major_crosswalk major_crosswalk_node _flag_real:=<flag>" << std::endl;
        std::cerr << "Available challenge identifiers are: 0/1" << std::endl;
        return 0;
    }

    std::vector<double> initPosesHistCrossWalk;
    initPosesHistCrossWalk.push_back(30);
    int offset = flag_real ? 40 : 40;
    crossWalkHist.initialize(IPM_POINT_Y - offset, 5, 20, initPosesHistCrossWalk, 5, 20, "LF_H", false, 1, IPM_POINT_X, IPM_POINT_Y);


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/major_tracking_camera/image_raw", 1, imageCallback);

    crossWalk_pub = nh.advertise<std_msgs::Bool>("/crossWalk", 1);

    ros::spin();
}
