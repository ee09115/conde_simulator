#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#define DISTANCE_REFERENCE 0.4
#define ANGLE_REFERENCE 0
#define VELOCITY_REFERENCE 0.5

ros::Publisher dist_angle_pub;
std::string semaphore_data = "";

void signallingPanelCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    semaphore_data = msg->data.c_str();
}

void trackingCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{

    double right_distance = array->data[0];
    double right_angle = array->data[1];

    /// Publish Distance and Angle Info
    std_msgs::Float64MultiArray array_out;
    array_out.data.clear();
    array_out.data.push_back(right_distance);              // actual distance
    array_out.data.push_back(right_angle);                 // actual angle
    array_out.data.push_back(VELOCITY_REFERENCE);          // linear velocity reference
    array_out.data.push_back(DISTANCE_REFERENCE);          // reference distance
    array_out.data.push_back(ANGLE_REFERENCE);             // reference angle
    ros::Rate loop_rate(100);
    dist_angle_pub.publish(array_out);
    ros::spinOnce();
    loop_rate.sleep();

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "major_decision");

    ros::NodeHandle n("~");

    ros::Subscriber sub2 = n.subscribe("/tracking_msg", 100, trackingCallback);
    ros::Subscriber sub3 = n.subscribe("/signalling_panel_msg", 100, signallingPanelCallback);

    dist_angle_pub = n.advertise<std_msgs::Float64MultiArray>("/major_msg", 1);

    ros::spin();

    return 0;
}
