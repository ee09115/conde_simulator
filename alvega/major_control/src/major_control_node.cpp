#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
//#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <string>


ros::Publisher vel_pub;
//ros::NodeHandle *vel_node;
ackermann_msgs::AckermannDriveStamped velocityToSend;

void sensorCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{

    std::vector<float> data(5);

    data[0] = array->data[0];       // Actual Distance (m)
    data[1] = array->data[1];       // Actual Angle (rad)
    data[2] = array->data[2];       // Vlinear (m/s)
    data[3] = array->data[3];       // Reference Distance (m)
    data[4] = array->data[4];       // Reference Angle (rad)

    velocityToSend.drive.steering_angle = 0;
    velocityToSend.drive.steering_angle_velocity = 0;
    velocityToSend.drive.speed = 0;
    velocityToSend.drive.acceleration = 0;
    velocityToSend.drive.jerk = 0;

    ros::Rate loop_rate(50);
    vel_pub.publish(velocityToSend);
    ros::spinOnce();
    loop_rate.sleep();

}

int main(int argc, char **argv) {

    std::string ref_topic("/major_msg");

    ros::init(argc,argv,"major_control");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe(ref_topic, 50, sensorCallback);

    vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);

    ros::spin();

    return 0;
}
