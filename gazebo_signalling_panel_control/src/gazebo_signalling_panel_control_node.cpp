#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include "std_msgs/String.h"

#include <termio.h>

sensor_msgs::ImagePtr im_msg;
cv::Mat image[5];

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_signalling_panel_controller");
    ros::NodeHandle nh1;
    image_transport::ImageTransport it1(nh1);
    image_transport::Publisher pub1 = it1.advertise("/monitor1/image1", 1);

    //ros::NodeHandle nh2;
    image_transport::ImageTransport it2(nh1);
    image_transport::Publisher pub2 = it2.advertise("/monitor2/image2", 1);

    std::string path = ros::package::getPath("gazebo_signalling_panel_control");
    printf("%s\n",path.c_str());

    image[0] = cv::imread(path + "/semaphores_pics/left.png", CV_LOAD_IMAGE_COLOR);
    image[1] = cv::imread(path + "/semaphores_pics/right.png", CV_LOAD_IMAGE_COLOR);
    image[2] = cv::imread(path + "/semaphores_pics/up.png", CV_LOAD_IMAGE_COLOR);
    image[3] = cv::imread(path + "/semaphores_pics/stop.png", CV_LOAD_IMAGE_COLOR);
    image[4] = cv::imread(path + "/semaphores_pics/parking.png", CV_LOAD_IMAGE_COLOR);

    if (image[0].empty() || image[1].empty() || image[2].empty() || image[3].empty() || image[4].empty())
    {
        printf("Error on reading images.\n");
        return 0;
    }

    im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[3]).toImageMsg();

    ros::Rate loop_rate(10);

    while (nh1.ok())
    {
        system("clear");
        printf("Choose the semaphore [Press <Esc> to exit]:\n");
        printf("[0] - left\n");
        printf("[1] - right\n");
        printf("[2] - up\n");
        printf("[3] - stop\n");
        printf("[4] - park\n");

        int opt = getch();
        switch (opt) {
        case '0':
            im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[0]).toImageMsg();
            break;
        case '1':
            im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[1]).toImageMsg();
            break;
        case '2':
            im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[2]).toImageMsg();
            break;
        case '3':
            im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[3]).toImageMsg();
            break;
        case '4':
            im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[4]).toImageMsg();
            break;
        case 27:
            exit(0);
            break;
        default:
            break;
        }

        pub1.publish(im_msg);
        pub2.publish(im_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    printf("FINISH\n");
}
