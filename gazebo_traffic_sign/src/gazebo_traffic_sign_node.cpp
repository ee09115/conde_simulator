#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include "std_msgs/String.h"

#include <termio.h>

sensor_msgs::ImagePtr im_msg;
//cv::Mat image[12];

int myrandom (int i) { return std::rand()%i;}


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
    ros::init(argc, argv, "gazebo_traffic_sign_controller");
    ros::NodeHandle nh1;
    
    image_transport::Publisher pub[12];
    std::vector<image_transport::ImageTransport> it;
    
    std::srand ( unsigned ( std::time(0) ) );

    
		for (int i = 0; i < 12; i++)
		{
			it.push_back(image_transport::ImageTransport (nh1));
		}
		for (int i = 1; i <= 12; i++)
		{
			char topic[50] = "";
			sprintf(topic,"/traffic_sign_%d/image", i);
			pub[i-1] = it[i-1].advertise(topic, 1);
		}
		
    std::string path = ros::package::getPath("gazebo_traffic_sign");
    printf("%s\n",path.c_str());
    
    std::vector<cv::Mat> image;
    
    cv::Mat img;
    char name[100] = {0};
    for (int i = 0; i <= 3; i++)
    {
    	for (int j = 1; j <=3; j++)
    	{
    		sprintf(name, "/traffic_sign_pics/%d%d.png", i,j);
    		img = cv::imread(path + name, CV_LOAD_IMAGE_COLOR);
				if (!img.empty())
				{
					image.push_back(img);
				}
				else
				{
					printf("error\n");
					return 0;
				}
  		}
    }
    
    // using myrandom:
  	std::random_shuffle ( image.begin(), image.end(), myrandom);

  	ros::Rate loop_rate(12);

		for (int i = 1; i <= 12; i++)
		{
			im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image[i-1]).toImageMsg();
			pub[i-1].publish(im_msg);
			ros::spinOnce();
		  loop_rate.sleep();
    }
          
      
    printf("FINISH\n");
}
