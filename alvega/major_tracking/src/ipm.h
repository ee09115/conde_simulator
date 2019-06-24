#ifndef IPM_H
#define IPM_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

class ipm
{
public:
    ipm(cv::Mat matI, cv::Mat matTchessRobot, cv::Mat matTcamChess);
    cv::Mat ipmTransform(std::vector <cv::Point2d> pt,int width,int height);
    std::vector<cv::Point> ipmTransformPoints(std::vector<cv::Point> pt, int width, int height);

    double calculateDistanceFromIPMPoint(cv::Point2d ipm_point, int width, int height);

private:
    cv::Mat mat_I;
    cv::Mat mat_T_chess_robot;
    cv::Mat mat_T_cam_chess;
    cv::Mat mat_T_cam_robot;
    cv::Mat mat_P;
    cv::Mat mat_t;

    double x_min,x_max;
    double y_min,y_max;
    int a,b,c,d;
    cv::Mat res2;

    cv::Mat mat_plane;
};

#endif // IPM_H
