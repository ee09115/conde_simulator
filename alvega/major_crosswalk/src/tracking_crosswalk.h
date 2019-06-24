#ifndef TRACKING_CROSSWALK_H
#define TRACKING_CROSSWALK_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <math.h>
#include <ctime>
#include "ros/ros.h"


#define MYSQR(X) ((X)*(X))
//#define MAX_LINE_X_POSE_DEVIATION 5
#define MIN_THRESH_VAL_CROSSWALK 0.4
#define TIME_TO_NOT_OK_HIST_CROSSWALK 40 //FRAMES

struct histogramSTCross{
    std::string name;
    cv::Mat img;
    double bandCenterYval;
    std::vector<double> values;
    double lastFoundPositon;
    double maxVal;
    double avgVal;
    double thresholdVal;
    std::vector<double> linePose;
    double estimatedPose;
    double lineSpeed;
    void calcLineSpeed()
    {
        for(int i = 0; i < linePose.size() - 1; i++)
            lineSpeed += linePose[i+1] - linePose[i];
        lineSpeed /= linePose.size();

    }
    void calcLineEstimatedPose()
    {
        calcLineSpeed();
        estimatedPose = linePose[linePose.size() - 1] + lineSpeed;
    }
    std::vector<double> intersections;
    std::vector<double> intersectionsWiths;
    std::vector<double> intersectionsWithsFiltred;
    std::vector<double> intersectionsCenters;
    std::vector<double> intersectionsCentersFiltred;
    bool ok;
    int timeToNotOK;
    bool lineFound;
    void updateTime()
    {
        (lineFound) ? timeToNotOK = TIME_TO_NOT_OK_HIST_CROSSWALK : timeToNotOK--;
        (timeToNotOK > 0) ? ok = true : ok = false;
        if(!ok)
            std::cerr << name << " Timeout " << timeToNotOK << std::endl;
    }
    void updateLastPose()
    {
        if(lineFound)
            lastFoundPositon = linePose[linePose.size()-1];
    }
    std::clock_t crossWalkFoundTimer;
};
//----------------------------------------------------------------------------------
class tracking_crosswalk
{
public:
    tracking_crosswalk();
    bool initialize(double first_band, double h_band_with,  double dist_between_h_band, std::vector<double> initial_line_poses_X, double band_min_with,
                       double band_max_with, std::string img_name,
                       bool show_images, int number_of_last_poses, double IPM_POINT_X, int IPM_POINT_Y);
    void show_images(bool show);
    void calcHistogram();
    int findCrossWalk(cv::Mat Mat_I);
    void displayImages();

private:
    std::string imgName;
    std::vector<histogramSTCross > histograms;
    double bandCenter;
    double firstHband;
    double HbandWith;
    double distBetweenHband;
    double numOfBands;
    std::vector<double> initPose;
    double minWith;
    double maxWith;
    std::vector<double> linePoseHist;
    cv::Mat matI;
    bool showImgs;
    bool init;
    double ipmPointX;
    double ipmPointY;
};

#endif // TRACKING_CROSSWALK_H
