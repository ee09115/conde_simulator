#ifndef LINE_H
#define LINE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <deque>
#include <numeric>

class line
{
public:
    line();

    double calculateDispersion(std::vector<cv::Point> points);
    void updateParameters(line new_line);
    void mergeParameters(line line_to_merge);
    void updateSlopeM(double new_m);
    void updateCrossPointB(double new_b);
    void updateDistanceToOrigin(double new_distanceToOrigin);
    void updateAngleToOrigin(double new_angleToOrigin);
    void updatePastSlope();
    void updateNumberOfPoints(int new_numberOfPoints);
    void updateLifetime(int new_lifetime);
    void updateThresholdM(double new_thresholdM);
    void updateThresholdB(double new_thresholdB);
    void updateImageHeight(int new_imageHeight); // Change b threshold - use image size
    void updateImageWidth(int new_imageWidth);
    void addAge();
    void updateRobotSpeed(double new_robotSpeed);
    void calculateLifetime(double speed);
    void printData();
    bool isSame(line line_to_compare);
    bool isCurve();
    bool isDiscontinuous(); // must be called before update_slope
    bool isValid();
    double getSlopeM();
    double getCrossPointB();
    double getDistanceToOrigin() const;
    double getAngleToOrigin();
    int getNumberOfPoints();

private:
    double getSign(double number);

    double m;
    std::deque<double> past_m;
    double b;
    double distanceToOrigin;
    double angleToOrigin;
    double thresholdX;
    double thresholdY;
    double thresholdM;
    double thresholdB;
    double robotSpeed;
    double maxDispersionX;
    double maxDiffSlope;
    double percentageMargin;
    int numberOfPoints;
    int imageHeight;
    int imageWidth;
    int lifetime;
    int age;
    int past_mContainerSize;
    bool wasUpdatedFlag;
};

#endif // LINE_H
