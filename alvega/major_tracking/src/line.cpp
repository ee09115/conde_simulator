#include "line.h"

//#define MAX_SPEED 10
//#define MAX_SIZE 5
//#define MAX_DIFF 5
//#define MAX_DISP 5

//#define HEIGHT 120
//#define WIDTH  160

//thresholdX = MAX_DISP;
//thresholdY = MAX_DISP;
//robotSpeed = MAX_SPEED;
//maxDispersionX = MAX_DISP;
//maxDiffSlope = MAX_DIFF;
//lifetime = MAX_SPEED;
//age = 0;
//past_mContainerSize = MAX_SIZE;
//wasUpdated = 0;

line::line()
{
    thresholdX = 5;
    thresholdY = 5;
    robotSpeed = 10;
    maxDispersionX = 5;
    maxDiffSlope = 5;
    numberOfPoints = 0;
    lifetime = 35;
    age = 0;
    past_mContainerSize = 5;
    wasUpdatedFlag = 0;
    percentageMargin = 10;
    imageHeight = 120;
    imageWidth = 160;
}

double line::calculateDispersion(std::vector<cv::Point> points) // change to better algorithm (difference between average of 3 most left congruent points and average of 3 most right congruent points)
{
    maxDispersionX = abs(points[0].x - points[points.size()].x);
    return maxDispersionX;
}

void line::updateParameters(line new_line)
{
    //double diff = new_line.m-m;
    //std::cout << "Slope diff: " << fabs(diff) << std::endl;
    updateSlopeM(new_line.m);
    b = new_line.b;
    distanceToOrigin = new_line.distanceToOrigin;
    angleToOrigin = new_line.angleToOrigin;
    numberOfPoints = new_line.numberOfPoints;
    age = 0;
    wasUpdatedFlag = 1;
}

void line::mergeParameters(line line_to_merge)
{
    double totalPoints = numberOfPoints + line_to_merge.numberOfPoints;
    double toMergeRatio = line_to_merge.numberOfPoints/totalPoints;
    double lineRatio = numberOfPoints/totalPoints;
    updateSlopeM(line_to_merge.m*toMergeRatio + m*lineRatio);
    b = line_to_merge.b*toMergeRatio + b*lineRatio;
    distanceToOrigin = line_to_merge.distanceToOrigin*toMergeRatio + distanceToOrigin*lineRatio;
    angleToOrigin = line_to_merge.angleToOrigin*toMergeRatio + angleToOrigin*lineRatio;
    age = 0;
    wasUpdatedFlag = 1;
    //numberOfPoints = totalPoints;
}

bool line::isCurve()
{
    if (maxDispersionX > thresholdX)
        return true;
    else
        return false;
}

bool line::isDiscontinuous() // must be called before update_slope
{
    double sum = 0.0;
    if (past_m.size() < past_mContainerSize)
        return false;
    for (int i = 0; i < past_m.size(); i++)
       sum = sum + past_m[i];
    if ( (sum / past_m.size() - m) > maxDiffSlope ) // change this
        return true;
    else
        return false;
}

bool line::isValid()
{
    if (age <= lifetime)
        return true;
    else
        return false;
}

bool line::isSame(line line_to_compare)
{
//    std::cout << "Old params: ";
//    printData();
//    std::cout << "New params: ";
//    line_to_compare.printData();

    if (line_to_compare.m > m-0.1 && line_to_compare.m < m+0.1
//       line_to_compare.m > m*(1-percentageMargin*getSign(m)) && line_to_compare.m < m*(1+percentageMargin*getSign(m))
        && line_to_compare.b > b-imageWidth/18 && line_to_compare.b < b+imageWidth/18
//       && line_to_compare.distanceToOrigin > distanceToOrigin*(1-percentageMargin*getSign(distanceToOrigin))
//       && line_to_compare.distanceToOrigin < distanceToOrigin*(1+percentageMargin*getSign(distanceToOrigin))
        )
    {
//        std::cout << "Same line." << std::endl;
        return true;
    }
    else
    {
//        std::cout << "Different line." << std::endl;
        return false;
    }
}

void line::updateSlopeM(double new_m)
{
    past_m.push_back(new_m);
    if (past_m.size() >= past_mContainerSize)
        past_m.pop_front();
    m = new_m;
    //m = past_m.back();
}

void line::updateCrossPointB(double new_b)
{
    b = new_b;
}

void line::updateDistanceToOrigin(double new_distanceToOrigin)
{
    distanceToOrigin = new_distanceToOrigin;
}

void line::updateAngleToOrigin(double new_angleToOrigin)
{
    angleToOrigin = new_angleToOrigin;
}

void line::updateNumberOfPoints(int new_numberOfPoints)
{
    numberOfPoints = new_numberOfPoints;
}

void line::updateLifetime(int new_lifetime)
{
    lifetime = new_lifetime;
}

void line::updateImageHeight(int new_imageHeight)
{
    imageHeight = new_imageHeight;
}

void line::updateImageWidth(int new_imageWidth)
{
    imageWidth = new_imageWidth;
}

void line::addAge()
{
    if (!wasUpdatedFlag)
        age++;
    wasUpdatedFlag = 0;
}

void line::updateRobotSpeed(double new_robotSpeed)
{
    robotSpeed = new_robotSpeed;
}

void line::calculateLifetime(double speed) //still in prototype
{
    if (speed < 0)
        return;
    if (speed > robotSpeed)
    {
        lifetime = robotSpeed;
        return;
    }
    lifetime = speed;
}

void line::printData()
{
    std::cout << "m: " << m << " b: " << b << " distanceToOrigin: " << distanceToOrigin << " age: " << age << std::endl;
}

double line::getSlopeM()
{
    return m;
}

double line::getCrossPointB()
{
    return b;
}

double line::getDistanceToOrigin() const
{
    return distanceToOrigin;
}

double line::getAngleToOrigin()
{
    return angleToOrigin;
}

int line::getNumberOfPoints()
{
    return numberOfPoints;
}

double line::getSign(double number)
{
    return fabs(number)/number;
}
