#include "tracking_crosswalk.h"

double vector_avg( std::vector<double> &data )
{
        double return_value = 0.0;
        double n = data.size();
        for ( int i=0; i < n; i++)
            return_value += data[i];
        return ( return_value / n);
}
///--------------------------------------------------------------------------------------------------
double vector_max( std::vector<double> &data )
{
    if(data.size() <= 0)
        return 0;
    double max = data[0];
    for(int i = 1; i < data.size(); i ++)
        if (data[i] > max)
            max = data[i];
    return max;
}
///---------------------------------------------------------------------------------------------------
inline static double sqr(double x) {
    return x*x;
}

//bool linreg(int n, std::vector<double> x, std::vector<double> y, double & m, double & b, double & r)
//{
//    double   sumx = 0.0;                        /* sum of x                      */
//    double   sumx2 = 0.0;                       /* sum of x**2                   */
//    double   sumxy = 0.0;                       /* sum of x * y                  */
//    double   sumy = 0.0;                        /* sum of y                      */
//    double   sumy2 = 0.0;                       /* sum of y**2                   */

//   for (int i=0;i<n;i++)
//      {
//      sumx  += x[i];
//      sumx2 += sqr(x[i]);
//      sumxy += x[i] * y[i];
//      sumy  += y[i];
//      sumy2 += sqr(y[i]);
//      }

//   double denom = (n * sumx2 - sqr(sumx));
//   if (denom == 0) {
//       // singular matrix. can't solve the problem.
//       m = 0;
//       b = 0;
//       if (r) r = 0;
//       return false;
//   }
//   m = (n * sumxy  -  sumx * sumy) / denom;
//   b = (sumy * sumx2  -  sumx * sumxy) / denom;
////   if (r!=NULL) {
//      r = (sumxy - sumx * sumy / n) /          /* compute correlation coeff     */
//            sqrt((sumx2 - sqr(sumx)/n) *
//            (sumy2 - sqr(sumy)/n));
////   }
//   return true;
//}

///---------------------------------------------------------------------------------------------------
struct WeightedData
{
double x;
double y;
double weight;
};

void findQuadraticFactors(WeightedData *data, double &a, double &b, double &c, unsigned int const datasize)
{
double w1 = 0.0;
double wx = 0.0, wx2 = 0.0, wx3 = 0.0, wx4 = 0.0;
double wy = 0.0, wyx = 0.0, wyx2 = 0.0;
double tmpx, tmpy;
double den;

for (unsigned int i = 0; i < datasize; ++i)
    {
    double x = data[i].x;
    double y = data[i].y;
    double w = data[i].weight;

    w1 += w;
    tmpx = w * x;
    wx += tmpx;
    tmpx *= x;
    wx2 += tmpx;
    tmpx *= x;
    wx3 += tmpx;
    tmpx *= x;
    wx4 += tmpx;
    tmpy = w * y;
    wy += tmpy;
    tmpy *= x;
    wyx += tmpy;
    tmpy *= x;
    wyx2 += tmpy;
    }

den = wx2 * wx2 * wx2 - 2.0 * wx3 * wx2 * wx + wx4 * wx * wx + wx3 * wx3 * w1 - wx4 * wx2 * w1;
if (den == 0.0)
    {
    a = 0.0;
    b = 0.0;
    c = 0.0;
    }
else
    {
    a = (wx * wx * wyx2 - wx2 * w1 * wyx2 - wx2 * wx * wyx + wx3 * w1 * wyx + wx2 * wx2 * wy - wx3 * wx * wy) / den;
    b = (-wx2 * wx * wyx2 + wx3 * w1 * wyx2 + wx2 * wx2 * wyx - wx4 * w1 * wyx - wx3 * wx2 * wy + wx4 * wx * wy) / den;
    c = (wx2 * wx2 * wyx2 - wx3 * wx * wyx2 - wx3 * wx2 * wyx + wx4 * wx * wyx + wx3 * wx3 * wy - wx4 * wx2 * wy) / den;
    }

}

double findY(double const a, double const b, double const c, double const x)
{
return a * x * x + b * x + c;
}
///----------------------------------------------------------------------------------------------------------------------------------------

tracking_crosswalk::tracking_crosswalk()
{
    init = true;
}
///----------------------------------------------------------------------------------------------------------------------------------------
bool tracking_crosswalk::initialize(double first_band, double h_band_with,  double dist_between_h_band,
                                       std::vector<double> initial_line_poses_X, double band_min_with,
                                       double band_max_with, std::string img_name, bool show_images, int number_of_last_poses, double IPM_POINT_X, int IPM_POINT_Y)
{
    ipmPointX = IPM_POINT_X;
    ipmPointY = IPM_POINT_Y;
    HbandWith = h_band_with;
    distBetweenHband = dist_between_h_band;
    numOfBands = initial_line_poses_X.size();
    linePoseHist = initial_line_poses_X;
    minWith = band_min_with;
    maxWith = band_max_with;
//    matI = matIgray;
    firstHband = first_band;
    showImgs = show_images;
    imgName = img_name;
    histograms.resize(numOfBands);
    if(showImgs) cv::namedWindow(imgName);

    for(int i = 0; i < numOfBands; i++)
    {
        std::ostringstream convert;
        convert << i;
        histograms[i].name = imgName + "_hist_" + convert.str();
        for(int k = 0; k < number_of_last_poses; k++)
            histograms[i].linePose.push_back(initial_line_poses_X[i]);
        histograms[i].calcLineEstimatedPose();
        histograms[i].bandCenterYval = firstHband - (i*distBetweenHband);
        histograms[i].ok = true;
        histograms[i].timeToNotOK = TIME_TO_NOT_OK_HIST_CROSSWALK;
        if(showImgs) cv::namedWindow(histograms[i].name);
    }


    return true;
}
//calc histogram for all bands
void tracking_crosswalk::calcHistogram()
{
    for(int i = 0; i < numOfBands; i++)
    {
        histograms[i].values.resize(matI.cols);
        for(int k = 0; k < matI.cols; k++)
        {
            histograms[i].values[k] = 0;
            for(int j = firstHband - floor(HbandWith/2) - (i*distBetweenHband); j <= (firstHband + floor(HbandWith/2)) - (i*distBetweenHband); j ++)
                histograms[i].values[k] += matI.at<uchar>(j, k);
            histograms[i].values[k] /= (255 * HbandWith); // scale 0 1
        }
    }

    //cacl max, avg, threshold
    for(int i = 0; i < numOfBands; i++)
    {
        histograms[i].maxVal = vector_max(histograms[i].values);
        histograms[i].avgVal = vector_avg(histograms[i].values);

        histograms[i].thresholdVal = (histograms[i].avgVal + (histograms[i].maxVal - histograms[i].avgVal)/2) < MIN_THRESH_VAL_CROSSWALK
                ? MIN_THRESH_VAL_CROSSWALK : (histograms[i].avgVal + (histograms[i].maxVal - histograms[i].avgVal)/2);
    }

    //show images
    if(showImgs)
    {
        for(int i = 0; i < numOfBands; i++)
        {
            //display band lines on gray image
            line(matI, cv::Point(0,firstHband - floor(HbandWith/2) - (i) * distBetweenHband),
                 cv::Point(matI.cols-1, firstHband - floor(HbandWith/2) - (i) * distBetweenHband), cv::Scalar(255,255,255), 1);
            line(matI, cv::Point(0,firstHband + floor(HbandWith/2) - (i) * distBetweenHband),
                 cv::Point(matI.cols-1, firstHband + floor(HbandWith/2) - (i) * distBetweenHband), cv::Scalar(255,255,255), 1);
            //create histograms
            histograms[i].img = cv::Mat::zeros(matI.rows, matI.cols,CV_8UC3);
            for(int k = 0; k < histograms[i].img.cols; k++)
                line(histograms[i].img, cv::Point(k,(histograms[i].img.rows-1)),
                     cv::Point(k,(histograms[i].img.rows-1) - floor(histograms[i].values[k] * (histograms[i].img.rows-1))), cv::Scalar(255,255,255), 1, CV_AA);
            // display on histogram the max, avg and threshold values
            line(histograms[i].img, cv::Point(0, (histograms[i].img.rows-1) - floor(histograms[i].avgVal * (histograms[i].img.rows-1))),
                 cv::Point(histograms[i].img.cols-1, (histograms[i].img.rows-1) - floor(histograms[i].avgVal * (histograms[i].img.rows-1))), cv::Scalar(255,0,0), 1, CV_AA);
            line(histograms[i].img, cv::Point(0, (histograms[i].img.rows-1) - floor(histograms[i].maxVal * (histograms[i].img.rows-1))),
                 cv::Point(histograms[i].img.cols-1, (histograms[i].img.rows-1) - floor(histograms[i].maxVal * (histograms[i].img.rows-1))), cv::Scalar(0,255,0), 1, CV_AA);
            line(histograms[i].img, cv::Point(0, (histograms[i].img.rows-1) - floor(histograms[i].thresholdVal * (histograms[i].img.rows-1))),
                 cv::Point(histograms[i].img.cols-1, (histograms[i].img.rows-1) - floor(histograms[i].thresholdVal * (histograms[i].img.rows-1))), cv::Scalar(0,0,255), 1, CV_AA);
        }
    }
}
///------------------------------------------------------------------------------------------------------

int tracking_crosswalk::findCrossWalk(cv::Mat Mat_I)
{
    static ros::Time prev_time;
    std::vector<cv::Point2d> Points;
    (showImgs) ? matI = Mat_I.clone() : matI = Mat_I;
    if(init)
    {    for(int i = 0; i < numOfBands ; i++)
            histograms[i].img = cv::Mat::zeros(matI.rows, matI.cols,CV_8UC3);
        init = false;
    }

    calcHistogram();
    for(int i = 0; i < numOfBands; i++)
        histograms[i].calcLineEstimatedPose();

    int stateHist = 0;
    for(int j = 0; j < numOfBands; j++)//for every histogram
    {
        histograms[j].intersections.clear();
        histograms[j].intersectionsWiths.clear();
        histograms[j].intersectionsWithsFiltred.clear();
        histograms[j].intersectionsCenters.clear();
        histograms[j].intersectionsCentersFiltred.clear();
        histograms[j].lineFound = false;
        stateHist = 0;
        for(int i = 0; i < histograms[j].values.size(); i++)
        {
            if((stateHist == 0) && (histograms[j].values[i] > histograms[j].thresholdVal))
            {
                histograms[j].intersections.push_back(i);
                stateHist = 1;
            }
            else if ((stateHist == 1) && (histograms[j].values[i] < histograms[j].thresholdVal))
            {
                histograms[j].intersections.push_back(i);
                histograms[j].intersectionsWiths.push_back(i - histograms[j].intersections[histograms[j].intersections.size() - 2]);
                histograms[j].intersectionsCenters.push_back( histograms[j].intersections[histograms[j].intersections.size() - 2]
                        + ((i - histograms[j].intersections[histograms[j].intersections.size() - 2])/2) );
                if( (histograms[j].intersectionsWiths[histograms[j].intersectionsWiths.size() - 1] > minWith)
                        && (histograms[j].intersectionsWiths[histograms[j].intersectionsWiths.size() - 1] < maxWith))
                {
                    histograms[j].intersectionsCentersFiltred.push_back(histograms[j].intersectionsCenters[histograms[j].intersectionsCenters.size() - 1]);
                    histograms[j].intersectionsWithsFiltred.push_back(histograms[j].intersectionsWiths[histograms[j].intersectionsWiths.size() - 1]);

                    if(showImgs)
                        cv::circle(histograms[j].img,cv::Point(floor( histograms[j].intersectionsCentersFiltred[histograms[j].intersectionsCentersFiltred.size() - 1])
                                   , histograms[j].img.rows - floor(histograms[j].thresholdVal * histograms[j].img.rows)),3,cv::Scalar(0,0,255),2);

//                    std::cout << j << " -> " << histograms[j].intersectionsWiths[histograms[j].intersectionsWiths.size() - 1] << "  |  "
//                                                                 << histograms[j].intersectionsCenters[histograms[j].intersectionsCenters.size() - 1] << " \t "
//                                                                 << histograms[j].lineSpeed << " \t "
//                                                                 << histograms[j].estimatedPose << " \t "
//                                                                 << histograms[j].intersectionsCenters[histograms[j].intersectionsCenters.size() - 1] << " \t "
//                                                                 << std::endl;
                }
                stateHist = 0;
            }
        }
    }


    double distBetweenBars;
    double distBetweenBarsSTD;
    for(int i = 1; i < histograms[0].intersectionsCentersFiltred.size(); i++)
    {
        distBetweenBars += histograms[0].intersectionsCentersFiltred[i] - histograms[0].intersectionsCentersFiltred[i-1];
    }
    if(histograms[0].intersectionsCentersFiltred.size() > 1)
    {
        distBetweenBars /= histograms[0].intersectionsCentersFiltred.size();
        for(int i = 1; i < histograms[0].intersectionsCentersFiltred.size(); i++)
        {
            distBetweenBarsSTD += fabs((histograms[0].intersectionsCentersFiltred[i] - histograms[0].intersectionsCentersFiltred[i-1]) - distBetweenBars);
        }
        distBetweenBarsSTD /= histograms[0].intersectionsCentersFiltred.size();
    }

//    std::cerr << "distBetweenBars:" << distBetweenBars << " distBetweenBarsSTD:" << distBetweenBarsSTD << std::endl;
//    std::cerr << (histograms[0].maxVal - histograms[0].avgVal) << std::endl;
//    std::cerr << "intersections: " << histograms[0].intersectionsCentersFiltred.size() << std::endl;
    if((histograms[0].intersectionsCentersFiltred.size() >= 4)
            && ((histograms[0].maxVal - histograms[0].avgVal) > 0.4) && (distBetweenBars > 12) && (distBetweenBars < 30) && (distBetweenBarsSTD < 10) && (distBetweenBarsSTD > 1.5))
    {
        std::cerr << "crossWalk" << std::endl;
    }


    static int stateCrossWalk = 0;
    ros::Time actual_time = ros::Time::now();
    if((stateCrossWalk == 0) && (histograms[0].intersectionsCentersFiltred.size() >= 4)
            && ((histograms[0].maxVal - histograms[0].avgVal) > 0.4) && (distBetweenBars > 12) && (distBetweenBars < 30) && (distBetweenBarsSTD < 10) && (distBetweenBarsSTD > 1.5))
    {
        stateCrossWalk = 1;
        histograms[0].crossWalkFoundTimer = std::clock();
        std::cerr << "------> found crossWalk" << std::endl;
    }
    else if( (stateCrossWalk == 1) && (histograms[0].thresholdVal < 0.6) &&  (histograms[0].intersectionsCentersFiltred.size() < 10) /*&& 0==distBetweenBarsSTD*/)
    {
        stateCrossWalk = 2;
        std::cerr << stateCrossWalk << std::endl;
        prev_time = actual_time;
    }
    else if( (stateCrossWalk == 2) &&  (actual_time.toSec()-prev_time.toSec())>5)
    {
        std::cerr << "crossWalk passed <------" << std::endl;
        stateCrossWalk = 0;
    }

    if(showImgs)
    {
        for(int j = 0; j < numOfBands; j++)
        {
            cv::circle(matI,cv::Point(floor(histograms[j].linePose[histograms[j].linePose.size() - 1])
                       , histograms[j].bandCenterYval ),4,cv::Scalar(0,0,0),2);
        }
        displayImages();
    }

    return stateCrossWalk;
}
///-------------------------------------------------------------------------------------------------------
void tracking_crosswalk::displayImages()
{
    for(int i = 0; i < numOfBands; i++)
    {
        cv::imshow(histograms[i].name, histograms[i].img);
    }
    cv::imshow(imgName, matI);
    uint8_t k = cv::waitKey(1);
}
///-------------------------------------------------------------------------------------------------------
void tracking_crosswalk::show_images(bool show)
{
    showImgs = show;
    if(showImgs)
    {
        cv::namedWindow(imgName);
        for(int i = 0; i < numOfBands; i++)
            cv::namedWindow(histograms[i].name);
    }
}


