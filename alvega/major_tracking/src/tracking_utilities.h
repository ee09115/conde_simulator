#ifndef TRACKING_UTILITIES_H
#define TRACKING_UTILITIES_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

#include "line.h"

#define IPM_POINT_X 93
#define IPM_POINT_Y 119

class tracking_utilities
{
public:
    tracking_utilities();
    ~tracking_utilities();
    void select_points(cv::Mat img, std::vector<std::vector<cv::Point> > &lines, int opt);
    void select_points(cv::Mat img, std::vector<std::vector<std::vector<cv::Point> > > &lines, int opt);
    bool isDiscontinuous(std::vector<double> sloppy);
    bool isCurve(std::vector<cv::Point> &line);
    int test_points(std::vector<cv::Point> &line);
    void merge_grid_lines(std::vector<std::vector<line> > &lines, std::vector<line> &merged_lines);
    void remove_lines(std::vector< std::vector<cv::Point> > &lines, int limit);
    void remove_lines(std::vector<std::vector<std::vector<cv::Point> > > &lines, int limit);
    void remove_lines(std::vector<line> &lines, int limit);
    void draw_points(cv::Mat img, std::vector< std::vector<cv::Point> > lines);
    void draw_points(cv::Mat img, std::vector<std::vector<std::vector<cv::Point> > > &lines);
    void estimate_lines(std::vector< std::vector<cv::Point> > lines, std::vector < std::vector<double> > &line_info, cv::Mat debug_image);
    void estimate_lines(std::vector<std::vector<cv::Point> > new_lines, std::vector<line> &stored_line_info, cv::Mat debug_image);
    void estimate_lines_in_grid(std::vector<std::vector<std::vector<cv::Point> > > lines, std::vector<std::vector<line> > &line_info, cv::Mat debug_image);
    void estimate_lines_and_curves(std::vector<std::vector<cv::Point> > new_lines, std::vector<line> &stored_line_info, cv::Mat debug_image);
    void update_lines(std::vector<line> &stored_line_info, std::vector<line> &new_line_info);
    void skeletonize(cv::Mat src, cv::Mat &skel);

    void calculateRightLineBasedOnLeftLine(double left_m, double left_b, double &new_distIPM, double &new_angleIPM, cv::Mat debug);

    double vector_avg( std::vector<double> &data );
    bool linreg(int n, std::vector<cv::Point2d> points, double & m, double & b, double & r);
    bool linRANSAC(cv::Mat data_points, double & m, double & b);
    double sqr(double x);
    double vector_max( std::vector<double> &data );

};

#endif // TRACKING_UTILITIES_H
