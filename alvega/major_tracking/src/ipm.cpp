    #include "ipm.h"

ipm::ipm(cv::Mat matI, cv::Mat matTchessRobot, cv::Mat matTcamChess) : mat_I(matI), mat_T_chess_robot(matTchessRobot), mat_T_cam_chess(matTcamChess)
{
    // Intrisic Camera parameters
    /*
        [   alpha_x ,   beta    ,   x0
            0       ,   alpha_Y ,   y0
            0       ,   0       ,   1   ]

        alpha_x, alpha_y -> focal length
        x0, y0           -> principal point
    */
    /*mat_I = matI;(cv::Mat_<double>(3,3) << \
                                 126.710436859505222, 0                     , 170.717179526753455,\
                                 0                  , 127.001318670739508   , 122.445121932515121,\
                                 0                  , 0                     , 1);*/

    // Transformation Matrix Chessboard to Robot
    /*mat_T_chess_robot = matTchessRobot; (cv::Mat_<double>(4,4) <<  \
                             -1,0,0,0.575,\
                              0,-1,0,0.0,\
                              0,0,1,-0.004,\
                              0,0,0,1);*/

    // Transformation Matrix Camera to Chessboard
    /*mat_T_cam_chess = matTcamChess; (cv::Mat_<double>(4,4) << \
                              -0.003512, 	 0.999103, 	 -0.042202,0.0740,\
                           0.800275, 	 -0.022498, 	 -0.599211, -0.5040,\
                           -0.599622, 	 -0.035878, 	 -0.799478,1.7378);*/

    // Transformation Matrix Camera to robot
    mat_T_cam_robot = mat_T_cam_chess * mat_T_chess_robot;

//    std::cout << mat_T_cam_robot << std::endl;

    // IPM stuff
    mat_P = mat_I * mat_T_cam_robot(cv::Rect(0,0,3,3));
    mat_t = mat_I * mat_T_cam_robot(cv::Rect(3,0,1,3));

    // Window to see in IPM transformation
    x_min = 0;
    x_max = 2.75;
    y_min = -1.25;
    y_max = 1.25;

    // Intersection plane from IPM
    a = 0, b = 0, c = 1, d = 0;
    mat_plane = (cv::Mat_<double>(1,4) << a,b,c,d);
    cv::vconcat(-1*mat_t, -d, res2);
}

cv::Mat ipm::ipmTransform(std::vector<cv::Point2d> pt, int width, int height)
{
    int W = width;
    int H = height;
    cv::Mat image_final = cv::Mat::zeros(H,W,CV_8U);
    for (std::vector<cv::Point2d>::const_iterator it = pt.begin(); it!=pt.end(); it++)
    {
        cv::Mat mat_pix = -1*(cv::Mat_<double>(3,1)<< it->x,it->y,1);
        cv::Mat res1;
        cv::hconcat(mat_P,mat_pix,res1);
        cv::Mat mat_B;
        cv::vconcat(res1,mat_plane,mat_B);
        cv::Mat mat_aux = mat_B.inv()*res2;
        if (mat_aux.at<double>(0,0)>x_min && mat_aux.at<double>(0,0)<x_max && mat_aux.at<double>(1,0)>y_min && mat_aux.at<double>(1,0)<y_max)
        {
            int x = round(H-((mat_aux.at<double>(0,0)-x_min)*H/(x_max-x_min)));
            int y = round(W/2-(mat_aux.at<double>(1,0)*W/(y_max-y_min)));

            x == H ? x = H-1 : x;
            y == W ? y = W-1 : y;

            image_final.at<u_int8_t>(x, y) = 255;
            //std::cerr << round(H-((mat_aux.at<double>(0,0)-x_min)*H/(x_max-x_min)))-1 << " " << round(W/2-(mat_aux.at<double>(1,0)*W/(y_max-y_min)))-1<< "\t";
        }
    }
    //<< std::endlstd::cout << std::endl<< std::endl<< std::endl;
    return image_final;
}
///---------------------------------------------------------------------------------------------------------------------------------------------------------
std::vector<cv::Point> ipm::ipmTransformPoints(std::vector<cv::Point> pt, int width, int height)
{
    std::vector<cv::Point> Points;
    int W = width;
    int H = height;
    for (std::vector<cv::Point>::const_iterator it = pt.begin(); it!=pt.end(); it++)
    {
        cv::Mat mat_pix = -1*(cv::Mat_<double>(3,1)<< it->x,it->y,1);
        cv::Mat res1;
        cv::hconcat(mat_P,mat_pix,res1);
        cv::Mat mat_B;
        cv::vconcat(res1,mat_plane,mat_B);
        cv::Mat mat_aux = mat_B.inv()*res2;
        if (mat_aux.at<double>(0,0)>x_min && mat_aux.at<double>(0,0)<x_max && mat_aux.at<double>(1,0)>y_min && mat_aux.at<double>(1,0)<y_max)
        {
            int x = round(H-((mat_aux.at<double>(0,0)-x_min)*H/(x_max-x_min)));
            int y = round(W/2-(mat_aux.at<double>(1,0)*W/(y_max-y_min)));

            x == H ? x = H-1 : x;
            y == W ? y = W-1 : y;
            Points.push_back(cv::Point(y,x));
        }
    }
    return Points;
}

double ipm::calculateDistanceFromIPMPoint(cv::Point2d ipm_point, int width, int height)
{
    int W = width;
    int H = height;

    double X = ((H-ipm_point.y)*(x_max-x_min)/(double)H)+x_min;

    double Y = (W/2-ipm_point.x)*(y_max-y_min)/(double)W;


    double distance = sqrt(X*X+Y*Y);


    return distance;
}
