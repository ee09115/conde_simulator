#include "tracking_utilities.h"
cv::RNG rng(12345);

tracking_utilities::tracking_utilities()
{

}

tracking_utilities::~tracking_utilities()
{

}

static const cv::Scalar colors[] = {
    cv::Scalar(0,128,0),      //COLORS_green
    cv::Scalar(130,0,75),     //COLORS_indigo
    cv::Scalar(0,165,255),    //COLORS_orange
    cv::Scalar(255,0,255),    //COLORS_fuchsia
    cv::Scalar(0,215,255),    //COLORS_gold
    cv::Scalar(192,192,192),  //COLORS_silver
    cv::Scalar(128,128,128),  //COLORS_gray
    cv::Scalar(255,255,0),    //COLORS_cyan
    cv::Scalar(180,105,255),  //COLORS_hotpink
    cv::Scalar(255,0,0),      //COLORS_blue
    cv::Scalar(250,230,230),  //COLORS_lavender
    cv::Scalar(42,42,165),    //COLORS_brown
    cv::Scalar(50,205,50),    //COLORS_limegreen
    cv::Scalar(255,0,255),    //COLORS_magenta
    cv::Scalar(128,0,0),      //COLORS_navy
    cv::Scalar(203,192,255),  //COLORS_pink
    cv::Scalar(128,0,128),    //COLORS_purple
    cv::Scalar(0,0,255),      //COLORS_red
    cv::Scalar(143,143,188),  //COLORS_rosybrown
    cv::Scalar(60,20,220),    //COLORS_crimson
    cv::Scalar(238,130,238),  //COLORS_violet
    cv::Scalar(0,255,255),    //COLORS_yellow
    cv::Scalar(255,255,255),  //COLORS_white
    cv::Scalar(0,0,0)        //COLORS_black
};

std::vector<cv::Scalar> colorList(colors, colors + sizeof(colors) / sizeof(colors[0]) );

void tracking_utilities::select_points(cv::Mat img, std::vector<std::vector<cv::Point> > &lines, int opt)
{
    bool left_point_flag = false;
    int left_x;
    int th = 128;
    int line_width_thresh = 6;
    int x_thresh = 5, y_thresh = 20;
//    double slippy;
//    std::vector<std::vector<double> > sloppy;

    switch (opt)
    {
        case 0:// Bottom up approach
            for (int i = img.rows-1; i > img.rows/2; i-=1)//y
            {
                left_point_flag = false;
                for (int j = 0; j < img.cols-1; j++)//x
                {
                    if ( img.at<u_int8_t>(i,j) < th && img.at<u_int8_t>(i,j+1) > th )//left point
                    {
                        left_point_flag = true;
                        left_x = j+1;
                    }
                    else if ( (img.at<u_int8_t>(i,j) > th && img.at<u_int8_t>(i,j+1) < th) && (left_point_flag && abs(left_x-j) < line_width_thresh) )//right point
                    {

                        left_point_flag = false;
                        if (lines.size()==0)
                        {
                            lines.push_back(std::vector<cv::Point>());
                            lines[0].push_back(cv::Point((left_x+j)/2,i));
                        }
                        else
                        {
                            bool new_line_flag = false;
                            for (int k = 0; k < lines.size(); k++)
                            {
                                if ( (abs(lines[k].back().x - (left_x+j)/2) < x_thresh) && (abs(lines[k].back().y - i) < y_thresh) )
                                {
                                    lines[k].push_back(cv::Point((left_x+j)/2,i));
                                    new_line_flag = true;
                                    break;
                                }
                            }
                            if (!new_line_flag)
                            {
                                lines.push_back(std::vector<cv::Point>());
                                lines[lines.size()-1].push_back(cv::Point((left_x+j)/2,i));
                            }
                        }
                    }
                }
            }
            break;

        case 1:// Top bottom approach
            for (int i = img.rows/2; i < img.rows; i++)//y
            {
                left_point_flag = false;
                for (int j = 0; j < img.cols-20; j++)//x
                {
                    if ( img.at<u_int8_t>(i,j) < th && img.at<u_int8_t>(i,j+1) > th )//left point
                    {
                        left_point_flag = true;
                        left_x = j+1;
                    }
                    else if ( (img.at<u_int8_t>(i,j) > th && img.at<u_int8_t>(i,j+1) < th) && left_point_flag && abs(left_x-j)<line_width_thresh)//right point
                    {

                        left_point_flag = false;
                        if (lines.size()==0)
                        {
                            lines.push_back(std::vector<cv::Point>());
                            lines[0].push_back(cv::Point((left_x+j)/2,i));
                        }
                        else
                        {
                            bool new_line_flag = false;
                            for (int k = 0; k < lines.size(); k++)
                            {
                                if ( (abs(lines[k].back().x - (left_x+j)/2) < x_thresh) && (abs(lines[k].back().y - i) < y_thresh) )
                                {
                                    lines[k].push_back(cv::Point((left_x+j)/2,i));
                                    new_line_flag = true;
                                    break;
                                }
                            }
                            if (!new_line_flag)
                            {
                                lines.push_back(std::vector<cv::Point>());
                                lines[lines.size()-1].push_back(cv::Point((left_x+j)/2,i));
                            }
                        }
                    }
                }
            }
            break;

        case 2:// Bottom up approach using almost full image
            for (int i = img.rows-1; i > img.rows/4; i-=1)//y
            {
                left_point_flag = false;
                for (int j = 0; j < img.cols-1; j++)//x
                {
                    if ( img.at<u_int8_t>(i,j) < th && img.at<u_int8_t>(i,j+1) > th )//left point
                    {
                        left_point_flag = true;
                        left_x = j+1;
                    }
                    else if ( (img.at<u_int8_t>(i,j) > th && img.at<u_int8_t>(i,j+1) < th) && left_point_flag && abs(left_x-j)<line_width_thresh)//right point
                    {

                        left_point_flag = false;
                        if (lines.size()==0)
                        {
                            lines.push_back(std::vector<cv::Point>());
                            lines[0].push_back(cv::Point((left_x+j)/2,i));
                        }
                        else
                        {
                            bool new_line_flag = false;
                            for (int k = 0; k < lines.size(); k++)
                            {
                                if ( (abs(lines[k].back().x - (left_x+j)/2) < x_thresh) && (abs(lines[k].back().y - i) < y_thresh) )
                                {
                                    lines[k].push_back(cv::Point((left_x+j)/2,i));
                                    new_line_flag = true;
                                    break;
                                }
                            }
                            if (!new_line_flag)
                            {
                                lines.push_back(std::vector<cv::Point>());
                                lines[lines.size()-1].push_back(cv::Point((left_x+j)/2,i));
                            }
                        }
                    }
                }
            }
            break;

//        case 2:// Bottom up approach with slope check
//            for (int i = img.rows-1; i > img.rows/2; i-=1)//y
//            {
//                left_point_flag = false;
//                for (int j = 0; j < img.cols-1; j++)//x
//                {
//                    if ( img.at<u_int8_t>(i,j) < th && img.at<u_int8_t>(i,j+1) > th )//left point
//                    {
//                        left_point_flag = true;
//                        left_x = j+1;
//                    }
//                    else if ( (img.at<u_int8_t>(i,j) > th && img.at<u_int8_t>(i,j+1) < th) && left_point_flag && abs(left_x-j)<line_width_thresh)//right point
//                    {
//                        left_point_flag = false;
//                        if (lines.size()==0)
//                        {
//                            lines.push_back(std::vector<cv::Point>());
//                            sloppy.push_back(std::vector<double>());
//                            lines[0].push_back(cv::Point((left_x+j)/2,i));
//                        }
//                        else
//                        {
//                            bool same_line_flag = false;
//                            for (int k = 0; k < lines.size(); k++)
//                            {
//                                if ( abs(lines[k].back().x - (left_x+j)/2) < x_thresh)
//                                {
//                                    lines[k].push_back(cv::Point((left_x+j)/2,i));
//                                    same_line_flag = true;
//                                    if (lines[k].back().y - lines[k].rbegin()[1].y == 0)
//                                    {
//                                        slippy = std::numeric_limits<double>::max();
//                                        std::cout << "IT'S NO USE!!!" << std::endl;
//                                    }
//                                    else
//                                    {
//                                        slippy = (lines[k].back().x - lines[k].rbegin()[1].x)/(lines[k].back().y - lines[k].rbegin()[1].y);
//                                        std::cout << "Last Point: " << lines[k].back().x << " " << lines[k].back().y << " " << std::endl;
//                                        std::cout << "2nd Last Point: " << lines[k].rbegin()[1].x << " " << lines[k].rbegin()[1].y << " " << std::endl;
//                                        std::cout << "X diff: " << lines[k].back().x - lines[k].rbegin()[1].x << " Y diff: " << lines[k].back().y - lines[k].rbegin()[1].y << std::endl;
//                                    }
//                                    sloppy[k].push_back(slippy);
//                                    if (isDiscontinuous(sloppy[k]))
//                                    {
//                                        std::cout << "Slippy is discontinuous. Shit." << std::endl;
//                                        sloppy[k].pop_back();
//                                        same_line_flag = false;
//                                        break;
//                                    }
//                                    break;
//                                }
//                            }
//                            if (!same_line_flag)
//                            {
//                                lines.push_back(std::vector<cv::Point>());
//                                lines[lines.size()-1].push_back(cv::Point((left_x+j)/2,i));
//                                sloppy.push_back(std::vector<double>());
//                                sloppy[sloppy.size()-1].push_back(slippy);
//                            }
//                        }
//                    }
//                }
//            }
//            break;

        default:
            break;
    }
}

void tracking_utilities::select_points(cv::Mat img, std::vector<std::vector<std::vector<cv::Point> > > &lines, int opt)
{
    bool left_point_flag = false;
    int left_x;
    int th = 128;
    int x_start, x_end;
    int y_start, y_end;
    int n = 0;
    int line_width_thresh = 6;
    int x_thresh = 5, y_thresh = 20;
//    y_start = 0;
//    y_end = (1/3)*img.rows - 1;

    switch (opt)
    {
        case 0:// 3x3 grid w/ bottom up approach

            for (int pos = 0; pos < 9; pos++)
            {
                lines.push_back(std::vector<std::vector<cv::Point> >());
                if ( pos%3 == 0 )
                {
                    if (pos != 0)
                        n += 3;
                    x_start = ((pos/3)*img.cols)/3;
                    x_end = ((pos/3+1)*img.cols)/3 - 1;
                }
                y_start = ((pos-n)*img.rows)/3;
                y_end = ((pos-n+1)*img.rows)/3 - 1;
                //std::cout << " pos: " << pos << " x_i: " << x_start << " x_f: " << x_end << " y_i: " << y_start << " y_f: " << y_end << " n: " << n << std::endl;

                for (int i = y_end; i > y_start; i-=1)//y
                {
                    left_point_flag = false;
                    for (int j = x_start; j < x_end; j++)//x
                    {
                        if ( (img.at<u_int8_t>(i,j) < th && img.at<u_int8_t>(i,j+1) > th)
                           ||(img.at<u_int8_t>(i,j) > th && j == x_start)  )//left point
                        {
                            left_point_flag = true;
                            left_x = j+1;
                        }
                        else if ( ( (img.at<u_int8_t>(i,j) > th && img.at<u_int8_t>(i,j+1) < th) && left_point_flag && abs(left_x-j) < line_width_thresh)
                                  ||((img.at<u_int8_t>(i,j) > th && j+1 == x_end) && left_point_flag && abs(left_x-j) < line_width_thresh) )//right point
                        {

                            left_point_flag = false;
                            if (lines[pos].size()==0)
                            {
                                lines[pos].push_back(std::vector<cv::Point>());
                                lines[pos][0].push_back(cv::Point((left_x+j)/2,i));
                            }
                            else
                            {
                                bool new_line_flag = false;
                                for (int k = 0; k < lines[pos].size(); k++)
                                {
                                if ( (abs(lines[pos][k].back().x - (left_x+j)/2) < x_thresh) && (abs(lines[pos][k].back().y - i) < y_thresh) )
                                    {
                                        lines[pos][k].push_back(cv::Point((left_x+j)/2,i));
                                        new_line_flag = true;
                                        break;
                                    }
                                }
                                if (!new_line_flag)
                                {
                                    lines[pos].push_back(std::vector<cv::Point>());
                                    lines[pos][lines[pos].size()-1].push_back(cv::Point((left_x+j)/2,i));
                                }
                            }
                        }
                    }
                }
            }
            break;

        default:
            break;
    }
}

bool tracking_utilities::isCurve(std::vector<cv::Point> &line)
{
//    int line_segment_size = 10; // 120(height)/8 segments
//    int number_of_segments = line.size()/line_segment_size;
//    if (number_of_segments < 3)
//    {
//        std::cout << line.size() << " -> too small" << std::endl;
//        return 0;
//    }
    int number_of_segments = 3;
    int line_segment_size = line.size()/number_of_segments;

    int slope_counter = 0, b_counter = 0;
    double mIPM, bIPM, rIPM;
    std::vector<double> mIPM_vec, bIPM_vec, rIPM_vec;
//    std::cout << "- begin isCurve -" << std::endl;
    for (int i = 0; i < number_of_segments; i++)
    {
        std::vector<cv::Point2d> rightPointsInverted;
        for (int j = 0; j < line_segment_size; j++)
        {
            rightPointsInverted.push_back(cv::Point2d(line[j+i*line_segment_size].y, line[j+i*line_segment_size].x));
        }
        linreg(line_segment_size, rightPointsInverted, mIPM, bIPM, rIPM);
        mIPM_vec.push_back(mIPM);
        bIPM_vec.push_back(bIPM);
        rIPM_vec.push_back(rIPM);
//        std::cout << "slope " << i << ": " << mIPM << " - b: " << bIPM << std::endl;
        if (i>0)
        {
            if (mIPM_vec[i]-mIPM_vec[i-1] > 0)
            {
                slope_counter++;
            }
            else if (mIPM_vec[i]-mIPM_vec[i-1] < 0)
            {
                slope_counter--;
            }

            if (bIPM_vec[i]-bIPM_vec[i-1] > 0)
            {
                b_counter++;
            }
            else if (bIPM_vec[i]-bIPM_vec[i-1] < 0)
            {
                b_counter--;
            }
        }
    }
//    std::cout << "- end isCurve -" << std::endl;
//    std::cout << "STATS! - slope_counter: " << slope_counter << " - b_counter: " << b_counter << " - number_of_segments: " << number_of_segments <<  std::endl;

    if ( (slope_counter == number_of_segments-1 || slope_counter == -(number_of_segments-1)) &&
         (b_counter == number_of_segments-1 || b_counter == -(number_of_segments-1)))
    {
        std::cout << "is a curve" << std::endl;
        return true;
    }
    else
    {
        std::cout << "is not a curve" << std::endl;
        return false;
    }
}

int tracking_utilities::test_points(std::vector<cv::Point> &line)
{
//    int line_segment_size = 10; // 120(height)/8 segments
//    int number_of_segments = line.size()/line_segment_size;
//    if (number_of_segments < 3)
//    {
//        std::cout << line.size() << " -> too small" << std::endl;
//        return 0;
//    }
    int number_of_segments = 3;
    int line_segment_size = line.size()/number_of_segments;

    double mean = 0.0, sum_deviation = 0.0, standard_deviation = 0.0;

    int slope_counter = 0, b_counter = 0;
    double mIPM, bIPM, rIPM;
    std::vector<double> mIPM_vec, bIPM_vec, rIPM_vec;
//    std::cout << "- begin test_points -" << std::endl;

    for (int i = 0; i < number_of_segments; i++)
    {
        std::vector<cv::Point2d> rightPointsInverted;
        for (int j = 0; j < line_segment_size; j++)
        {
            rightPointsInverted.push_back(cv::Point2d(line[j+i*line_segment_size].y, line[j+i*line_segment_size].x));
        }
        linreg(line_segment_size, rightPointsInverted, mIPM, bIPM, rIPM);
        mIPM_vec.push_back(mIPM);
        bIPM_vec.push_back(bIPM);
        rIPM_vec.push_back(rIPM);
        mean += bIPM;
//        std::cout << "slope " << i << ": " << mIPM << " - b: " << bIPM << std::endl;
        if (i>0)
        {
            if (mIPM_vec[i]-mIPM_vec[i-1] > 0)
            {
                slope_counter++;
            }
            else if (mIPM_vec[i]-mIPM_vec[i-1] < 0)
            {
                slope_counter--;
            }

            if (bIPM_vec[i]-bIPM_vec[i-1] > 0)
            {
                b_counter++;
            }
            else if (bIPM_vec[i]-bIPM_vec[i-1] < 0)
            {
                b_counter--;
            }
        }
    }

    mean = mean/bIPM_vec.size();
    for(int i = 0; i < bIPM_vec.size(); i++)
        sum_deviation += (bIPM_vec[i]-mean)*(bIPM_vec[i]-mean);
    standard_deviation = sqrt(sum_deviation/bIPM_vec.size());

//    std::cout << "- end test_points -" << std::endl;
//    std::cout << "STATS! - slope_counter: " << slope_counter << " - b_counter: " << b_counter << " - number_of_segments: " << number_of_segments <<
//                 " - mean: " << mean << " - standard_deviation: " << standard_deviation << std::endl;

    if ( (slope_counter == number_of_segments-1 || slope_counter == -(number_of_segments-1)) &&
         (b_counter == number_of_segments-1 || b_counter == -(number_of_segments-1)))
    {
        std::cout << "is a curve" << std::endl;
        return 1;
    }
    else if(standard_deviation < mean/3)
    {
        std::cout << "is a line" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "is neither" << std::endl;
        return -1;
    }
}

bool tracking_utilities::isDiscontinuous(std::vector<double> sloppy) // must be called before updating slope
{
    double sum = 0.0, maxDiffSlope = 3;

//    for (int i = 0; i < sloppy.size(); i++)
//       sum = sum + sloppy[i];
//    if ( (sum / sloppy.size() - sloppy.back()) > maxDiffSlope )
    std::cout << sloppy.rbegin()[1] << " " << sloppy.back() << " " << fabs(sloppy.rbegin()[1] - sloppy.back()) << std::endl;
    if ( fabs(sloppy.rbegin()[1] - sloppy.back()) > maxDiffSlope )
        return true;
    else
        return false;
}

void tracking_utilities::merge_grid_lines(std::vector<std::vector<line> > &lines, std::vector<line> &merged_lines)
{
    int grid_counter = 0, line_was_merged = 0;

    // Push back to merged lines the first line in the first grid possessing lines
    while (grid_counter < lines.size())
    {
        if (lines[grid_counter].size() > 0)
        {
            merged_lines.push_back(lines[grid_counter][0]);
            break;
        }
        else
            grid_counter++;
    }

    // Merge the remaining lines in the first grid possessing lines
    for (int j = 1; j < lines[grid_counter].size(); j++)
    {
        for (int k = 0; k < merged_lines.size(); k++)
        {
            if (merged_lines[k].isSame(lines[grid_counter][j]))
            {
                merged_lines[k].mergeParameters(lines[grid_counter][j]);
                line_was_merged = 1;
            }

            if (!line_was_merged)
                merged_lines.push_back(lines[grid_counter][j]);
            line_was_merged = 0;
        }
    }



    // Iterate remaining grids to merge lines
    for (int i = grid_counter+1; i < lines.size(); i++)
    {
        for (int j = 0; j < lines[i].size(); j++)
        {
            for (int k = 0; k < merged_lines.size(); k++)
            {
                if (merged_lines[k].isSame(lines[i][j]))
                {
                    merged_lines[k].mergeParameters(lines[i][j]);
                    line_was_merged = 1;
                }
            }
            if (!line_was_merged)
                merged_lines.push_back(lines[i][j]);
            line_was_merged = 0;
        }
    }
}

void tracking_utilities::remove_lines(std::vector<std::vector<cv::Point> > &lines, int limit)
{
    for (int k = 0; k < lines.size(); k++)
    {
        if ( (abs(lines[k].front().y-lines[k].back().y) < limit) || (lines[k].size() < limit/3) )
        {
            lines[k].erase(lines[k].begin(),lines[k].end());
            lines.erase(lines.begin()+k);
            k--;
        }
    }
}

void tracking_utilities::remove_lines(std::vector<std::vector<std::vector<cv::Point> > > &lines, int limit)
{
    for (int i = 0; i < lines.size(); i++)
    {
        for (int k = 0; k < lines[i].size(); k++)
        {
            if (lines[i][k].size()<limit)
            {
                lines[i][k].erase(lines[i][k].begin(),lines[i][k].end());
                lines[i].erase(lines[i].begin()+k);
                k--;
            }
        }
    }
}

void tracking_utilities::remove_lines(std::vector<line> &lines, int limit)
{
    for (std::vector<line>::iterator it = lines.begin(); it != lines.end();)
    {
        if (it->getNumberOfPoints() < limit)
            it = lines.erase(it);
        else
            ++it;
    }
}

void tracking_utilities::draw_points(cv::Mat img, std::vector<std::vector<cv::Point> > lines)
{
    for (int k = 0; k < lines.size(); k++)
    {
        cv::Scalar color = colorList[k];//cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
//        std::cout << "line points size " << center_lines[k].size() << std::endl;
        for (int l = 0; l < lines[k].size(); l++)
        {
            cv::circle(img, lines[k][l], 3, color, -1, 8, 0);
        }
    }
}

void tracking_utilities::draw_points(cv::Mat img, std::vector<std::vector<std::vector<cv::Point> > > &lines)
{
    for (int i = 0; i < lines.size(); i++)
    {
        cv::Scalar color;
        if(i < 9)
            color = cv::Scalar(255-(i-6)*45,0+(i-6)*25,0+(i-6)*15);
        if(i < 6)
            color = cv::Scalar(0+(i-3)*25,255-(i-3)*45,0+(i-3)*15);
        if(i < 3)
            color = cv::Scalar(0+i*15,0+i*25,255-i*45);

        for (int k = 0; k < lines[i].size(); k++)
        {
            for (int l = 0; l < lines[i][k].size(); l++)
            {
                cv::circle(img, lines[i][k][l], 3, color, -1, 8, 0);
            }
        }
    }
}

void tracking_utilities::estimate_lines(std::vector<std::vector<cv::Point> > lines, std::vector<std::vector<double> > &line_info, cv::Mat debug_image)
{
    /// Estimate lines using and calculate distance/orientation
    std::vector<double> distances_to_line, angles_to_line, m_IPM, b_IPM;
//    std::vector < std::vector<double> > line_info;
    double mIPM, bIPM, rIPM;
    double distIPM, angleIPM;
    for (int i = 0; i < lines.size(); i++)
    {
        std::vector<cv::Point2d> rightPointsInverted;
        for(int j = 0; j < lines[i].size(); j++)
        {
            rightPointsInverted.push_back(cv::Point2d(lines[i][j].y, lines[i][j].x));
        }
        linreg(lines[i].size(), rightPointsInverted, mIPM, bIPM, rIPM);
        cv::Point p1, p2;
        p1.y = 0;
        p1.x = mIPM * p1.y + bIPM;
        p2.y = debug_image.rows - 1;
        p2.x = mIPM * p2.y + bIPM;

        cv::line(debug_image, p1, p2, cv::Scalar(0,0,255), 2);
        cv::line(debug_image, cv::Point(0, IPM_POINT_Y), cv::Point(debug_image.cols - 1, IPM_POINT_Y), cv::Scalar(0,100,0), 2);
        cv::line(debug_image, cv::Point(IPM_POINT_X, 0), cv::Point(IPM_POINT_X, debug_image.rows - 1), cv::Scalar(0,100,0), 2);

        m_IPM.push_back(mIPM);
        b_IPM.push_back(bIPM);
        distIPM = ((mIPM*IPM_POINT_Y+bIPM) - IPM_POINT_X) * (4.0/160.0);
        angleIPM = atan2(mIPM, 1);
        distances_to_line.push_back(distIPM);
        angles_to_line.push_back(angleIPM);
        line_info.push_back(std::vector<double>());
        line_info[line_info.size()-1].push_back(distIPM);
        line_info[line_info.size()-1].push_back(angleIPM);
        line_info[line_info.size()-1].push_back(mIPM);
        line_info[line_info.size()-1].push_back(bIPM);
    }
}

void tracking_utilities::estimate_lines(std::vector<std::vector<cv::Point> > new_lines, std::vector<line> &stored_line_info, cv::Mat debug_image)
{
    /// Estimate lines using and calculate distance/orientation
    std::vector<double> distances_to_line, angles_to_line, m_IPM, b_IPM;
    double mIPM, bIPM, rIPM;
//    double mIPM_ransac = 0, bIPM_ransac = 0, rIPM_ransac;
    double distIPM, angleIPM;

    for (int i = 0; i < new_lines.size(); i++)
    {
        line temp_line;
        temp_line.calculateDispersion(new_lines[i]);

        std::vector<cv::Point2d> rightPointsInverted;
//        cv::Mat mat_points = cv::Mat::zeros(2,new_lines[i].size(),CV_64F);
        for(int j = 0; j < new_lines[i].size(); j++)
        {
            rightPointsInverted.push_back(cv::Point2d(new_lines[i][j].y, new_lines[i][j].x));
//            mat_points.at<double>(0,j) = new_lines[i][j].y;
//            mat_points.at<double>(1,j) = new_lines[i][j].x;
        }

        linreg(new_lines[i].size(), rightPointsInverted, mIPM, bIPM, rIPM);

//        linRANSAC(mat_points, mIPM_ransac, bIPM_ransac);
//        std::cout << "\nm = " << mIPM << " b = " << bIPM <<  std::endl;
//        std::cout << "m = " << mIPM_ransac << " b = " << bIPM_ransac <<  std::endl;
//        mIPM = mIPM_ransac;
//        bIPM = bIPM_ransac;

        cv::Point p1, p2;
        p1.y = 0;
        p1.x = mIPM * p1.y + bIPM;
        p2.y = debug_image.rows - 1;
        p2.x = mIPM * p2.y + bIPM;

        cv::line(debug_image, p1, p2, cv::Scalar(0,0,255), 2);

        m_IPM.push_back(mIPM);
        b_IPM.push_back(bIPM);
        distIPM = ((mIPM*IPM_POINT_Y+bIPM) - IPM_POINT_X) * (4.0/160.0);
        angleIPM = atan2(mIPM, 1);
        distances_to_line.push_back(distIPM);
        angles_to_line.push_back(angleIPM);

        temp_line.updateSlopeM(mIPM);
        temp_line.updateCrossPointB(bIPM);
        temp_line.updateDistanceToOrigin(distIPM);
        temp_line.updateAngleToOrigin(angleIPM);
        temp_line.updateNumberOfPoints(new_lines[i].size());
        stored_line_info.push_back(temp_line);
    }
    cv::line(debug_image, cv::Point(0, IPM_POINT_Y), cv::Point(debug_image.cols - 1, IPM_POINT_Y), cv::Scalar(0,100,0), 2);
    cv::line(debug_image, cv::Point(IPM_POINT_X, 0), cv::Point(IPM_POINT_X, debug_image.rows - 1), cv::Scalar(0,100,0), 2);
}

void tracking_utilities::estimate_lines_and_curves(std::vector<std::vector<cv::Point> > new_lines, std::vector<line> &stored_line_info, cv::Mat debug_image)
{
    /// Estimate lines using and calculate distance/orientation
    std::vector<double> distances_to_line, angles_to_line, m_IPM, b_IPM;
    double mIPM, bIPM, rIPM;
    double distIPM, angleIPM;
    int line_result;
    cv::Scalar color;

    for (int i = 0; i < new_lines.size(); i++)
    {
        line temp_line;
        temp_line.calculateDispersion(new_lines[i]);

        std::vector<cv::Point2d> rightPointsInverted;
        line_result = test_points(new_lines[i]);
        if (line_result == 1) //is a curve
        {
            //std::cout << "is Curve" << std::endl;
            color = cv::Scalar(0, 255, 255);
            for(int j = 0; j < 20; j++)
            {
                rightPointsInverted.push_back(cv::Point2d(new_lines[i][j].y, new_lines[i][j].x));
            }
            linreg(20, rightPointsInverted, mIPM, bIPM, rIPM);
        }
        else if (line_result == 0) //is a line
        {
            //std::cout << "is Line" << std::endl;
            color = cv::Scalar(255, 0, 255);
            for(int j = 0; j < new_lines[i].size(); j++)
            {
                rightPointsInverted.push_back(cv::Point2d(new_lines[i][j].y, new_lines[i][j].x));
            }
            linreg(new_lines[i].size(), rightPointsInverted, mIPM, bIPM, rIPM);
        }
        else // disregard this line
            continue;
        cv::Point p1, p2;
        p1.y = 0;
        p1.x = mIPM * p1.y + bIPM;
        p2.y = debug_image.rows - 1;
        p2.x = mIPM * p2.y + bIPM;

        cv::line(debug_image, p1, p2, color, 2);

        m_IPM.push_back(mIPM);
        b_IPM.push_back(bIPM);
        distIPM = ((mIPM*IPM_POINT_Y+bIPM) - IPM_POINT_X) * (4.0/160.0);
        angleIPM = atan2(mIPM, 1);
        distances_to_line.push_back(distIPM);
        angles_to_line.push_back(angleIPM);

        temp_line.updateSlopeM(mIPM);
        temp_line.updateCrossPointB(bIPM);
        temp_line.updateDistanceToOrigin(distIPM);
        temp_line.updateAngleToOrigin(angleIPM);
        temp_line.updateNumberOfPoints(new_lines[i].size());
        stored_line_info.push_back(temp_line);
    }

    cv::line(debug_image, cv::Point(0, IPM_POINT_Y), cv::Point(debug_image.cols - 1, IPM_POINT_Y), cv::Scalar(0,100,0), 2);
    cv::line(debug_image, cv::Point(IPM_POINT_X, 0), cv::Point(IPM_POINT_X, debug_image.rows - 1), cv::Scalar(0,100,0), 2);
}

void tracking_utilities::update_lines(std::vector<line> &stored_line_info, std::vector<line> &new_line_info)
{
    for (int i = 0; i < new_line_info.size(); i++ )
    {
        int line_updated = 0;
        if (stored_line_info.size() < 1)
            stored_line_info.push_back(new_line_info[i]);
        else
        {
            for (int k = 0; k < stored_line_info.size(); k++ )
            {
                if (stored_line_info[k].isSame(new_line_info[i]))
                {
//                        std::cout << "line " << k << " updated" << std::endl;
//                        std::cout << "Old params: ";
//                        stored_line_info[k].printData();
//                        std::cout << "New params: ";
//                        new_line_info[i].printData();
//                    stored_line_info[k].updateParameters(new_line_info[i]);
                    stored_line_info[k].mergeParameters(new_line_info[i]);
                    line_updated = 1;
                    break;
                }
            }

            if (0 == line_updated)
            {
//                    std::cout << "new line added" << std::endl;
                stored_line_info.push_back(new_line_info[i]);
            }
        }
        //std::cout << i << " " << line_info[i][0] << " " << line_info[i][1]*180.0/3.1415 << " " << line_info[i][2] << std::endl;
    }

    if (stored_line_info.size() > 0)
    {
        for (int i = 0; i < stored_line_info.size()-1; i++ )
        {
//            std::cout << "i: " << i << " j: ";
            for (int j = i+1; j < stored_line_info.size(); j++ )
            {
//                std::cout << j << " ";
                if (stored_line_info[i].isSame(stored_line_info[j]))
                {
                    stored_line_info[i].mergeParameters(stored_line_info[j]);
                    stored_line_info.erase(stored_line_info.begin()+j);
//                    std::cout << " Erased - ";
                    i--;
                    break;
                }
            }
//            std::cout << std::endl;
        }
    }

    for (std::vector<line>::iterator it = stored_line_info.begin(); it != stored_line_info.end();)
    {
        if (stored_line_info.size() < 1)
            break;
        it->addAge();
        if (it->isValid())
        {
//            std::cout << "line " << std::distance( stored_line_info.begin(), it ) << ": ";
//            it->printData();
//            cv::Point p1, p2;
//            p1.y = 0;
//            p1.x = it->getSlopeM() * p1.y + it->getCrossPointB();
//            p2.y = color_debug_Peter.rows - 1;
//            p2.x = it->getSlopeM() * p2.y + it->getCrossPointB();
//            cv::line(color_debug_Peter, p1, p2, cv::Scalar(255,0,0), 2);
            ++it;
        }
        else
        {
            it = stored_line_info.erase(it);
//            std::cout << "Erased line. New size: " << stored_line_info.size() << std::endl;
        }
    }
//    std::cout << "Number of lines tracked: " << stored_line_info.size() << std::endl;
}

void tracking_utilities::estimate_lines_in_grid(std::vector<std::vector<std::vector<cv::Point> > > lines, std::vector<std::vector<line> > &line_info, cv::Mat debug_image)
{
    /// Estimate lines using and calculate distance/orientation
    std::vector<double> distances_to_line, angles_to_line, m_IPM, b_IPM;
    double mIPM, bIPM, rIPM;
    double distIPM, angleIPM;

    for(int k = 0; k < lines.size(); k++)
    {
        line_info.push_back(std::vector<line>());
        for (int i = 0; i < lines[k].size(); i++)
        {
            line temp_line;// Peter
            temp_line.calculateDispersion(lines[k][i]);

            std::vector<cv::Point2d> rightPointsInverted;
            if (isCurve(lines[k][i]))
            {
                //std::cout << "is Curve" << std::endl;
                for(int j = 0; j < 15; j++) // 120(height)/8 segments
                {
                    rightPointsInverted.push_back(cv::Point2d(lines[k][i][j].y, lines[k][i][j].x));
                }
            }
            else
            {
                //std::cout << "is Line" << std::endl;
                for(int j = 0; j < lines[k][i].size(); j++)
                {
                    rightPointsInverted.push_back(cv::Point2d(lines[k][i][j].y, lines[k][i][j].x));
                }
            }
            linreg(lines[k][i].size(), rightPointsInverted, mIPM, bIPM, rIPM);
            cv::Point p1, p2;
            p1.y = 0;
            p1.x = mIPM * p1.y + bIPM;
            p2.y = debug_image.rows - 1;
            p2.x = mIPM * p2.y + bIPM;

//            cv::line(debug_image, p1, p2, cv::Scalar(0,0,255), 2);
//            cv::line(debug_image, cv::Point(0, IPM_POINT_Y), cv::Point(debug_image.cols - 1, IPM_POINT_Y), cv::Scalar(0,100,0), 2);
//            cv::line(debug_image, cv::Point(IPM_POINT_X, 0), cv::Point(IPM_POINT_X, debug_image.rows - 1), cv::Scalar(0,100,0), 2);

            m_IPM.push_back(mIPM);
            b_IPM.push_back(bIPM);
            distIPM = ((mIPM*IPM_POINT_Y+bIPM) - IPM_POINT_X) * (4.0/160.0);
            angleIPM = atan2(mIPM, 1);
            distances_to_line.push_back(distIPM);
            angles_to_line.push_back(angleIPM);

            temp_line.updateSlopeM(mIPM);
            temp_line.updateCrossPointB(bIPM);
            temp_line.updateDistanceToOrigin(distIPM);
            temp_line.updateAngleToOrigin(angleIPM);
            temp_line.updateNumberOfPoints(lines[k][i].size());
            line_info[k].push_back(temp_line);
        }
    }
}

void tracking_utilities::skeletonize(cv::Mat src, cv::Mat &skel)
{
    cv::Mat temp;
    cv::Mat eroded;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

    bool done;
    do
    {
        cv::erode(src, eroded, element);
        cv::dilate(eroded, temp, element); // temp = open(img)
        cv::subtract(src, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(src);

        done = (cv::countNonZero(src) == 0);
    } while (!done);
}

void tracking_utilities::calculateRightLineBasedOnLeftLine(double left_m, double left_b, double &new_distIPM, double &new_angleIPM, cv::Mat debug)
{
    cv::Point p3, p4;
    /// Calculate Right line based on left line
    // Draw robot's referential
//    cv::line(debug, cv::Point(0, IPM_POINT_Y), cv::Point(debug.cols - 1, IPM_POINT_Y), cv::Scalar(0,100,0), 2);
//    cv::line(debug, cv::Point(IPM_POINT_X, 0), cv::Point(IPM_POINT_X, debug.rows - 1), cv::Scalar(0,100,0), 2);

    int offset = 29;
    p3.y = 0;
    p3.x = left_m * p3.y + left_b+offset;
    p4.y = debug.rows - 1;
    p4.x = left_m * p4.y + left_b+offset;

    // Draw left line
//    cv::line(debug, cv::Point(IPM_POINT_X, IPM_POINT_Y), cv::Point(left_m*IPM_POINT_Y+left_b, IPM_POINT_Y), cv::Scalar(0,255,0), 2);

    new_distIPM = ((left_m*IPM_POINT_Y+left_b+offset) - IPM_POINT_X) * (4.0/160.0);
    new_angleIPM = atan2(left_m, 1);
//    std::cout << "New dist: " << new_distIPM << " New Angle: " << new_angleIPM*180/3.1415 << std::endl;

//     Draw distance to new right line
    cv::line(debug, cv::Point(IPM_POINT_X, IPM_POINT_Y), cv::Point(left_m*IPM_POINT_Y+left_b+offset, IPM_POINT_Y), cv::Scalar(0,255,0), 2);
//     Draw new right line
    cv::line(debug, p3, p4, cv::Scalar(0,127,0), 2);
}

//void tracking_utilities::calculateRightLineBasedOnLeftLine(line left_line, line &right_line, cv::Mat debug)
//{
//    cv::Point p3, p4;
//    /// Calculate Right line based on left line
//    // Draw robot's referential
//    cv::line(debug, cv::Point(0, IPM_POINT_Y), cv::Point(debug.cols - 1, IPM_POINT_Y), cv::Scalar(0,100,0), 2);
//    cv::line(debug, cv::Point(IPM_POINT_X, 0), cv::Point(IPM_POINT_X, debug.rows - 1), cv::Scalar(0,100,0), 2);

//    int offset = 31;
//    p3.y = 0;
//    p3.x = left_m * p3.y + left_b+offset;
//    p4.y = debug.rows - 1;
//    p4.x = left_m * p4.y + left_b+offset;

//    // Draw left line
//    cv::line(debug, cv::Point(IPM_POINT_X, IPM_POINT_Y), cv::Point(left_m*100+left_b, 100), cv::Scalar(0,255,0), 2);

//    new_distIPM = ((left_m*IPM_POINT_Y+left_b+offset) - IPM_POINT_X) * (4.0/160.0);
//    new_angleIPM = atan2(left_m, 1);
//    cv::line(debug, p3, p4, cv::Scalar(127,0,0), 2);
//    std::cout << "New dist: " << new_distIPM << " New Angle: " << new_angleIPM*180/3.1415 << std::endl;
//    // Draw distance to new right line
//    cv::line(debug, cv::Point(IPM_POINT_X, IPM_POINT_Y), cv::Point(left_m*100+left_b+offset, 100), cv::Scalar(255,0,0), 2);
//}

double tracking_utilities::vector_avg(std::vector<double> &data)
{
        double return_value = 0.0;
        double n = data.size();
        for ( int i=0; i < n; i++)
            return_value += data[i];
        return ( return_value / n);
}
double tracking_utilities::vector_max( std::vector<double> &data )
{
    if(data.size() <= 0)
        return 0;
    double max = data[0];
    for(int i = 1; i < data.size(); i ++)
        if (data[i] > max)
            max = data[i];
    return max;
}
///--------------------------------------------------------------------------------------------------
double tracking_utilities::sqr(double x)
{
    return x*x;
}

bool tracking_utilities::linreg(int n, std::vector<cv::Point2d> points, double & m, double & b, double & r)
{
    double   sumx = 0.0;                        /* sum of x                      */
    double   sumx2 = 0.0;                       /* sum of x**2                   */
    double   sumxy = 0.0;                       /* sum of x * y                  */
    double   sumy = 0.0;                        /* sum of y                      */
    double   sumy2 = 0.0;                       /* sum of y**2                   */

    for (int i=0;i<n;i++)
    {
        sumx  += points[i].x;
        sumx2 += sqr(points[i].x);
        sumxy += points[i].x * points[i].y;
        sumy  += points[i].y;
        sumy2 += sqr(points[i].y);
    }

   double denom = (n * sumx2 - sqr(sumx));
   if (denom == 0) {
       // singular matrix. can't solve the problem.
       m = 0;
       b = 0;
       if (r) r = 0;
       return false;
   }
   m = (n * sumxy  -  sumx * sumy) / denom;
   b = (sumy * sumx2  -  sumx * sumxy) / denom;
//   if (r!=NULL) {
      r = (sumxy - sumx * sumy / n) /          /* compute correlation coeff     */
            sqrt((sumx2 - sqr(sumx)/n) *
            (sumy2 - sqr(sumy)/n));
//   }
      return true;
}

bool tracking_utilities::linRANSAC(cv::Mat data_points, double &m, double &b)
{

    int num=2;
    int iter=100;
    double threshDist=1.2;
    double inlierRatio=0.05;

    srand (time(NULL));

    int number = data_points.cols;
    int bestInNum = 0;
    double bestParameter1 = 0;
    double bestParameter2 = 0;

    int first_index = -1;
    int second_index = -1;

    cv::Mat sample = (cv::Mat_<double>(2,2) << \
                      0,0,\
                      0,0);

    cv::Mat repmat = cv::Mat::zeros(num,number,CV_64F);
    //            (cv::Mat_<double>(num,number));

    for (int i = 0; i < iter; i++)
    {
        first_index = rand()%number;
        do
        {
            second_index = rand()%number;
        }while (second_index==first_index);
//        std::cout << first_index << " " << second_index << std::endl;

        sample.at<double>(0,0) = data_points.at<double>(0,first_index);
        sample.at<double>(1,0) = data_points.at<double>(1,first_index);
        sample.at<double>(0,1) = data_points.at<double>(0,second_index);
        sample.at<double>(1,1) = data_points.at<double>(1,second_index);

//        std::cout << sample << std::endl;

        cv::Mat kLine = sample.col(1)-sample.col(0);
        cv::Mat kLineNorm = kLine/cv::norm(kLine);
        cv::Mat normVector = (cv::Mat_<double>(1,2)<< -kLineNorm.at<double>(1,0), kLineNorm.at<double>(0,0));
//        std::cout << normVector << std::endl;
        for (int j = 0; j < number; j++)
            sample.col(0).copyTo(repmat.col(j));

//        std::cout<<repmat<<std::endl;
        cv::Mat distance = normVector*(data_points-repmat);
//        std::cout << distance << std::endl;
        distance = cv::abs(distance);
//        std::cout << distance << std::endl;
        int inlierNum = 0;
        for (int j = 0; j < distance.cols; j++)
        {
            inlierNum += (distance.col(j).at<double>(0))<=threshDist;
        }
        if (inlierNum>=round(inlierRatio*number) && inlierNum>bestInNum)
        {
            bestInNum = inlierNum;
            double parameter1 = (sample.at<double>(1,1)-sample.at<double>(1,0))/(sample.at<double>(0,1)-sample.at<double>(0,0));
            double parameter2 = sample.at<double>(1,0)-parameter1*sample.at<double>(0,0);
            bestParameter1 = parameter1;//m
            bestParameter2 = parameter2;//b
        }
    }
    m = bestParameter1;
    b = bestParameter2;
    return true;
}

