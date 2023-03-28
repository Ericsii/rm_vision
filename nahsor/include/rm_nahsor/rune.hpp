#ifndef NAHSOR_MODULES_RUNE_H
#define NAHSOR_MODULES_RUNE_H
#include "opencv2/opencv.hpp"
#define RM_DEBUG_MODE
class Rune
{
public:
    int id;
    cv::Point3f D3_points[4];
    cv::RotatedRect rect;
    cv::Point2f points[4];
    cv::Point2f sorted_points[4];
    cv::Point2f center;
    float armor_hight;
    float armor_width;
    float armor_area;
    float armor_ratio_wh; //宽高比
    float angle;
};
#endif