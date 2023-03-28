#include "rune_detector/detector.hpp"

#include <array>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace rm_power_rune
{
Detector::Detector(rclcpp::Node::SharedPtr node): node_(node){};

cv::Mat Detector::PreProcess(const cv::Mat & src)
{
    cv::Mat src_frame;
    src_frame = src;
    cv::Mat origin_frame = src_frame;
    // cv::resize(origin_frame,origin_frame,cv::Size(640,480));
    // cv::imshow("origin", origin_frame);
    // cv::waitKey(1);

    cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::dilate(src_frame, src_frame, dilate_kernel);
    cv::erode(src_frame, src_frame, erode_kernel);

    cv::resize(src_frame,src_frame,cv::Size(640,480));
    // cv::Mat bin_new = Detector::binarize(src_frame);
    // cv::resize(bin_new,bin_new,cv::Size(640,480));
    // cv::imshow("src_new", bin_new);
    // cv::waitKey(1);

    std::vector<cv::Mat> bgr_images;
    cv::split(src_frame,bgr_images);
    cv::Mat b_src_img = bgr_images[0];
    cv::blur(b_src_img,b_src_img,cv::Size(5,5));
    
    cv::Mat threshold_img;
    cv::threshold(b_src_img,threshold_img,130,255,cv::THRESH_BINARY);
    return threshold_img;
}



bool Detector::New_detect(const cv::Mat & src)
{
    cv::Mat src_frame;
    src_frame = src;
    cv::resize(src_frame,src_frame,cv::Size(640,480));
//   cv::Mat origin_frame = src_frame;
//   // cv::resize(origin_frame,origin_frame,cv::Size(640,480));
//   // cv::imshow("origin", origin_frame);
//   // cv::waitKey(1);

//   cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
//   cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
//   cv::dilate(src_frame, src_frame, dilate_kernel);
//   cv::erode(src_frame, src_frame, erode_kernel);

//   cv::resize(src_frame,src_frame,cv::Size(640,480));
//   // cv::Mat bin_new = Detector::binarize(src_frame);
//   // cv::resize(bin_new,bin_new,cv::Size(640,480));
//   // cv::imshow("src_new", bin_new);
//   // cv::waitKey(1);

//   std::vector<cv::Mat> bgr_images;
//   cv::split(src_frame,bgr_images);
//   cv::Mat b_src_img = bgr_images[0];
//   cv::blur(b_src_img,b_src_img,cv::Size(5,5));
    
//   cv::Mat threshold_img;
//   cv::threshold(b_src_img,threshold_img,130,255,cv::THRESH_BINARY);
    cv::Mat preprocessedimg = Detector::PreProcess(src);
    cv::imshow("pre_process", preprocessedimg);
    cv::waitKey(1);




    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(preprocessedimg,contours,cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    // sort(contours.begin(),contours.end(),[](std::vector<cv::Point> a,std::vector<cv::Point> b){return cv::contourArea(a) < cv::contourArea(b);}); //排序来排除另一种错误识别情况
    // cv::findContours(bin_new,contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(src_frame,contours,-1,cv::Scalar(0,0,255));
    
    cv::Point r_center; // R 的中心点
    std::vector<cv::RotatedRect> contours_min_rects;//所有轮廓的最小外接矩形
    std::vector<cv::RotatedRect> armor_min_rects;
    std::vector<cv::RotatedRect> target_min_rects;
    std::vector<cv::RotatedRect> r_min_rects; //R

    int r_index = -1;
    // int cnt = 0;
    for (unsigned int contour_index = 0; contour_index < contours.size(); contour_index++) {

        //寻找最小旋转矩形,放进容器，顺便将面积过大的剔除,长宽比悬殊的剔除
        cv::RotatedRect minrect = minAreaRect(contours[contour_index]);
        cv::Rect rect = boundingRect(contours[contour_index]);
        // Detector::drawRotatedRect(src_frame, minrect, cv::Scalar(0,0,255),3);

        if (minrect.size.area() <= 15000.0 && minrect.size.area() > 100) {
            float width;
            float height;
            float tmp;
            //判断长宽比
            if (minrect.size.width > minrect.size.height){
                width = minrect.size.width;
                height = minrect.size.height;


            } else {
                width = minrect.size.height;
                height = minrect.size.width;

            }
            if (width / height < 5) {
                contours_min_rects.push_back(minrect);
                if(minrect.size.area() < 1000 && minrect.center.y > 80 && minrect.center.x >80) { // find R
                    if(height / width > 0.80){
                        // R_minRects.push_back(minrect);

                        r_center = minrect.center;
                        
                        circle.R_center = r_center;
                        // RCLCPP_INFO(node_->get_logger(), "R_Center_Area:%f", minrect.size.area());
                        cv::circle(src_frame,minrect.center,15,cv::Scalar(5,255,100));
                        r_index = contour_index;
                        // std::cout<<cnt++<<std::endl;    
                    }
                } 
                else {
                    if(minrect.size.area() > 1000 && minrect.size.area() < 18000 && (height / width) < 0.9) {
                        armor_min_rects.push_back(minrect);
                    }     
                }
            }   
        }
    }

    // if(armor_min_rects.size())
    // for(int i = 0; i<armor_min_rects.size(); i++)
    // Detector::drawRotatedRect(src_frame, armor_min_rects[i], cv::Scalar(0,0,255),3);


    // RCLCPP_INFO(node_->get_logger(), "armor_vector_size:%d", armor_min_rects.size());
    bool find_ok = false;
    for (int i = 0;i<armor_min_rects.size()-1;i++){
        for (int j = armor_min_rects.size()-1;j>i;j--){
            double dis = Detector::distance(armor_min_rects[i].center,armor_min_rects[j].center);

            double min_height = std::min(armor_min_rects[i].size.height, armor_min_rects[j].size.height);
            double max_height = std::max(armor_min_rects[i].size.height, armor_min_rects[j].size.height);

            double min_width = std::min(armor_min_rects[i].size.width, armor_min_rects[j].size.width);
            double max_width = std::max(armor_min_rects[i].size.width, armor_min_rects[j].size.width);

            double min_size = std::min(min_height, min_width);
            double max_size = std::max(max_height, max_width);
            double ratio_low = dis/min_size;
            double ratio_upper = dis/max_size;
            // bool judge_dis = 0;

            if((ratio_low > 1) && (ratio_upper <1)){
            double s1 = armor_min_rects[i].size.area();
            double s2 = armor_min_rects[j].size.area();
            double rat1 = s1/s2;
            double rat2 = s2/s1;
            if((rat1 > 2 && rat1 < 4.2) ||(rat2 > 2 && rat2 < 4.2)){
                target_min_rects.push_back(armor_min_rects[i]);
                target_min_rects.push_back(armor_min_rects[j]);
                find_ok = true;
                break;
            }
            }

            // std::cout<<dis<<std::endl;
        }   
        if (find_ok){
            break;
        }    
    }
    if(target_min_rects.size() != 2){
        return false;
        RCLCPP_WARN(node_->get_logger(), "Detect Error");
    } else {
        cv::RotatedRect rrect_in; //= target_minRects[0];
        cv::RotatedRect rrect_out;// = target_minRects[1];
        double dis1 = Detector::distance(circle.R_center,target_min_rects[0].center);
        double dis2 = Detector::distance(circle.R_center,target_min_rects[1].center);
        if (dis1 > dis2){
            rrect_in = target_min_rects[1];
            rrect_out = target_min_rects[0];
        } else {
            rrect_in = target_min_rects[0];
            rrect_out = target_min_rects[1];
        }
        Detector::drawRotatedRect(src_frame,rrect_in,cv::Scalar(0,250,0),1);
        Detector::drawRotatedRect(src_frame,rrect_out,cv::Scalar(0,0,255),1);

        cv::Point2f target_center = cv::Point2f((float)((rrect_in.center.x + rrect_out.center.x)/2),(float)((rrect_in.center.y + rrect_out.center.y)/2));
        
        cv::Point2f in_vet_points[4];
        cv::Point2f out_vet_points[4];
        rrect_in.points(in_vet_points);
        rrect_out.points(out_vet_points);
        std::vector<armor_point> armor_points; 
        for(int i = 0;i<4;i++){
            armor_point point;
            point.point = in_vet_points[i];
            point.dis = Detector::distance(target_center,in_vet_points[i]);
            armor_points.push_back(point);
        }
        for(int i = 0;i<4;i++){
            armor_point point;
            point.point = out_vet_points[i];
            point.dis = Detector::distance(target_center,out_vet_points[i]);
            armor_points.push_back(point);
        }

        sort(armor_points.begin(),armor_points.end(),[](armor_point a,armor_point b){return a.dis < b.dis;});
        std::vector<cv::Point2f> rect_points;
        
        cv::Point2f new_target_center = {0.0,0.0};

        for(int i = 0; i<4; i++){
        rect_points.push_back(armor_points[i].point);
        new_target_center += armor_points[i].point;
        }

        target_center = new_target_center/4;
        cv::circle(src_frame,target_center,5,cv::Scalar(0,0,255),-1);
        
        // cv::Point2f tmp[4];
        // for(int i=0 ; i<4;i++){
        //   tmp[i] = armor_points[i].point;
        // }

        
        // std::array<cv::Point2f, 4> test_sorted_points;
        // if (cv::norm(tmp[0] - tmp[1]) > cv::norm(tmp[1] - tmp[2])) {
        //   // 0-1 is the long side
        //   test_sorted_points = {tmp[0], tmp[1], tmp[2], tmp[3]};

        // } else {
        //   // 1-2 is the long side
        //   test_sorted_points = {tmp[1], tmp[2], tmp[3], tmp[0]};
        // }
        // cv::circle(src_frame,test_sorted_points[0],3,cv::Scalar(255,255,255),-1);
        // cv::circle(src_frame,test_sorted_points[1],3,cv::Scalar(255,0,0),-1);
        // cv::circle(src_frame,test_sorted_points[2],3,cv::Scalar(0,0,255),-1);
        
        // for(int i = 0;i<4;i++)
        // {
        //     cv::circle(src_frame,armor_points[i].point,3,cv::Scalar(255,0,255),-1);
        // }
        int buff_run_radius = (int)Detector::distance(r_center,target_center);
        if(r_center.x >60){
        circle.radius = buff_run_radius;
        }
        cv::circle(src_frame,circle.R_center,circle.radius,cv::Scalar(55,110,255),1);
        cv::RotatedRect test_rect = cv::minAreaRect(rect_points);
        bool resget = Detector::getArmorDescriptor(test_rect, armor_);

        cv::circle(src_frame,armor_.points[0],3,cv::Scalar(255,255,255),-1);
        cv::circle(src_frame,armor_.points[1],3,cv::Scalar(255,0,0),-1);
        cv::circle(src_frame,armor_.points[2],3,cv::Scalar(0,255,0),-1);
        cv::circle(src_frame,armor_.points[3],3,cv::Scalar(0,0,255),-1);

        auto armor_angle = rm_util::deg_to_rad(rm_util::calc_inclination_angle(circle.R_center, armor_.center));
        armor_.angle = armor_angle;
        // RCLCPP_INFO(node_->get_logger(), "%f",armor_angle);
    }
    // cv::resize(src_frame,src_frame,cv::Size(1350,1080));
    // writer.write(src_frame);//把图像写入视频流

    
    cv::imshow("src_img",src_frame);
    cv::waitKey(1);
    // cv::imshow("threshold_img",threshold_img);
    return true;

}

bool Detector::getArmorDescriptor(cv::RotatedRect rect, Rune &armor_)
{
    armor_.rect = rect;
    armor_.center = rect.center;
    cv::Point2f rect_points[4];
    rect.points(rect_points);
    //确定宽和高，长的边为宽
    float linelen1 = rect.size.height;
    float linelen2 = rect.size.width;
    armor_.armor_width = std::max(linelen1, linelen2);
    armor_.armor_hight = std::min(linelen1, linelen2);
    bool line1_max_flag = (linelen1 == armor_.armor_width);
    armor_.armor_area = armor_.armor_width * armor_.armor_hight;
    armor_.armor_ratio_wh = armor_.armor_width / armor_.armor_hight;

    cv::Point2f vec_long_line2d, vec_short_line2d;
    if (line1_max_flag)
    {
        vec_long_line2d = rect_points[0] - rect_points[1];
        vec_short_line2d = rect_points[2] - rect_points[1];
    }
    else
    {
        vec_long_line2d = rect_points[2] - rect_points[1];
        vec_short_line2d = rect_points[0] - rect_points[1];
    }
    float z = vec_long_line2d.x * vec_short_line2d.y -
              vec_short_line2d.x * vec_long_line2d.y;
    if (z > 0)
    {
        // 1做起点
        if (line1_max_flag)
        {
            armor_.points[0] = rect_points[1];
            armor_.points[1] = rect_points[0];
            armor_.points[2] = rect_points[3];
            armor_.points[3] = rect_points[2];
        }
        else
        {
            armor_.points[0] = rect_points[1];
            armor_.points[1] = rect_points[2];
            armor_.points[2] = rect_points[3];
            armor_.points[3] = rect_points[0];
        }
    }
    else
    {
        // 1不能做起点
        if (line1_max_flag)
        {
            // 0做起点
            armor_.points[0] = rect_points[0];
            armor_.points[1] = rect_points[1];
            armor_.points[2] = rect_points[2];
            armor_.points[3] = rect_points[3];
        }
        else
        {
            // 2做起点
            armor_.points[0] = rect_points[2];
            armor_.points[1] = rect_points[1];
            armor_.points[2] = rect_points[0];
            armor_.points[3] = rect_points[3];
        }
    }

  return true;
}



double Detector::calc_dis(cv::Point2f &point1, cv::Point2f &point2)
{
    auto dis = (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y);
    return sqrt(dis);
}

double Detector::calc_angle(cv::Point2f &point1, cv::Point2f &point2)
{
    auto rad = atan2(point2.y - point1.y, point2.x - point1.x);
    return rad;
}

Rune Detector::get_armors()
{
    return armor_;
}

Circle Detector::get_circle()
{
    return circle;
}

void Detector::drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
{
    cv::Point2f Vertex[4];
    rect.points(Vertex);
    for(int i = 0 ; i < 4 ; i++)
    {
        cv::line(img , Vertex[i] , Vertex[(i + 1) % 4] , color , thickness);
    }
}

double Detector::distance(cv::Point a,cv::Point b)
{
    return sqrt((a.x -b.x)*(a.x -b.x) + (a.y -b.y)*(a.y -b.y));
}

void Detector::set_color(bool is_red)
{
    color_ = is_red ? "red" : "blue";
}
}  // namespace rm_power_rune
