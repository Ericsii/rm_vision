// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef RUNE_DETECTOR__DETECTOR_HPP_
#define RUNE_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <utility>
#include <vector>

#include "rm_nahsor/rune.hpp"

#include "rclcpp/rclcpp.hpp"

#include "rm_util/calc_tool.hpp"
namespace rm_power_rune
{
class Circle
{

public:
    cv::Point2f R_center;
    double radius;
    
};

class Detector
{
public:

  typedef struct armor_point{
      cv::Point2f point;
      double dis;
  }armor_point;

  Detector(rclcpp::Node::SharedPtr node);
  
  cv::Mat PreProcess(const cv::Mat & src);

  bool New_detect(const cv::Mat & src);

  void drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness);
  
  double distance(cv::Point a,cv::Point b);

  bool getArmorDescriptor(cv::RotatedRect rect, Rune &armor_);

  double calc_dis(cv::Point2f &point1, cv::Point2f &point2);
  /**
   * @brief 计算两点角度
   *
   * @param point1 第一个点
   * @param point2 第二个点
   * @return double 角度
   */
  double calc_angle(cv::Point2f &point1, cv::Point2f &point2);
  /**
   * @brief Get the armors object
   *
   * @return std::vector<Rune>
   */
  Rune get_armors();
  /**
   * @brief Get the circle object
   *
   * @return Circle
   */
  Circle get_circle();

    enum class Color {
      RED,
      BLUE,
    } detect_color;

  void set_color(bool is_red);

  int bin_thresh;

  std::array<cv::Point2f, 4> sorted_pts;

private:
  rclcpp::Node::SharedPtr node_;

  cv::Mat src;
  cv::Mat bin;
  cv::Mat floodfilled;
  cv::Point2f center;

  cv::Mat bin_;
  cv::Mat floodfilled_;

  Rune armor_;
  cv::Point2f center_;
  Circle circle;

  bool USE_HSV_;
  std::string color_;

  std::vector<int> low_red = {156, 100, 130};
  std::vector<int> up_red = {180, 255, 255};
  std::vector<int> low_red2 = {0, 100, 130};
  std::vector<int> up_red2 = {34, 255, 255};
  std::vector<int> low_blue = {80, 150, 180};
  std::vector<int> up_blue = {124, 255, 255};
};

}  // namespace rm_power_rune

#endif  // RUNE_DETECTOR__DETECTOR_HPP_
