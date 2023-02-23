// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{
class Detector
{
public:
  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
  };
  struct ArmorParams
  {
    double min_light_ratio;
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  Detector(
    const int & init_min_l, const int & init_color, const LightParams & init_l,
    const ArmorParams & init_a);

  int min_lightness;
  int detect_color;
  LightParams l;
  ArmorParams a;

  // Debug msgs
  auto_aim_interfaces::msg::DebugLights debug_lights;
  auto_aim_interfaces::msg::DebugArmors debug_armors;

  cv::Mat preprocessImage(const cv::Mat & rbg_img);

  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);

  std::vector<Armor> matchLights(const std::vector<Light> & lights);

private:
  bool isLight(const Light & light);

  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);

  bool isArmor(Armor & armor);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
