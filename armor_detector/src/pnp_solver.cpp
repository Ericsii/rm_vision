// Copyright 2022 Chen Jun

#include "armor_detector/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_auto_aim
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  double small_half_x = kSmallArmorWidth / 2.0;
  double small_half_y = kSmallArmorHeight / 2.0;
  double large_half_x = kLargeArmorWidth / 2.0;
  double large_half_y = kLargeArmorHeight / 2.0;

  // Start from bottom left in clockwise order
  small_armor_points_.emplace_back(cv::Point3f(-small_half_x, small_half_y, 0));
  small_armor_points_.emplace_back(cv::Point3f(-small_half_x, -small_half_y, 0));
  small_armor_points_.emplace_back(cv::Point3f(small_half_x, -small_half_y, 0));
  small_armor_points_.emplace_back(cv::Point3f(small_half_x, small_half_y, 0));

  large_armor_points_.emplace_back(cv::Point3f(-large_half_x, large_half_y, 0));
  large_armor_points_.emplace_back(cv::Point3f(-large_half_x, -large_half_y, 0));
  large_armor_points_.emplace_back(cv::Point3f(large_half_x, -large_half_y, 0));
  large_armor_points_.emplace_back(cv::Point3f(large_half_x, large_half_y, 0));
}

bool PnPSolver::solvePnP(const Armor & armor, geometry_msgs::msg::Point & point)
{
  std::vector<cv::Point2f> image_armor_points;

  // Fill in image points
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);

  // Solve pnp
  cv::Mat rvec, tvec;
  auto object_points = armor.armor_type == SMALL ? small_armor_points_ : large_armor_points_;
  bool success = cv::solvePnP(
    object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  if (success) {
    // Convert to geometry_msgs::msg::Point
    point.x = tvec.at<double>(0) * 0.001;
    point.y = tvec.at<double>(1) * 0.001;
    point.z = tvec.at<double>(2) * 0.001;
    return true;
  } else {
    return false;
  }
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
