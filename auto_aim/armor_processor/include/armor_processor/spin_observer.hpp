// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__SPIN_OBSERVER_HPP_
#define ARMOR_PROCESSOR__SPIN_OBSERVER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include "auto_aim_interfaces/msg/spin_info.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
class SpinObserver
{
public:
  SpinObserver(
    const rclcpp::Clock::SharedPtr clock, double max_jump_angle, double max_jump_period,
    double allow_following_range);

  void update(auto_aim_interfaces::msg::Target & target_msg);

  double max_jump_angle;
  double max_jump_period;
  double allow_following_range;

  auto_aim_interfaces::msg::SpinInfo spin_info_msg;

private:
  bool target_spinning_;

  double jump_period_;
  int jump_count_;

  double last_yaw_;
  double last_jump_yaw_diff_;

  rclcpp::Time last_jump_time_;
  Eigen::Vector3d last_jump_position_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__SPIN_OBSERVER_HPP_
