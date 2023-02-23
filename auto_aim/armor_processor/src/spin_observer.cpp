// Copyright 2022 Chen Jun

#include "armor_processor/spin_observer.hpp"

#include <angles/angles.h>

#include <cmath>

namespace rm_auto_aim
{
SpinObserver::SpinObserver(
  const rclcpp::Clock::SharedPtr clock, double max_jump_angle, double max_jump_period,
  double allow_following_range)
: max_jump_angle(max_jump_angle),
  max_jump_period(max_jump_period),
  allow_following_range(allow_following_range)
{
  target_spinning_ = false;
  jump_period_ = 0.0;
  jump_count_ = 0;
  last_jump_time_ = clock->now();
  last_jump_position_ = Eigen::Vector3d(0, 0, 0);
}

void SpinObserver::update(auto_aim_interfaces::msg::Target & target_msg)
{
  rclcpp::Time current_time = target_msg.header.stamp;
  Eigen::Vector3d current_position(
    target_msg.position.x, target_msg.position.y, target_msg.position.z);

  double time_after_jumping = (current_time - last_jump_time_).seconds();

  if (time_after_jumping > max_jump_period) {
    target_spinning_ = false;
    jump_count_ = 0;
  }

  double current_yaw = 0.0;
  double yaw_diff = 0.0;
  if (target_msg.tracking) {
    current_yaw = std::atan2(current_position.y(), current_position.x());
    yaw_diff = angles::shortest_angular_distance(last_yaw_, current_yaw);

    if (std::abs(yaw_diff) > max_jump_angle) {
      jump_count_++;
      if (jump_count_ > 1 && std::signbit(yaw_diff) == std::signbit(last_jump_yaw_diff_)) {
        target_spinning_ = true;
        jump_period_ = time_after_jumping;
      }

      last_jump_time_ = current_time;
      last_jump_position_ = current_position;
      last_jump_yaw_diff_ = yaw_diff;
    }

    last_yaw_ = current_yaw;
  }

  if (target_spinning_) {
    if (time_after_jumping / jump_period_ < allow_following_range) {
      target_msg.suggest_fire = true;
    } else {
      target_msg.position.x = last_jump_position_.x();
      target_msg.position.y = last_jump_position_.y();
      target_msg.position.z = last_jump_position_.z();
      target_msg.velocity.x = 0;
      target_msg.velocity.y = 0;
      target_msg.velocity.z = 0;

      target_msg.suggest_fire = false;
    }
  } else {
    target_msg.suggest_fire = target_msg.tracking;
  }

  // Update spin_info_msg
  spin_info_msg.header = target_msg.header;
  spin_info_msg.target_spinning = target_spinning_;
  spin_info_msg.suggest_fire = target_msg.suggest_fire;
  spin_info_msg.jump_count = jump_count_;
  spin_info_msg.yaw_diff = yaw_diff;
  spin_info_msg.jump_period = jump_period_;
  spin_info_msg.time_after_jumping = time_after_jumping;
}

}  // namespace rm_auto_aim
