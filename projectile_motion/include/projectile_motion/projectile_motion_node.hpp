// Copyright 2023 Yunlong Feng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_
#define PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <auto_aim_interfaces/msg/target.hpp>
#include <rmoss_projectile_motion/gimbal_transform_tool.hpp>
#include <rmoss_projectile_motion/projectile_solver_interface.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <rm_interfaces/msg/robot_shoot_data.hpp>

namespace projectile_motion
{

class ProjectileMotionNode : public rclcpp::Node
{
public:
  using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>;

public:
  explicit ProjectileMotionNode(rclcpp::NodeOptions options);

private:
  void target_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
  void shoot_data_callback(const rm_interfaces::msg::RobotShootData::SharedPtr msg);

private:
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double offset_pitch_;
  double offset_yaw_;
  double offset_time_;
  double shoot_speed_;
  double friction_{0.001};

  std::string target_topic_;
  std::string gimbal_cmd_topic_;
  std::string shoot_data_topic_;
  std::string solver_type_;

  std::shared_ptr<rmoss_projectile_motion::ProjectileSolverInterface> solver_;
  rclcpp::Subscription<rm_interfaces::msg::RobotShootData>::SharedPtr shoot_data_subscriber_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_subscriber_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_publisher_;
};

}  // namespace projectile_motion

#endif  // PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_
