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

#include <projectile_motion/projectile_motion_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rmoss_projectile_motion/gravity_projectile_solver.hpp>
#include <rmoss_projectile_motion/gaf_projectile_solver.hpp>

namespace projectile_motion
{

ProjectileMotionNode::ProjectileMotionNode(rclcpp::NodeOptions options)
: Node("projectile_motion_node", options.use_intra_process_comms(true))
{
  offset_x_ = this->declare_parameter("projectile.offset_x", 0.0);
  offset_y_ = this->declare_parameter("projectile.offset_y", 0.0);
  offset_z_ = this->declare_parameter("projectile.offset_z", 0.0);
  offset_pitch_ = this->declare_parameter("projectile.offset_pitch", 0.0);
  offset_yaw_ = this->declare_parameter("projectile.offset_yaw", 0.0);
  offset_time_ = this->declare_parameter("projectile.offset_time", 0.0);
  shoot_speed_ = this->declare_parameter("projectile.initial_speed", 18.0);
  target_topic_ = this->declare_parameter("projectile.target_topic", "tracker/target");
  gimbal_cmd_topic_ = this->declare_parameter("projectile.gimbal_cmd_topic", "gimbal_cmd");
  shoot_data_topic_ = this->declare_parameter("projectile.shoot_data_topic", "robot_shoot_data");
  solver_type_ = this->declare_parameter("projectile.solver_type", "gravity");

  RCLCPP_INFO(this->get_logger(), "Projectile motion solver type: %s", solver_type_.c_str());
  if (solver_type_ == "gravity") {
    solver_ = std::make_shared<rmoss_projectile_motion::GravityProjectileSolver>(shoot_speed_);
  } else if (solver_type_ == "gaf") {
    friction_ = this->declare_parameter("projectile.friction", 0.001);
    solver_ =
      std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(shoot_speed_, friction_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown solver type: %s", solver_type_.c_str());
    return;
  }

  gimbal_cmd_publisher_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
    gimbal_cmd_topic_,
    10);
  shoot_data_subscriber_ = this->create_subscription<rm_interfaces::msg::RobotShootData>(
    shoot_data_topic_, 10,
    std::bind(&ProjectileMotionNode::shoot_data_callback, this, std::placeholders::_1));

  shooter_frame_ = this->declare_parameter("projectile.target_frame", "shooter_link");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10.0));
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  target_sub_.subscribe(this, target_topic_, rclcpp::SensorDataQoS().get_rmw_qos_profile());
  tf_filter_ =
    std::make_shared<tf2_filter>(
    target_sub_, *tf_buffer_, shooter_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  tf_filter_->registerCallback(&ProjectileMotionNode::target_callback, this);

  RCLCPP_INFO(this->get_logger(), "Projectile motion node initialized.");
}

void ProjectileMotionNode::target_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  if (!msg->tracking) {
    return;
  }
  // Get current gimbal angle.
  double cur_roll, cur_pitch, cur_yaw;
  try {
    auto transfrom = tf_buffer_->lookupTransform(
      msg->header.frame_id, shooter_frame_,
      msg->header.stamp);
    tf2::Quaternion rot_q(transfrom.transform.rotation.x, transfrom.transform.rotation.y,
      transfrom.transform.rotation.z, transfrom.transform.rotation.w);
    tf2::Matrix3x3 rot_m(rot_q);

    rot_m.getRPY(cur_roll, cur_pitch, cur_yaw);
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
    return;
  }

  // Calculate the targets position at current time.
  rclcpp::Time target_time = msg->header.stamp;
  auto center_position =
    Eigen::Vector3d(
    msg->position.x + offset_x_, msg->position.y + offset_y_,
    msg->position.z + offset_z_);
  auto center_velocity = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  // Calculate each target position at current time & predict time.
  double min_yaw = DBL_MAX, min_dis = DBL_MAX;
  double hit_yaw, hit_pitch;
  bool is_current_pair = true;
  double r = 0., target_dz = 0., fly_time = 0.;
  double target_pitch, target_yaw;
  Eigen::Vector3d target_position, target_predict_position;
  for (int i = 0; i < msg->armors_num; ++i) {
    double tmp_yaw = msg->yaw + i * (2 * M_PI / msg->armors_num);
    if (msg->armors_num == 4) {
      r = is_current_pair ? msg->radius_1 : msg->radius_2;
      is_current_pair = !is_current_pair;
      target_dz = is_current_pair ? 0. : msg->dz;
    } else {
      r = msg->radius_1;
      target_dz = 0.;
    }
    target_position = center_position + Eigen::Vector3d(
      -r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw),
      target_dz);

    // Use distance to calculate the time offset. (Approximate)
    fly_time = target_position.head(2).norm() / shoot_speed_ + offset_time_;
    tmp_yaw = tmp_yaw + msg->v_yaw * fly_time;
    target_predict_position = center_position + center_velocity * fly_time +
      Eigen::Vector3d(
      -r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw),
      target_dz);

    solver_->solve(
      target_predict_position.head(2).norm(), target_predict_position.z(),
      target_pitch);
    target_pitch = -target_pitch;  // Right-handed system
    target_yaw = std::atan2(target_predict_position.y(), target_predict_position.x());

    // Choose the target with minimum yaw error.
    if (::abs(
        ::fmod(
          tmp_yaw,
          M_PI) - cur_yaw) < min_yaw && target_predict_position.head(2).norm() < min_dis)
    {
      min_yaw = ::abs(::fmod(tmp_yaw, M_PI) - cur_yaw);
      min_dis = target_predict_position.head(2).norm();
      hit_yaw = target_yaw;
      hit_pitch = target_pitch;
    }
  }

  // Publish the gimbal command.
  auto gimbal_cmd = rm_interfaces::msg::GimbalCmd();
  gimbal_cmd.pitch_type = rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  gimbal_cmd.yaw_type = rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  gimbal_cmd.position.pitch = hit_pitch + offset_pitch_;
  gimbal_cmd.position.yaw = hit_yaw + offset_yaw_;
  gimbal_cmd.velocity.pitch = 0.0;
  gimbal_cmd.velocity.yaw = 0.0;
  gimbal_cmd_publisher_->publish(gimbal_cmd);
}

void ProjectileMotionNode::shoot_data_callback(
  const rm_interfaces::msg::RobotShootData::SharedPtr msg)
{
  shoot_speed_ = static_cast<double>(msg->bullet_speed);
  if (solver_type_ == "gravity") {
    auto solver_ptr = std::dynamic_pointer_cast<rmoss_projectile_motion::GravityProjectileSolver>(
      solver_);
    solver_ptr->set_initial_vel(shoot_speed_);
  } else if (solver_type_ == "gaf") {
    auto solver_ptr = std::dynamic_pointer_cast<rmoss_projectile_motion::GafProjectileSolver>(
      solver_);
    solver_ptr->set_initial_vel(shoot_speed_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown solver type: %s", solver_type_.c_str());
  }
}

}  // namespace projectile_motion

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(projectile_motion::ProjectileMotionNode)
