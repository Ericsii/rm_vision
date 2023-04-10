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
  target_topic_ = this->declare_parameter("projectile.target_topic", "processor/target");
  gimbal_cmd_topic_ = this->declare_parameter("projectile.gimbal_cmd_topic", "gimbal_cmd");
  shoot_data_topic_ = this->declare_parameter("projectile.shoot_data_topic", "robot_shoot_data");
  solver_type_ = this->declare_parameter("projectile.solver_type", "gravity");
  target_frame_ = this->declare_parameter("projectile.target_frame", "shooter_link");

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

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
    this->get_clock(), rclcpp::Duration::from_seconds(10.0).to_chrono<std::chrono::nanoseconds>());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  target_subscriber_.subscribe(this, target_topic_, rmw_qos_profile_sensor_data);
  tf_filter_ = std::make_shared<tf2_filter>(
    target_subscriber_, *tf_buffer_, target_frame_, 10,
    this->get_node_logging_interface(), this->get_node_clock_interface(),
    std::chrono::duration<int>(1));
  tf_filter_->registerCallback(&ProjectileMotionNode::target_callback, this);
}

void ProjectileMotionNode::target_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  auto target_position = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
  auto target_time = msg->header.stamp;
  auto target_velocity = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  // Use distance to calculate the time offset. (Approximate)
  auto distance = target_position.norm();
  auto time_offset = distance / shoot_speed_ + offset_time_;

  // Calculate the target position at hit time.
  auto hit_position = target_position + target_velocity * time_offset;
  auto hit_distance = hit_position.head(2).norm(), hit_height = hit_position.z();
  double pitch, yaw;
  solver_->solve(hit_distance, hit_height, pitch);
  yaw = std::atan2(hit_position.y(), hit_position.x());

  // Transform the pitch and yaw to the shooter_link frame.
  // Define the transform from the target frame to the shooter_link frame.
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped =
      tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, target_time);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transform target pose: %s", ex.what());
    return;
  }

  // Create a quaternion from the pitch and yaw angles.
  Eigen::Quaterniond q = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  // Transform the quaternion to the shooter_link frame.
  Eigen::Quaterniond q_shooter_link =
    Eigen::Quaterniond(
    transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x,
    transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z) *
    q *
    Eigen::Quaterniond(
    transform_stamped.transform.rotation.w, -transform_stamped.transform.rotation.x,
    -transform_stamped.transform.rotation.y, -transform_stamped.transform.rotation.z);

  // Extract the pitch and yaw angles from the transformed quaternion.
  Eigen::Vector3d euler_angles = q_shooter_link.toRotationMatrix().eulerAngles(2, 1, 0);
  double pitch_shooter_link = euler_angles[1] + offset_pitch_;
  double yaw_shooter_link = euler_angles[0] + offset_yaw_;

  // Publish the gimbal command.
  auto gimbal_cmd = rm_interfaces::msg::GimbalCmd();
  gimbal_cmd.pitch_type = rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  gimbal_cmd.yaw_type = rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  gimbal_cmd.position.pitch = pitch_shooter_link;
  gimbal_cmd.position.yaw = yaw_shooter_link;
  gimbal_cmd.velocity.pitch = 0.0;
  gimbal_cmd.velocity.yaw = 0.0;
  gimbal_cmd_publisher_->publish(gimbal_cmd);
}

void ProjectileMotionNode::shoot_data_callback(
  const rm_interfaces::msg::RobotShootData::SharedPtr msg)
{
  shoot_speed_ = static_cast<double>(msg->bullet_speed);
  if (solver_type_ == "gravity") {
    solver_ = std::make_shared<rmoss_projectile_motion::GravityProjectileSolver>(shoot_speed_);
  } else if (solver_type_ == "gaf") {
    solver_ =
      std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(shoot_speed_, friction_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown solver type: %s", solver_type_.c_str());
    return;
  }
}

}  // namespace projectile_motion
