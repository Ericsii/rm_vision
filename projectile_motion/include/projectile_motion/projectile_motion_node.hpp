#ifndef PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_PROJECTILE_MOTION_NODE_
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
  ProjectileMotionNode(rclcpp::NodeOptions options);

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
  std::string target_frame_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_filter> tf_filter_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Target> target_subscriber_;

  std::shared_ptr<rmoss_projectile_motion::ProjectileSolverInterface> solver_;
  rclcpp::Subscription<rm_interfaces::msg::RobotShootData>::SharedPtr shoot_data_subscriber_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_publisher_;
};

}  // namespace projectile_motion

#endif  // PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_
