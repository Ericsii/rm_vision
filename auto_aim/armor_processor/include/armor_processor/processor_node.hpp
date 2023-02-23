// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_processor/kalman_filter.hpp"
#include "armor_processor/spin_observer.hpp"
#include "armor_processor/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/spin_info.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
class ArmorProcessorNode : public rclcpp::Node
{
public:
  explicit ArmorProcessorNode(const rclcpp::NodeOptions & options);

private:
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  // Last time received msg
  rclcpp::Time last_time_;

  // Initial KF matrices
  KalmanFilterMatrices kf_matrices_;
  double dt_;

  // Armor tracker
  std::unique_ptr<Tracker> tracker_;

  // Spin observer
  bool allow_spin_observer_;
  std::unique_ptr<SpinObserver> spin_observer_;
  rclcpp::Publisher<auto_aim_interfaces::msg::SpinInfo>::SharedPtr spin_info_pub_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker velocity_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Debug information publishers
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
