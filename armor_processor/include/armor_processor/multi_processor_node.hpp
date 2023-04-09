// Copyright 2023 Yunlong Feng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// This program is based on https://github.com/chenjunnn/rm_auto_aim.
// which is released under the MIT License.
//
// Copyright (c) 2022 ChenJun

#ifndef ARMOR_PROCESSOR__MULTI_PROCESSOR_NODE_MULTI_PROCESSOR_NODE_
#define ARMOR_PROCESSOR__MULTI_PROCESSOR_NODE_HPP_

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "armor_processor/exkalman_filter.hpp"
#include <armor_processor/spin_observer.hpp>
#include <armor_processor/ekf_tracker.hpp>
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/spin_info.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
class MultiProcessorNode : public rclcpp::Node
{
public:
  using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
  explicit MultiProcessorNode(rclcpp::NodeOptions options);

private:
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);
  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

private:
  // Last time received msg
  rclcpp::Time last_time_;

  // Initial KF matrices
  double sigma_p_; // position noise
  double sigma_v_; // velocity noise
  double sigma_R_; // measurement noise
  Eigen::Matrix<double, 6, 6> Q_; // initial covariance matrix
  Eigen::Matrix<double, 3, 3> R_; // initial covariance matrix

  // Tracker
  double max_match_distance_;
  int tracking_threshold_;
  int lost_threshold_;
  double filter_dt_;
  uint8_t target_color_; // 0: blue 1: red
  std::shared_ptr<EKFTracker> tracker_;

  // Spin observer
  bool enable_spin_observer_;
  double max_jump_angle_;
  double max_jump_period_;
  double allow_following_range_;
  std::shared_ptr<SpinObserver> spin_observer_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::SpinInfo>::SharedPtr spin_info_pub_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_filter> tf_filter_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;

  // Debug info
  bool debug_mode_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  rclcpp::ParameterCallbackHandle::SharedPtr debug_cb_handle_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker velocity_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__MULTI_PROCESSOR_NODE_HPP_
