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

#ifndef RM_OMNI__OMNI_NODE_HPP_
#define RM_OMNI__OMNI_NODE_HPP_

#include <queue>
#include <future>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rmoss_util/url_resolver.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <openvino_armor_detector/types.hpp>
#include <openvino_armor_detector/openvino_detector.hpp>
#include <openvino_armor_detector/mono_measure_tool.hpp>

namespace rm_omni
{

class OmniNode : public rclcpp::Node
{
public:
  explicit OmniNode(rclcpp::NodeOptions options);

private:
  void init_detector();

  void img_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info);

  void openvino_detect_callback(
    const std::vector<rm_auto_aim::ArmorObject> & objs, int64_t timestamp_nanosec,
    const cv::Mat & src_img);

  // Debug functions
  void create_debug_publishers();

  void destroy_debug_publishers();

private:
  int camera_num_;
  std::vector<std::string> camera_names_;
  std::string transport_type_;
  std::string frame_id_;

  // OpenVINO Detector
  int detect_color_;  // 0: red, 1: blue
  std::mutex detector_mutex_;
  std::unique_ptr<rm_auto_aim::OpenVINODetector> detector_;
  std::queue<std::future<bool>> detect_requests_;

  // Camera measure
  std::unique_ptr<rm_auto_aim::MonoMeasureTool> measure_tool_;
  sensor_msgs::msg::CameraInfo::SharedPtr cam_info_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;

  // ROS
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Armors>::SharedPtr armors_pub_;
  std::vector<std::shared_ptr<image_transport::CameraSubscriber>> img_subs_;

  // Debug publishers
  bool debug_mode_{false};
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  image_transport::Publisher debug_img_pub_;
};

}  // namespace rm_omni

#endif  // RM_OMNI__OMNI_NODE_HPP_
