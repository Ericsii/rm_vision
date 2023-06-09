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

#include <rm_omni/omni_node.hpp>

#include <fmt/format.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace rm_omni
{

OmniNode::OmniNode(rclcpp::NodeOptions options)
: Node("omni_node", options.use_intra_process_comms(true))
{
  RCLCPP_INFO(this->get_logger(), "Initializing omni node");

  // OpenVINO
  RCLCPP_INFO(this->get_logger(), "Initializing OpenVINO");
  detector_ = nullptr;
  this->init_detector();
  if (!detector_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize OpenVINO");
    return;
  }
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  detect_color_ = this->declare_parameter("detect_color", 0, param_desc);

  // Measure tool
  measure_tool_ = std::make_unique<rm_auto_aim::MonoMeasureTool>();

  // ROS params
  auto use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
  camera_num_ = this->declare_parameter<int>("camera_num", 0);
  transport_type_ =
    this->declare_parameter("omni.subscribe_compressed", false) ? "compressed" : "raw";
  for (int i = 0; i < camera_num_; ++i) {
    camera_names_.push_back(
      this->declare_parameter<std::string>(
        "omni.camera" + std::to_string(i) +
        "_name", ""));
    RCLCPP_INFO(
      this->get_logger(), "camera_name: %s, transport_type: %s", camera_names_[i].c_str(),
      transport_type_.c_str());
  }

  // Debug mode handler
  RCLCPP_INFO(this->get_logger(), "Setup debug_mode handler");
  debug_mode_ = this->declare_parameter("debug_mode", false);
  if (debug_mode_) {
    this->create_debug_publishers();
  }
  // Regiter debug mode param handler
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ = debug_param_sub_->add_parameter_callback(
    "debug_mode", [this](const rclcpp::Parameter & p) {
      this->debug_mode_ = p.as_bool();
      this->debug_mode_ ? this->create_debug_publishers() : this->destroy_debug_publishers();
    });

  RCLCPP_INFO(this->get_logger(), "Setup ROS subs pubs");
  // Armors publisher
  armors_pub_ = this->create_publisher<auto_aim_interfaces::msg::Armors>(
    "omni/armors",
    rclcpp::SensorDataQoS());

  // Visualization Marker
  position_marker_.ns = "armors";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.r = 1.0;

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("omni/marker", 10);

  for (int i = 0; i < camera_num_; ++i) {
    img_subs_.push_back(
      std::make_shared<image_transport::CameraSubscriber>(
        image_transport::create_camera_subscription(
          this, "/" + camera_names_[i] + "/image_raw",
          std::bind(&OmniNode::img_callback, this, std::placeholders::_1, std::placeholders::_2),
          transport_type_,
          use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default)));
  }

  RCLCPP_INFO(this->get_logger(), "Initializing finished.");
}

void OmniNode::init_detector()
{
  auto model_path = this->declare_parameter("omni.model_path", "");
  auto device_type = this->declare_parameter("omni.device_type", "AUTO");
  float conf_threshold = this->declare_parameter("omni.confidence_threshold", 0.25);
  int top_k = this->declare_parameter("omni.top_k", 128);
  float nms_threshold = this->declare_parameter("omni.nms_threshold", 0.3);

  if (model_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Require model_path but got empty.");
    return;
  }
  auto resolved_path = rmoss_util::URLResolver::get_resolved_path(model_path);
  if (resolved_path.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Model file path format error. Should be package://<package>/<path> or "
      "file:///<path>");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Model path: %s", resolved_path.c_str());

  // Create detector
  detector_ =
    std::make_unique<rm_auto_aim::OpenVINODetector>(
    resolved_path, device_type, conf_threshold,
    top_k, nms_threshold);
  // Set detect callback
  detector_->set_callback(
    std::bind(
      &OmniNode::openvino_detect_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));
  // init detector
  detector_->init();
}

void OmniNode::img_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  std::lock_guard<std::mutex> lock(detector_mutex_);
  // limits request size
  while (detect_requests_.size() > 1) {
    detect_requests_.front().get();
    detect_requests_.pop();
  }

  cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*cam_info);
  measure_tool_->set_camera_info(
    std::vector<double>(
      cam_info->k.begin(),
      cam_info->k.end()), cam_info->d);
  frame_id_ = img->header.frame_id;

  auto timestamp = rclcpp::Time(img->header.stamp);
  auto img_cv = cv_bridge::toCvCopy(img, "rgb8")->image;

  // push image to detector
  detect_requests_.push(std::move(detector_->push_input(img_cv, timestamp.nanoseconds())));
}

void OmniNode::openvino_detect_callback(
  const std::vector<rm_auto_aim::ArmorObject> & objs, int64_t timestamp_nanosec,
  const cv::Mat & src_img)
{
  if (measure_tool_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "No camera_info recieve yet.");
    return;
  }

  detect_color_ = this->get_parameter("target_color").as_int();

  auto timestamp = rclcpp::Time(timestamp_nanosec);

  // Used to draw debug info
  cv::Mat debug_img;
  if (debug_mode_) {
    debug_img = src_img.clone();
  }

  auto_aim_interfaces::msg::Armors armors_msg;
  armors_msg.header.frame_id = frame_id_;
  armors_msg.header.stamp = timestamp;

  for (auto & obj : objs) {
    if (detect_color_ == 0 && obj.color != rm_auto_aim::ArmorColor::RED) {
      continue;
    } else if (detect_color_ == 1 && obj.color != rm_auto_aim::ArmorColor::BLUE) {
      continue;
    }

    auto_aim_interfaces::msg::Armor armor;

    cv::Point3f target_position;
    cv::Mat target_rvec;

    if (!measure_tool_->calc_armor_target(obj, target_position, target_rvec)) {
      RCLCPP_WARN(this->get_logger(), "Calc target failed.");
    }

    cv::Mat rot_mat;
    cv::Rodrigues(target_rvec, rot_mat);
    tf2::Matrix3x3 tf_rot_mat(rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1),
      rot_mat.at<double>(0, 2),
      rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
      rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
    tf2::Quaternion tf_quaternion;
    tf_rot_mat.getRotation(tf_quaternion);

    armor.number = rm_auto_aim::kArmorNames[static_cast<int>(obj.number)];
    armor.pose.position.x = target_position.x;
    armor.pose.position.y = target_position.y;
    armor.pose.position.z = target_position.z;
    armor.pose.orientation.x = tf_quaternion.x();
    armor.pose.orientation.y = tf_quaternion.y();
    armor.pose.orientation.z = tf_quaternion.z();
    armor.pose.orientation.w = tf_quaternion.w();
    armor.distance_to_image_center = measure_tool_->calc_distance_to_center(obj);

    armors_msg.armors.push_back(std::move(armor));

    if (debug_mode_) {
      if (debug_img.empty()) {
        // Avoid debug_mode change in processing
        continue;
      }

      // Draw armor
      for (size_t i = 0; i < 4; ++i) {
        cv::line(debug_img, obj.pts[i], obj.pts[(i + 1) % 4], cv::Scalar(255, 48, 48), 2);
      }

      std::string armor_color;
      switch (obj.color) {
        case rm_auto_aim::ArmorColor::BLUE:
          armor_color = "B";
          break;
        case rm_auto_aim::ArmorColor::RED:
          armor_color = "R";
          break;
        case rm_auto_aim::ArmorColor::NONE:
          armor_color = "N";
          break;
        case rm_auto_aim::ArmorColor::PURPLE:
          armor_color = "P";
          break;
        default:
          armor_color = "UNKOWN";
          break;
      }

      std::string armor_key = fmt::format("{} {}", armor_color, static_cast<int>(obj.number));
      cv::putText(
        debug_img, armor_key, cv::Point2i(obj.pts[0]), cv::FONT_HERSHEY_SIMPLEX, 0.8,
        cv::Scalar(0, 255, 255),
        2);
    }
  }

  armors_pub_->publish(std::move(armors_msg));

  if (debug_mode_) {
    if (debug_img.empty()) {
      // Avoid debug_mode change in processing
      return;
    }

    cv::circle(
      debug_img, cv::Point2i(cam_info_->width / 2., cam_info_->height / 2.), 5,
      cv::Scalar(255, 0, 0), 2);

    auto end = this->get_clock()->now();
    auto duration = end.seconds() - timestamp.seconds();
    std::string letency = fmt::format("Latency: {:.3f}ms", duration * 1000);
    cv::putText(
      debug_img, letency, cv::Point2i(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
    debug_img_pub_.publish(cv_bridge::CvImage(armors_msg.header, "rgb8", debug_img).toImageMsg());
  }
}

void OmniNode::create_debug_publishers()
{
  debug_img_pub_ = image_transport::create_publisher(this, "omni/debug_img");
}

void OmniNode::destroy_debug_publishers()
{
  debug_img_pub_.shutdown();
}

}  // namespace rm_omni

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_omni::OmniNode)
