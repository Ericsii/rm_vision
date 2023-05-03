#ifndef RM_OMNI__OMNI_NODE_HPP_
#define RM_OMNI__OMNI_NODE_HPP_

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <openvino_armor_detector/openvino_detector.hpp>

namespace rm_omni
{

class OmniNode : public rclcpp::Node
{
public:
  explicit OmniNode(rclcpp::NodeOptions options);

private:
  void init_detector();

private:
  int camera_num_;
  std::vector<std::string> camera_names_;

  std::unique_ptr<rm_auto_aim::OpenVINODetector> detector_;
};

}  // namespace rm_omni

#endif  // RM_OMNI__OMNI_NODE_HPP_
