#include <rm_omni/omni_node.hpp>

namespace rm_omni
{

OmniNode::OmniNode(rclcpp::NodeOptions options) : Node("omni_node", options.use_intra_process_comms(true))
{
  camera_num_ = this->declare_parameter<int>("camera_num", 0);

  for (int i = 0; i < camera_num_; ++i)
  {
    camera_names_.push_back(this->declare_parameter<std::string>("camera" + std::to_string(i) + "_name", ""));
  }
}

}  // namespace rm_omni
