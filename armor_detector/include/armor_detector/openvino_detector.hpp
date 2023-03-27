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

#ifndef ARMOR_DETECTOR__OPENVINO_DETECTOR_HPP_
#define ARMOR_DETECTOR__OPENVINO_DETECTOR_HPP_

#include <Eigen/Dense>

#include <string>
#include <vector>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <future>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

#include <armor_detector/types.hpp>

namespace rm_auto_aim
{

struct GridAndStride
{
  int grid0;
  int grid1;
  int stride;
};

class OpenVINODetector
{
public:
  using DetectorCallback = std::function<void (const std::vector<ArmorObject> &, int64_t,
      const cv::Mat &)>;

public:
  /**
   * @brief Construct a new OpenVINO Detector object
   *
   * @param model_path IR/ONNX file path
   * @param device_name Target device (CPU, GPU, AUTO)
   * @param conf_threshold Confidence threshold for output filtering
   * @param top_k Topk parameter
   * @param nms_threshold NMS threshold
   * @param auto_init If initializing detector inplace
   */
  explicit OpenVINODetector(
    const std::filesystem::path & model_path, const std::string & device_name,
    float conf_threshold = 0.25, int top_k = 128, float nms_threshold = 0.3,
    bool auto_init = false);

  /**
   * @brief Initialize detector
   *
   */
  void init();

  /**
   * @brief Push a single image to inference
   *
   * @param rgb_img
   * @param timestamp_nanosec
   * @return std::future<bool> If callback finished return ture.
   */
  std::future<bool> push_input(const cv::Mat & rgb_img, int64_t timestamp_nanosec);

  /**
   * @brief Set the inference callback
   *
   * @param callback
   */
  void set_callback(DetectorCallback callback);

private:
  bool process_callback(
    const cv::Mat resized_img, Eigen::Matrix3f transform_matrix, int64_t timestamp_nanosec,
    const cv::Mat & src_img);

private:
  std::string model_path_;
  std::string device_name_;
  float conf_threshold_;
  int top_k_;
  float nms_threshold_;
  std::vector<int> strides_;
  std::vector<GridAndStride> grid_strides_;

  DetectorCallback infer_callback_;

  std::unique_ptr<ov::Core> ov_core_;
  std::unique_ptr<ov::CompiledModel> compiled_model_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__OPENVINO_DETECTOR_HPP_
