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

#include <gtest/gtest.h>

#include <iostream>
#include <chrono>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/mat.hpp>

#include <armor_detector/openvino_detector.hpp>

using hrc = std::chrono::high_resolution_clock;

TEST(OpenVINODetector, benchmark)
{
  auto pkg_path = ament_index_cpp::get_package_share_directory("armor_detector");
  auto model_path = pkg_path + "/model/opt-1208-001.onnx";


  rm_auto_aim::OpenVINODetector detector(model_path, "AUTO");

  detector.init();

  detector.set_callback(
    [](const std::vector<rm_auto_aim::ArmorObject> & objs, int64_t timestamp,
    const cv::Mat & src_img) {
      (void)objs;
      (void)timestamp;
      (void)src_img;
      return;
    });


  auto test_mat = cv::Mat(1280, 1024, CV_8UC3);

  int loop_num = 200;
  int warm_up = 30;

  double time_min = DBL_MAX;
  double time_max = -DBL_MAX;
  double time_avg = 0;

  for (int i = 0; i < warm_up + loop_num; ++i) {
    auto start = hrc::now();
    auto res = detector.push_input(test_mat, 0);
    res.get();
    auto end = hrc::now();
    double time = std::chrono::duration<double, std::milli>(end - start).count();
    if (i >= warm_up) {
      time_min = std::min(time_min, time);
      time_max = std::max(time_max, time);
      time_avg += time;
    }
  }

  time_avg /= loop_num;

  std::cout << "time_min: " << time_min << "ms" << std::endl;
  std::cout << "time_max: " << time_max << "ms" << std::endl;
  std::cout << "time_avg: " << time_avg << "ms" << std::endl;
}
