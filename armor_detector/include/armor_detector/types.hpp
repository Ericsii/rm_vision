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

#ifndef ARMOR_DETECTOR__TYPES_HPP_
#define ARMOR_DETECTOR__TYPES_HPP_

#include <vector>

#include <opencv2/core/mat.hpp>

namespace rm_auto_aim
{

enum class ArmorColor
{
  BLUE = 0,
  RED,
  NONE,
  PURPLE
};

enum class ArmorNumber
{
  SENTRY = 0,
  NO1,
  NO2,
  NO3,
  NO4,
  NO5,
  OUTPOST,
  BASE
};

typedef struct
{
  ArmorColor color;
  ArmorNumber number;
  float prob;
  std::vector<cv::Point2f> pts;
  cv::Rect box;
} ArmorObject;

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__TYPES_HPP_
