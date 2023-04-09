// Copyright 2023 Tingxu Chen
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

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include "armor_processor/filter_base.hpp"

namespace rm_auto_aim
{

class KalmanFilter : public Filter
{
public:
  explicit KalmanFilter(const int dim_x, const int dim_z, const int dim_u = 0);

  Eigen::VectorXd predict(const Eigen::VectorXd & u) override;

  Eigen::VectorXd update(const Eigen::VectorXd & z) override;

public:
  Eigen::MatrixXd F, H, B;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
