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

#include <armor_processor/kalman_filter.hpp>

namespace rm_auto_aim
{

KalmanFilter::KalmanFilter(const int dim_x, const int dim_z, const int dim_u)
: Filter(dim_x, dim_z, dim_u)
{
  F = Eigen::MatrixXd::Zero(dim_x, dim_x);
  H = Eigen::MatrixXd::Zero(dim_z, dim_x);
  B = Eigen::MatrixXd::Zero(dim_x, dim_u);
}

Eigen::MatrixXd KalmanFilter::predict(const Eigen::VectorXd & u)
{
  x_prior = F * x_post + B * u;
  P = F * P * F.transpose() + Q;

  return x_prior;
}

Eigen::MatrixXd KalmanFilter::update(const Eigen::VectorXd & z)
{
  K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  x_post = x_prior + K * (z - H * x_prior);
  P = (I - K * H) * P;

  return x_post;
}

}  // namespace rm_auto_aim
