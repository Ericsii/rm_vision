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

#include <armor_processor/exkalman_filter.hpp>

namespace rm_auto_aim
{
ExKalmanFilter::ExKalmanFilter(
  StateTransFunction state_func, MeasureFunction measure_func,
  JacobianStateTransFunction jaco_state_func, JacobianMeasureFunction jaco_measure_func,
  const int dim_x, const int dim_z, const int dim_u)
: Filter(dim_x, dim_z, dim_u)
  , state_func(state_func)
  , measure_func(measure_func)
  , jaco_state_func(jaco_state_func)
  , jaco_measure_func(jaco_measure_func)
{
}

Eigen::VectorXd ExKalmanFilter::predict(const Eigen::VectorXd & u)
{
  dG = jaco_state_func(u, x_post);
  x_prior = state_func(u, x_post);
  P = dG * P * dG.transpose() + Q;

  return x_prior;
}

Eigen::VectorXd ExKalmanFilter::update(const Eigen::VectorXd & z)
{
  dH = jaco_measure_func(z);
  K = P * dH.transpose() * (dH * P * dH.transpose() + R).inverse();
  x_post = x_prior + K * (z - measure_func(x_prior));
  P = (I - K * dH) * P;

  return x_post;
}

}  // namespace rm_auto_aim
