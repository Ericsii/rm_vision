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

#ifndef ARMOR_PROCESSOR__EXKALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__EXKALMAN_FILTER_HPP_

#include <functional>

#include <armor_processor/filter_base.hpp>

namespace rm_auto_aim
{

class ExKalmanFilter : public Filter
{
public:
  using StateTransFunction = std::function<Eigen::MatrixXd(
        const Eigen::VectorXd &,
        const Eigen::VectorXd &)>;
  using JacobianStateTransFunction = StateTransFunction;
  using MeasureFunction = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
  using JacobianMeasureFunction = MeasureFunction;

public:
  /**
   * @brief Construct a new Ex Kalman Filter object
   *
   * @param state_func State transmition function g(u, x)
   * @param measure_func Measurement function h(z)
   * @param jaco_state_func Jacobian function of g(u, x)
   * @param jaco_measure_func Jacobian function of h(z)
   * @param dim_x State vector dimension
   * @param dim_z Measurement vector dimension
   * @param dim_u Control vector dimension
   */
  explicit ExKalmanFilter(
    StateTransFunction state_func, MeasureFunction measure_func,
    JacobianStateTransFunction jaco_state_func, JacobianMeasureFunction jaco_measure_func,
    const int dim_x, const int dim_z, const int dim_u = 0);

  Eigen::MatrixXd predict(const Eigen::VectorXd & u) override;

  Eigen::MatrixXd update(const Eigen::VectorXd & z) override;

public:
  StateTransFunction state_func;
  MeasureFunction measure_func;
  JacobianStateTransFunction jaco_state_func;
  JacobianMeasureFunction jaco_measure_func;

  Eigen::MatrixXd dH, dG;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__EXKALMAN_FILTER_HPP_
