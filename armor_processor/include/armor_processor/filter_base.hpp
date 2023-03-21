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

#ifndef ARMOR_PROCESSOR__FILTER_BASE_HPP_
#define ARMOR_PROCESSOR__FILTER_BASE_HPP_

#include <Eigen/Dense>

namespace rm_auto_aim
{

class Filter
{
public:
  /**
   * @brief Construct a new Filter object
   *
   * @param dim_x 状态维度
   * @param dim_z 测量维度
   * @param dim_u 控制维度 default: 0
   */
  explicit Filter(const int dim_x, const int dim_z, const int dim_u = 0)
  : dim_x(dim_x), dim_u(dim_u), dim_z(dim_z)
  {
    x_post = x_prior = Eigen::VectorXd::Zero(dim_x);
    P = Eigen::MatrixXd::Zero(dim_x, dim_x);
    Q = Eigen::MatrixXd::Zero(dim_x, dim_x);
    R = Eigen::MatrixXd::Zero(dim_z, dim_z);
    K = Eigen::MatrixXd::Zero(dim_x, dim_z);
    I = Eigen::MatrixXd::Identity(dim_x, dim_x);
  }

  // Initialize the filter with a guess for initial states.
  /**
   * @brief
   *
   * @param x0
   */
  virtual void init(const Eigen::VectorXd & x0)
  {
    x_post = x0;
  }

  /**
   * @brief Predict next state
   *
   * @param u Control input
   * @return Eigen::MatrixXd
   */
  virtual Eigen::MatrixXd predict(const Eigen::VectorXd & u) = 0;
  /**
   * @brief Predict next state u = 0
   *
   * @return Eigen::MatrixXd
   */
  virtual Eigen::MatrixXd predict()
  {
    return predict(Eigen::VectorXd::Zero(dim_u));
  }

  /**
   * @brief Update the estimated state based on measurement
   *
   * @param z Measurement state
   * @return Eigen::MatrixXd
   */
  virtual Eigen::MatrixXd update(const Eigen::VectorXd & z) = 0;

public:
  // System dimensions
  int dim_x, dim_u, dim_z;

  // Invariant matrices
  Eigen::MatrixXd Q, R;

  // Posteriori error covariance matrix
  Eigen::MatrixXd P;

  // Kalman gain
  Eigen::MatrixXd K;

  // Predicted state
  Eigen::VectorXd x_prior;
  // Updated state
  Eigen::VectorXd x_post;

  // Identity matrix
  Eigen::MatrixXd I;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__FILTER_BASE_HPP_
