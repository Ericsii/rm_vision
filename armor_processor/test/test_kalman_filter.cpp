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

#include <gtest/gtest.h>

#include <iostream>
#include <memory>

#include "armor_processor/kalman_filter.hpp"
#include "armor_processor/exkalman_filter.hpp"


TEST(Filter, KalmanFilter)
{
  using rm_auto_aim::Filter;
  using rm_auto_aim::KalmanFilter;
  std::shared_ptr<Filter> test_filter;
  auto test_kf = std::make_shared<KalmanFilter>(6, 3);

  Eigen::MatrixXd F(6, 6);
  F << 1, 0, 0, 1, 0, 0,
    0, 1, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 1,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;

  Eigen::MatrixXd H(3, 6);
  H << 1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0;

  Eigen::MatrixXd Q(6, 6);
  Q << 0.01, 0, 0, 0, 0, 0,
    0, 0.01, 0, 0, 0, 0,
    0, 0, 0.01, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0,
    0, 0, 0, 0, 0.01, 0,
    0, 0, 0, 0, 0, 0.01;

  Eigen::MatrixXd R(3, 3);
  R << 0.01, 0, 0,
    0, 0.01, 0,
    0, 0, 0.01;

  test_kf->F = F;
  test_kf->H = H;
  test_kf->Q = Q;
  test_kf->R = R;
  test_filter = test_kf;
  Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6);

  test_filter->init(x_0);

  for (int i = 0; i < 100; ++i) {
    Eigen::VectorXd z(3);
    z << i, i, i;
    test_filter->predict();
    auto x = test_filter->update(z);
  }
}

TEST(Filter, ExKalmanFilter)
{
  using rm_auto_aim::Filter;
  using rm_auto_aim::ExKalmanFilter;
  std::shared_ptr<Filter> test_filter;

  Eigen::MatrixXd F(6, 6);
  F << 1, 0, 0, 1, 0, 0,
    0, 1, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 1,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;

  Eigen::MatrixXd H(3, 6);
  H << 1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0;

  ExKalmanFilter::StateTransFunction state_func =
    [&F](const Eigen::VectorXd & u, const Eigen::VectorXd & x)
    {
      (void) u;
      return F * x;
    };

  ExKalmanFilter::JacobianStateTransFunction jaco_state_func =
    [&F](const Eigen::VectorXd & u, const Eigen::VectorXd & x)
    {
      (void) x;
      (void) u;
      return F;
    };

  ExKalmanFilter::MeasureFunction measure_func = [&H](const Eigen::VectorXd & x)
    {
      return H * x;
    };

  ExKalmanFilter::JacobianMeasureFunction jaco_measure_func = [&H](const Eigen::VectorXd & x)
    {
      (void) x;
      return H;
    };

  test_filter = std::make_shared<ExKalmanFilter>(
    state_func, measure_func, jaco_state_func,
    jaco_measure_func, 6, 3);

  Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6);

  test_filter->init(x_0);

  for (int i = 0; i < 100; ++i) {
    Eigen::VectorXd z(3);
    z << i, i, i;
    test_filter->predict();
    auto x = test_filter->update(z);
  }
}
