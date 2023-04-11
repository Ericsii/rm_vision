// Copyright 2023 Chen Jun
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#ifndef ARMOR_PROCESSOR__EXTENDED_KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__EXTENDED_KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{

class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;

  using NonlinearFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using JacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;

  explicit ExtendedKalmanFilter(
    const NonlinearFunc & f, const NonlinearFunc & h, const JacobianFunc & Jf,
    const JacobianFunc & Jh, const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & P0);

  // Set the initial state
  void setState(const Eigen::VectorXd & x0);

  // Compute a predicted state
  Eigen::MatrixXd predict();

  // Update the estimated state based on measurement
  Eigen::MatrixXd update(const Eigen::VectorXd & z);

private:
  // Process nonlinear vector function
  NonlinearFunc f;
  // Observation nonlinear vector function
  NonlinearFunc h;
  // Jacobian of f()
  JacobianFunc Jf;
  Eigen::MatrixXd F;
  // Jacobian of h()
  JacobianFunc Jh;
  Eigen::MatrixXd H;
  // Process noise covariance matrix
  Eigen::MatrixXd Q;
  // Measurement noise covariance matrix
  Eigen::MatrixXd R;

  // Priori error estimate covariance matrix
  Eigen::MatrixXd P_pri;
  // Posteriori error estimate covariance matrix
  Eigen::MatrixXd P_post;

  // Kalman gain
  Eigen::MatrixXd K;

  // System dimensions
  int n;

  // N-size identity
  Eigen::MatrixXd I;

  // Priori state
  Eigen::VectorXd x_pri;
  // Posteriori state
  Eigen::VectorXd x_post;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__EXTENDED_KALMAN_FILTER_HPP_
