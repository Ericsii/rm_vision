// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace rm_auto_aim
{
struct KalmanFilterMatrices
{
  Eigen::MatrixXd F;  // state transition matrix
  Eigen::MatrixXd H;  // measurement matrix
  Eigen::MatrixXd Q;  // process noise covariance matrix
  Eigen::MatrixXd R;  // measurement noise covariance matrix
  Eigen::MatrixXd P;  // error estimate covariance matrix
};

class KalmanFilter
{
public:
  explicit KalmanFilter(const KalmanFilterMatrices & matrices);

  // Initialize the filter with a guess for initial states.
  void init(const Eigen::VectorXd & x0);

  // Computes a predicted state
  Eigen::MatrixXd predict(const Eigen::MatrixXd & F);

  // Update the estimated state based on measurement
  Eigen::MatrixXd update(const Eigen::VectorXd & z);

private:
  // Invariant matrices
  Eigen::MatrixXd F, H, Q, R;

  // Priori error estimate covariance matrix
  Eigen::MatrixXd P_pre;
  // Posteriori error estimate covariance matrix
  Eigen::MatrixXd P_post;

  // Kalman gain
  Eigen::MatrixXd K;

  // System dimensions
  int n;

  // N-size identity
  Eigen::MatrixXd I;

  // Predicted state
  Eigen::VectorXd x_pre;
  // Updated state
  Eigen::VectorXd x_post;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
