// Copyright 2022 Chen Jun

#include <gtest/gtest.h>

#include <memory>
#include <random>
#include <vector>

#include "armor_processor/kalman_filter.hpp"

int N = 2;  // Number of states
int M = 1;  // Number of measurements

Eigen::MatrixXd F(N, N);  // state transition matrix
Eigen::MatrixXd H(M, N);  // measurement matrix
Eigen::MatrixXd Q(N, N);  // process noise covariance matrix
Eigen::MatrixXd R(M, M);  // measurement noise covariance matrix
Eigen::MatrixXd P(N, N);  // error estimate covariance matrix

std::unique_ptr<rm_auto_aim::KalmanFilter> KF;

TEST(KalmanFilterTest, init)
{
  // Test x = x0 + v*t
  double dt = 1.0;
  F << 1, dt, 0, 1;
  H << 1, 0;

  Q << .05, .05, .0, .05;
  R << 0.1;
  P.setIdentity();

  auto matrices = rm_auto_aim::KalmanFilterMatrices{F, H, Q, R, P};

  KF = std::make_unique<rm_auto_aim::KalmanFilter>(matrices);

  std::cout << "F: \n" << F << std::endl;
  std::cout << "H: \n" << H << std::endl;
  std::cout << "Q: \n" << Q << std::endl;
  std::cout << "R: \n" << R << std::endl;
  std::cout << "P: \n" << P << std::endl;
}

TEST(KalmanFilterTest, predict_update)
{
  std::vector<double> measurements{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  // Add noise
  std::default_random_engine e;
  std::uniform_real_distribution<double> u(-0.1, 0.1);
  std::cout << "measurement:" << std::endl;
  for (auto & measurement : measurements) {
    measurement += u(e);
    std::cout << measurement << std::endl;
  }

  // Init
  Eigen::VectorXd x0(N);
  x0 << 0, 0;
  KF->init(x0);

  // Estimate
  std::cout << "Estimate based on measurement:" << std::endl;
  Eigen::VectorXd measurement_vector(M);
  for (const auto & measurement : measurements) {
    KF->predict(F);
    measurement_vector << measurement;
    auto result = KF->update(measurement_vector);
    std::cout << result.transpose() << std::endl;
  }

  // Predict only
  std::cout << "Predict only:" << std::endl;
  for (size_t i = 0; i < 10; ++i) {
    auto prediction = KF->predict(F);
    std::cout << prediction.transpose() << std::endl;
  }
}
