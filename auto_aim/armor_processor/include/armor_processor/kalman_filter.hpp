#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include "armor_processor/filter_base.hpp"

namespace rm_auto_aim
{

class KalmanFilter : public Filter
{
public:
  explicit KalmanFilter(const int dim_x, const int dim_z, const int dim_u = 0);

  Eigen::MatrixXd predict(const Eigen::VectorXd& u) override;

  Eigen::MatrixXd update(const Eigen::VectorXd& z) override;

public:
  Eigen::MatrixXd F, H, B;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
