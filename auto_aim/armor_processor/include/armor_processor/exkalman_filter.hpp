#ifndef ARMOR_PROCESSOR__EXKALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__EXKALMAN_FILTER_HPP_

#include <armor_processor/filter_base.hpp>

#include <functional>

namespace rm_auto_aim
{

using StateTransFunction = std::function<Eigen::MatrixXd(const Eigen::MatrixXd&, const Eigen::MatrixXd&)>;
using JacobianStateTransFunction = StateTransFunction;
using MeasureFunction = std::function<Eigen::MatrixXd(const Eigen::MatrixXd&)>;
using JacobianMeasureFunction = MeasureFunction;

class ExKalmanFilter : public Filter
{
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
  explicit ExKalmanFilter(StateTransFunction state_func, MeasureFunction measure_func, 
                          JacobianStateTransFunction jaco_state_func, JacobianMeasureFunction jaco_measure_func,
                          const int dim_x, const int dim_z, const int dim_u = 0);

  Eigen::MatrixXd predict(const Eigen::VectorXd& u) override;

  Eigen::MatrixXd update(const Eigen::VectorXd& z) override;

public:
  StateTransFunction state_func;
  MeasureFunction measure_func;
  JacobianStateTransFunction jaco_state_func;
  JacobianMeasureFunction jaco_measure_func;

  Eigen::MatrixXd dH, dG;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__EXKALMAN_FILTER_HPP_