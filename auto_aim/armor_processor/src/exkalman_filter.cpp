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

Eigen::MatrixXd ExKalmanFilter::predict(const Eigen::VectorXd & u)
{
  dG = jaco_state_func(u, x_post);
  x_prior = state_func(u, x_post);
  P = dG * P * dG.transpose() + R;

  return x_prior;
}

Eigen::MatrixXd ExKalmanFilter::update(const Eigen::VectorXd & z)
{
  dH = jaco_measure_func(z);
  K = P * dH.transpose() * (dH * P * dH.transpose() + Q).inverse();
  x_post = x_prior + K * (z - measure_func(z));
  P = (I - K * dH) * P;

  return x_post;
}

}  // namespace rm_auto_aim
