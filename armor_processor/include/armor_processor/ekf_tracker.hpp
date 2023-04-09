#ifndef ARMOR_PROCESSOR__EKF_TRACKER_HPP_
#define ARMOR_PROCESSOR__EKF_TRACKER_HPP_

#include <auto_aim_interfaces/msg/armors.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <armor_processor/exkalman_filter.hpp>

namespace rm_auto_aim
{
class EKFTracker
{
public:
  EKFTracker(
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R, double dt, double max_match_distance,
    int tracking_threshold, int lost_threshold, uint8_t target_color = 0);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armor::SharedPtr & armor);

  void update(const Armors::SharedPtr & armors_msg, double dt);

public:
  enum State
  {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  uint8_t target_color;
  uint8_t tracking_id;
  Eigen::VectorXd target_state;

private:
  Eigen::MatrixXd Q_, R_;
  double dt_;
  double max_match_distance_;
  int lost_threshold_;
  int tracking_threshold_;
  int detect_count_;
  int lost_count_;

  std::shared_ptr<Filter> kf_;
  Eigen::Vector3d tracking_velocity_;

  std::function<Eigen::VectorXd(
      const Eigen::VectorXd &,
      const Eigen::VectorXd &)> state_trans_func_;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> measure_func_;
  std::function<Eigen::MatrixXd(
      const Eigen::VectorXd &,
      const Eigen::VectorXd &)> state_trans_jacobian_func_;
  std::function<Eigen::MatrixXd(const Eigen::VectorXd &)> measure_jacobian_func_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__EKF_TRACKER_HPP_
