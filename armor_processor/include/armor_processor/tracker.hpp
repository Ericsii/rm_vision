#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>

#include "armor_processor/kalman_filter.hpp"
#include "armor_processor/exkalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{
class Tracker
{
public:
  Tracker(
    const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R, double dt, double max_match_distance,
    int tracking_threshold,
    int lost_threshold);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg);

  void update(const Armors::SharedPtr & armors_msg, const double & dt);

  enum State
  {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  char tracking_id;
  Eigen::VectorXd target_state;

private:
  Eigen::MatrixXd Q_, R_;
  std::shared_ptr<Filter> kf_;

  double dt_;

  Eigen::Vector3d tracking_velocity_;

  double max_match_distance_;

  int tracking_threshold_;
  int lost_threshold_;

  int detect_count_;
  int lost_count_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
