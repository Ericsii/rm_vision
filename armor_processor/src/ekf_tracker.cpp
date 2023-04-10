#include <armor_processor/ekf_tracker.hpp>

#include <cfloat>

#include <armor_processor/exkalman_filter.hpp>

namespace rm_auto_aim
{
EKFTracker::EKFTracker(
  const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R, double dt, double max_match_distance,
  int tracking_threshold, int lost_threshold, uint8_t target_color)
: target_color(target_color)
  , Q_(Q)
  , R_(R)
  , dt_(dt)
{
  max_match_distance_ = max_match_distance;
  tracking_threshold_ = tracking_threshold;
  lost_threshold_ = lost_threshold;

  // Create kinematic model (Use constant velocity model)
  state_trans_func_ = [this](const Eigen::VectorXd & u, const Eigen::VectorXd & x) {
      Eigen::VectorXd x_ = x;
      x_(0) += x(3) * this->dt_ + u(0) * this->dt_;
      x_(1) += x(4) * this->dt_ + u(1) * this->dt_;
      x_(2) += x(5) * this->dt_ + u(2) * this->dt_;
      return x_;
    };
  measure_func_ = [](const Eigen::VectorXd & z) {return z.head(3);};
  state_trans_jacobian_func_ = [this](const Eigen::VectorXd & u, const Eigen::VectorXd & x) {
      (void)u;
      (void)x;
      Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
      F(0, 3) = this->dt_;
      F(1, 4) = this->dt_;
      F(2, 5) = this->dt_;
      return F;
    };
  measure_jacobian_func_ = [](const Eigen::VectorXd & x) {
      (void)x;
      return Eigen::MatrixXd::Identity(3, 6);
    };
}

void EKFTracker::init(const Armor::SharedPtr & armor_msg)
{
  kf_ = std::make_shared<ExKalmanFilter>(
    state_trans_func_, measure_func_, state_trans_jacobian_func_,
    measure_jacobian_func_, 6, 3, 3);
  kf_->Q = Q_;
  kf_->R = R_;
  Eigen::VectorXd x(6);
  x << armor_msg->position.x, armor_msg->position.y, armor_msg->position.z, 0, 0, 0;
  kf_->init(x);

  this->tracking_id = armor_msg->number;
  this->target_color = armor_msg->color;
  this->tracker_state = DETECTING;
}

void EKFTracker::update(const Armors::SharedPtr & armors_msg, double dt)
{
  // Update dt
  dt_ = dt;
  // Predict
  Eigen::VectorXd u(3);
  u << 0, 0, 0;
  Eigen::VectorXd kf_prediction = kf_->predict(u);
  target_state = kf_prediction;

  // Find target armors
  bool is_target_match = false;
  std::vector<Armor> target_armors;
  for (const auto & armor : armors_msg->armors) {
    if (armor.color == this->target_color && armor.number == this->tracking_id) {
      target_armors.push_back(armor);
    }
  }

  // Match
  if (!target_armors.empty()) {
    is_target_match = true;
    // Find the closest armor
    double min_distance = DBL_MAX;
    Armor closest_armor;
    for (const auto & armor : target_armors) {
      Eigen::Vector3d position_vec(armor.position.x, armor.position.y, armor.position.z);
      double distance = (position_vec - kf_prediction.head(3)).norm();
      if (distance < min_distance) {
        min_distance = distance;
        closest_armor = armor;
      }
    }

    if (min_distance < max_match_distance_) {
      // Update
      Eigen::VectorXd z(3);
      z << closest_armor.position.x, closest_armor.position.y, closest_armor.position.z;
      target_state = kf_->update(z);
    } else {
      // Reset KF
      kf_ = std::make_shared<ExKalmanFilter>(
        state_trans_func_, measure_func_, state_trans_jacobian_func_,
        measure_func_, 6, 3, 3);
      kf_->Q = Q_;
      kf_->R = R_;
      Eigen::VectorXd x(6);
      x << closest_armor.position.x, closest_armor.position.y, closest_armor.position.z,
        tracking_velocity_;
      kf_->init(x);
      target_state = x;
    }
  }

  // Save tracking target velocity
  tracking_velocity_ = target_state.tail(3);

  // Update tracking state machine
  switch (tracker_state) {
    case LOST:
      if (is_target_match) {
        detect_count_++;
        tracker_state = DETECTING;
      } else {
        detect_count_ = 0;
      }
      break;
    case DETECTING:
      if (is_target_match) {
        detect_count_++;
        if (detect_count_ >= tracking_threshold_) {
          tracker_state = TRACKING;
          detect_count_ = 0;
        }
      } else {
        detect_count_ = 0;
        tracker_state = LOST;
      }
      break;
    case TRACKING:
      if (!is_target_match) {
        lost_count_++;
        tracker_state = TEMP_LOST;
      }
      break;
    case TEMP_LOST:
      if (is_target_match) {
        lost_count_ = 0;
        tracker_state = TRACKING;
      } else {
        lost_count_++;
        if (lost_count_ >= lost_threshold_) {
          tracker_state = LOST;
          lost_count_ = 0;
        }
      }
      break;
  }
}
}  // namespace rm_auto_aim
