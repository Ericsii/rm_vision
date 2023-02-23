// Copyright 2022 Chen Jun

#include "armor_processor/tracker.hpp"

#include <cfloat>
#include <iostream>
#include <memory>

namespace rm_auto_aim
{
Tracker::Tracker(
  const KalmanFilterMatrices & kf_matrices, double max_match_distance, int tracking_threshold,
  int lost_threshold)
: tracker_state(LOST),
  tracking_id(0),
  kf_matrices_(kf_matrices),
  tracking_velocity_(Eigen::Vector3d::Zero()),
  max_match_distance_(max_match_distance),
  tracking_threshold_(tracking_threshold),
  lost_threshold_(lost_threshold)
{
}

void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }

  // TODO(chenjun): need more judgement
  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  auto chosen_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      chosen_armor = armor;
    }
  }

  // KF init
  kf_ = std::make_unique<KalmanFilter>(kf_matrices_);
  Eigen::VectorXd init_state(6);
  const auto position = chosen_armor.position;
  init_state << position.x, position.y, position.z, 0, 0, 0;
  kf_->init(init_state);

  tracking_id = chosen_armor.number;
  tracker_state = DETECTING;
}

void Tracker::update(const Armors::SharedPtr & armors_msg, const double & dt)
{
  // KF predict
  kf_matrices_.F(0, 3) = kf_matrices_.F(1, 4) = kf_matrices_.F(2, 5) = dt;
  Eigen::VectorXd kf_prediction = kf_->predict(kf_matrices_.F);

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = kf_prediction;

  if (!armors_msg->armors.empty()) {
    Armor matched_armor;
    double min_position_diff = DBL_MAX;
    for (const auto & armor : armors_msg->armors) {
      Eigen::Vector3d position_vec(armor.position.x, armor.position.y, armor.position.z);
      Eigen::Vector3d predicted_position = kf_prediction.head(3);
      // Difference of the current armor position and tracked armor's predicted position
      double position_diff = (predicted_position - position_vec).norm();
      if (position_diff < min_position_diff) {
        min_position_diff = position_diff;
        matched_armor = armor;
      }
    }

    if (min_position_diff < max_match_distance_) {
      // Matching armor found
      matched = true;
      Eigen::Vector3d position_vec(
        matched_armor.position.x, matched_armor.position.y, matched_armor.position.z);
      target_state = kf_->update(position_vec);
    } else {
      // Check if there is same id armor in current frame
      for (const auto & armor : armors_msg->armors) {
        if (armor.number == tracking_id) {
          matched = true;
          // Reset KF
          kf_ = std::make_unique<KalmanFilter>(kf_matrices_);
          Eigen::VectorXd init_state(6);
          // Set init state with current armor position and tracking velocity before
          init_state << armor.position.x, armor.position.y, armor.position.z, tracking_velocity_;
          kf_->init(init_state);
          target_state = init_state;
          break;
        }
      }
    }
  }

  // Save tracking target velocity
  tracking_velocity_ = target_state.tail(3);

  // Tracking state machine
  if (tracker_state == DETECTING) {
    // DETECTING
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_threshold_) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }

  } else if (tracker_state == TRACKING) {
    // TRACKING
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }

  } else if (tracker_state == TEMP_LOST) {
    // LOST
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_threshold_) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
}

}  // namespace rm_auto_aim
