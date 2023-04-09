#include <armor_processor/multi_processor_node.hpp>

namespace rm_auto_aim
{
constexpr int kMaxArmorNum = 10;

MultiProcessorNode::MultiProcessorNode(rclcpp::NodeOptions options)
: Node("armor_processor", options.use_intra_process_comms(true)), last_time_(0)
{
  RCLCPP_INFO(this->get_logger(), "Setup ProcessorNode.");

  // Filter
  sigma_p_ = this->declare_parameter("filter.filter_p", 5000.0);
  sigma_v_ = this->declare_parameter("filter.filter_v", 5000.0);
  sigma_R_ = this->declare_parameter("filter.filter_R", 0.01);
  Eigen::DiagonalMatrix<double, 6> q;
  q.diagonal() << sigma_p_, sigma_p_, sigma_p_, sigma_v_, sigma_v_, sigma_v_;
  Q_ = q;
  R_ = Eigen::MatrixXd::Identity(3, 3) * sigma_R_;

  // Tracker
  max_match_distance_ = this->declare_parameter("tracker.max_match_distance", 0.2);
  tracking_threshold_ = this->declare_parameter("tracker.tracking_threshold", 5);
  lost_threshold_ = this->declare_parameter("tracker.lost_threshold", 5);
  filter_dt_ = this->declare_parameter("filter.filter_dt", 0.1);
  target_frame_ = this->declare_parameter("tracker.target_frame", "gimbal_odom");
  target_color_ = this->declare_parameter("tracker.initial_target_color", 0);

  tracker_ = std::make_shared<EKFTracker>(
    Q_, R_, filter_dt_, max_match_distance_, tracking_threshold_, lost_threshold_,
    target_color_);

  // Spin Observer
  enable_spin_observer_ = this->declare_parameter("spin_observer.enable", true);
  max_jump_angle_ = this->declare_parameter("spin_observer.max_jump_angle", 0.2);
  max_jump_period_ = this->declare_parameter("spin_observer.max_jump_period", 0.8);
  allow_following_range_ = this->declare_parameter("spin_observer.allow_following_range", 0.3);
  if (enable_spin_observer_) {
    spin_observer_ =
      std::make_shared<SpinObserver>(
      this->get_clock(), max_jump_angle_, max_jump_period_, allow_following_range_);
  }

  // Subscriber with tf2 message_filter
  // tf2 relevant
  RCLCPP_INFO(this->get_logger(), "Setup tf2 listener.");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
    this->get_clock(),
    rclcpp::Duration::from_seconds(1).to_chrono<tf2::Duration>());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  tf_filter_ =
    std::make_shared<tf2_filter>(
    armors_sub_, *tf_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  tf_filter_->registerCallback(&MultiProcessorNode::armorsCallback, this);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/processor/target",
    rclcpp::SensorDataQoS());

  // Deubug
  debug_mode_ = this->declare_parameter("debug_mode", false);

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  velocity_marker_.type = visualization_msgs::msg::Marker::ARROW;
  velocity_marker_.ns = "velocity";
  velocity_marker_.scale.x = 0.03;
  velocity_marker_.scale.y = 0.05;
  velocity_marker_.color.a = 1.0;
  velocity_marker_.color.b = 1.0;
  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/processor/marker", 10);

  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ = debug_param_sub_->add_parameter_callback(
    "debug_mode", [this](const rclcpp::Parameter & p) {debug_mode_ = p.as_bool();});
}

void MultiProcessorNode::armorsCallback(
  const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PointStamped ps;
    ps.header = armors_msg->header;
    ps.point = armor.position;
    try {
      armor.position = tf_buffer_->transform(ps, target_frame_).point;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  if (tracker_->tracker_state == EKFTracker::LOST) {
    // If lost, try to find a new target
    if (armors_msg->armors.size() > 0) {
      // If there are armors, find the closest one
      double min_distance = DBL_MAX;
      int min_index = 0;
      for (::size_t i = 0; i < armors_msg->armors.size(); i++) {
        if (armors_msg->armors[i].color != target_color_) {
          continue;
        }
        double distance = std::sqrt(
          armors_msg->armors[i].position.x * armors_msg->armors[i].position.x +
          armors_msg->armors[i].position.y * armors_msg->armors[i].position.y +
          armors_msg->armors[i].position.z * armors_msg->armors[i].position.z);
        if (distance < min_distance) {
          min_distance = distance;
          min_index = i;
        }
      }
      tracker_->init(
        std::make_shared<auto_aim_interfaces::msg::Armor>(
          armors_msg->armors[min_index]));
    } else {
      // If there are no armors, do nothing
      return;
    }
  } else {
    // Set dt
    filter_dt_ = (time - last_time_).seconds();
    // Update state
    tracker_->update(armors_msg, filter_dt_);
  }

  if (tracker_->tracker_state == EKFTracker::DETECTING ||
    tracker_->tracker_state == EKFTracker::LOST)
  {
    target_msg.tracking = false;
  } else if (tracker_->tracker_state == EKFTracker::TRACKING ||
    tracker_->tracker_state == EKFTracker::TEMP_LOST)
  {
    target_msg.tracking = true;
    target_msg.id = tracker_->tracking_id;
  }

  if (target_msg.tracking) {
    target_msg.position.x = tracker_->target_state(0);
    target_msg.position.y = tracker_->target_state(1);
    target_msg.position.z = tracker_->target_state(2);
    target_msg.velocity.x = tracker_->target_state(3);
    target_msg.velocity.y = tracker_->target_state(4);
    target_msg.velocity.z = tracker_->target_state(5);
  }

  if (enable_spin_observer_ && spin_observer_) {
    spin_observer_->max_jump_angle = get_parameter("spin_observer.max_jump_angle").as_double();
    spin_observer_->max_jump_period = get_parameter("spin_observer.max_jump_period").as_double();
    spin_observer_->allow_following_range =
      get_parameter("spin_observer.allow_following_range").as_double();

    spin_observer_->update(target_msg);
    spin_info_pub_->publish(spin_observer_->spin_info_msg);
  }

  target_pub_->publish(target_msg);

  last_time_ = time;

  if (debug_mode_) {
    publishMarkers(target_msg);
    RCLCPP_INFO_STREAM(this->get_logger(), "Tracker state:" << tracker_->tracker_state);
  }
}

void MultiProcessorNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  velocity_marker_.header = target_msg.header;

  if (target_msg.tracking) {
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position = target_msg.position;
    position_marker_.color.r = target_msg.suggest_fire ? 0. : 1.;

    velocity_marker_.action = visualization_msgs::msg::Marker::ADD;
    velocity_marker_.points.clear();
    velocity_marker_.points.emplace_back(target_msg.position);
    geometry_msgs::msg::Point arrow_end = target_msg.position;
    arrow_end.x += target_msg.velocity.x;
    arrow_end.y += target_msg.velocity.y;
    arrow_end.z += target_msg.velocity.z;
    velocity_marker_.points.emplace_back(arrow_end);
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    velocity_marker_.action = visualization_msgs::msg::Marker::DELETE;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(velocity_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::MultiProcessorNode)
