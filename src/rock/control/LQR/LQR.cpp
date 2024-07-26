#include "LQR.h"

#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/utils/find_path.h"
namespace controller {
namespace LQR {
LQR::LQR() : pnh_("~") {
  matrix_state_ = Matrix::Zero(state_size_, 1);
  matrix_a_ = Matrix::Zero(state_size_, state_size_);
  matrix_ad_ = Matrix::Zero(state_size_, state_size_);
  matrix_a_coeff_ = Matrix::Zero(state_size_, state_size_);
  matrix_b_ = Matrix::Zero(state_size_, 1);
  matrix_bd_ = Matrix::Zero(state_size_, 1);
  matrix_q_ = Matrix::Zero(state_size_, state_size_);
  matrix_reverse_q_ = Matrix::Zero(state_size_, state_size_);
  matrix_r_ = Matrix::Identity(1, 1);
  matrix_k_ = Matrix::Zero(1, state_size_);

  enable_leadlag_control_ = false;
  enable_lookahead_back_control_ = false;

  cb = boost::bind(&LQR::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);

  load_params();
  matrix_init();

  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  cyberc3::common::math::LpfCoefficients(model_configs_.ts, 10, &den, &num);

  digital_filter_.set_coefficients(den, num);

  leadlag_controller_.Init(leadlag_config_, model_configs_.ts);

  sub_localization_ = pnh_.subscribe("/localization/estimation", 1,
                                     &LQR::LocalizationCallback, this);
  sub_localtraj_ = pnh_.subscribe("/control/local_trajectory", 1,
                                  &LQR::TrajectoryCallback, this);
  sub_steerfeedback_ =
      pnh_.subscribe("/rock_can/steer_feedback", 1, &LQR::SteerCallback, this);
  sub_speedfeedback_ =
      pnh_.subscribe("/rock_can/speed_feedback", 1, &LQR::SpeedCallback, this);
  sub_stager_mode =
      pnh_.subscribe("/stager_mode", 1, &LQR::StagerModeCallback, this);
  pub_steer_cmd_ =
      pnh_.advertise<cyber_msgs::steercmd>("/rock_can/steer_command", 1);

  pub_point_marker_ =
      pnh_.advertise<visualization_msgs::MarkerArray>("/visualization/LQR", 1);

  pub_debug_msg_ = pnh_.advertise<cyber_msgs::LqrDebug>("/lqr_debug", 1);

  ros::MultiThreadedSpinner spinner(4);

  spinner.spin();
}

LQR::~LQR() {}

void LQR::load_params() {
  pnh_.param<double>("filter_param_", filter_param_, 0.6);
  pnh_.param<double>("wheelbase_", wheelbase_, 2.65);
  AINFO << "wheel_base : " << wheelbase_;
  pnh_.param<double>("k_wheel2steering", k_wheel2steering_, 15.58);
  pnh_.param<double>("max_delta_steering_degree", max_delta_steering_degree_,
                     10.0);
  pnh_.param<double>("max_steering_degree", max_steering_degree_, 430.0);
  pnh_.param<double>("wheel_zero_", wheel_zero_, 0.0);
  pnh_.param<double>("k_minimum_curvature_", k_minimum_curvature_, 0.1);

  pnh_.param<double>("cf", model_configs_.cf, 155494.663);
  pnh_.param<double>("cr", model_configs_.cr, 155494.663);
  pnh_.param<double>("eps", model_configs_.eps, 0.01);
  pnh_.param<double>("mass_fl", model_configs_.mass_fl, 520);
  pnh_.param<double>("mass_fr", model_configs_.mass_fr, 520);
  pnh_.param<double>("mass_rl", model_configs_.mass_rl, 520);
  pnh_.param<double>("mass_rr", model_configs_.mass_rr, 520);
  pnh_.param<double>("max_iteration", model_configs_.max_iteration, 300);
  pnh_.param<double>("ts", model_configs_.ts, 0.02);
  const double mass_front = model_configs_.mass_fl + model_configs_.mass_fr;
  const double mass_rear = model_configs_.mass_rl + model_configs_.mass_rr;
  mass_ = mass_front + mass_rear;
  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  pnh_.param<double>("weight_of_lat_error", weight_of_lat_error_, 0.05);
  pnh_.param<double>("weight_of_lat_error_rate", weight_of_lat_error_rate_,
                     0.0);
  pnh_.param<double>("weight_of_heading_error", weight_of_heading_error_, 1.0);
  pnh_.param<double>("weight_of_heading_error_rate",
                     weight_of_heading_error_rate_, 0.0);
  pnh_.param<double>("weight_of_control", weight_of_control_, 1.0);
  pnh_.param<double>("look_forward_dis", look_forward_distance_, 7.0);
  pnh_.param<double>("look_forward_time", look_forward_time_, 0.0);
  pnh_.param<double>("reverse_look_forward_dis", reverse_look_forward_distance_,
                     4.0);
  pnh_.param<double>("reverse_weight_of_lat_error", matrix_reverse_q_(0, 0),
                     0.05);
  pnh_.param<double>("reverse_weight_of_lat_error_rate",
                     matrix_reverse_q_(1, 1), 0.0);
  pnh_.param<double>("reverse_weight_of_heading_error", matrix_reverse_q_(2, 2),
                     3.0);
  pnh_.param<double>("reverse_weight_of_heading_error_rate",
                     matrix_reverse_q_(3, 3), 0.0);
  pnh_.param<bool>("enable_gain_scheduler", enable_gain_scheduler_, false);
  gain_ref_speeds_ = {0, 2.8, 4.2, 5.6, 8.3, 11.1};
  heading_error_gains_ = {1.0, 1.33, 2.0, 2.67, 3.0, 3.33};
  lat_error_gains_ = {1.0, 1.0, 1.0, 1.0, 0.8, 0.3};
  ACHECK(gain_ref_speeds_.size() == heading_error_gains_.size());
  ACHECK(gain_ref_speeds_.size() == lat_error_gains_.size());
}

void LQR::matrix_init() {
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (model_configs_.cf + model_configs_.cr) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * model_configs_.cf - lr_ * model_configs_.cr) / iz_;

  matrix_a_coeff_(1, 1) = -(model_configs_.cf + model_configs_.cr) / mass_;
  matrix_a_coeff_(1, 3) =
      (lr_ * model_configs_.cr - lf_ * model_configs_.cf) / mass_;
  matrix_a_coeff_(3, 1) =
      (lr_ * model_configs_.cr - lf_ * model_configs_.cf) / iz_;
  matrix_a_coeff_(3, 3) =
      -1.0 * (lf_ * lf_ * model_configs_.cf + lr_ * lr_ * model_configs_.cr) /
      iz_;

  matrix_b_(1, 0) = model_configs_.cf / mass_;
  matrix_b_(3, 0) = lf_ * model_configs_.cf / iz_;
  matrix_bd_ = matrix_b_ * model_configs_.ts;
}

void LQR::compute_lat_error() {
  double d_min = std::sqrt(PointDistanceSquare(
      target_traj_.points[current_index_], current_pose_.x, current_pose_.y));

  for (size_t i = current_index_; i < target_traj_.points.size(); ++i) {
    double d_temp = std::sqrt(PointDistanceSquare(
        target_traj_.points[i], current_pose_.x, current_pose_.y));
    if (d_temp < d_min) {
      d_min = d_temp;
      current_index_ = i;
    } else if (i - current_index_ > 20) {  // search extra 20 points
      break;
    }
  }

  cyber_msgs::LocalTrajPoint target_point;
  if (gear_ == vehicle_gear::R) {
    target_point = QueryNearestPointByPosition(current_pose_.x, current_pose_.y,
                                               reverse_look_forward_distance_);
  } else {
    double look_forward_dis =
        look_forward_distance_ +
        std::max(current_pose_.linear_v * look_forward_time_, 0.0);
    target_point = QueryNearestPointByPosition(current_pose_.x, current_pose_.y,
                                               look_forward_dis);
  }

  // target_point = QueryNearestPointByRelativeTime(query_relative_time_);

  const double dx = current_pose_.x - target_point.position.x;
  const double dy = current_pose_.y - target_point.position.y;

  const double cos_target_heading = std::cos(target_point.theta);
  const double sin_target_heading = std::sin(target_point.theta);
  lateral_error_ = cos_target_heading * dy - sin_target_heading * dx;
  ref_heading_ = target_point.theta;
  heading_error_ =
      cyberc3::common::math::NormalizeAngle(current_pose_.yaw - ref_heading_);

  double lookahead_station = 0.0;
  double lookback_station = 0.0;
  if (std::fabs(current_pose_.angular_v) > low_speed_bound_) {
    lookahead_station = lookahead_station_high_speed_;
    lookback_station = lookback_station_high_speed_;
  } else if (std::fabs(current_pose_.linear_v) <
             low_speed_bound_ - low_speed_window_) {
    lookahead_station = lookahead_station_low_speed_;
    lookback_station = lookback_station_low_speed_;
  } else {
    lookahead_station = cyberc3::common::math::lerp(
        lookahead_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookahead_station_high_speed_, low_speed_bound_,
        std::fabs(current_pose_.linear_v));
    lookback_station = cyberc3::common::math::lerp(
        lookback_station_low_speed_, low_speed_bound_ - low_speed_window_,
        lookback_station_high_speed_, low_speed_bound_,
        std::fabs(current_pose_.linear_v));
  }

  auto lookahead_point = QueryNearestPointByRelativeTime(
      target_point.t +
      lookahead_station / (std::max(std::fabs(current_pose_.linear_v), 0.1) *
                           std::cos(heading_error_)));

  if (gear_ == vehicle_gear::R) {
    lateral_error_feedback_ =
        lateral_error_ - lookahead_station * std::sin(heading_error_);
    heading_error_feedback_ = heading_error_;
  } else {
    lateral_error_feedback_ =
        lateral_error_ + lookahead_station * std::sin(heading_error_);
    heading_error_feedback_ = cyberc3::common::math::NormalizeAngle(
        heading_error_ + target_point.theta - lookahead_point.theta);
  }

  lateral_error_rate_ = current_pose_.linear_v * std::sin(heading_error_);
  if (gear_ == vehicle_gear::R) {
    heading_rate_ = -current_pose_.angular_v;
  } else {
    heading_rate_ = current_pose_.angular_v;
  }
  ref_heading_rate_ = target_point.kappa * target_point.velocity;

  heading_error_rate_ = heading_rate_ - ref_heading_rate_;

  ref_curvature_ = target_point.kappa;

  debug_msg_.ref_heading = ref_heading_;
  debug_msg_.ref_curvature = ref_curvature_;

#pragma region Visualization_Points in Rviz
  int point_id = 0;
  visualization_msgs::Marker point;
  visualization_msgs::MarkerArray vis_points;
  point.header.stamp = ros::Time::now();
  point.header.frame_id = "map";
  point.scale.x = 1.0;
  point.scale.y = 0.3;
  point.scale.z = 0.3;
  point.action = visualization_msgs::Marker::ADD;
  point.type = visualization_msgs::Marker::ARROW;
  point.lifetime = ros::Duration(0.15);
  point.id = point_id++;
  point.color.r = 0.0;
  point.color.g = 0.0;
  point.color.b = 1.0;
  point.color.a = 2.0;
  point.pose.position.x = current_pose_.x;
  point.pose.position.y = current_pose_.y;
  point.pose.position.z = 0.0;
  point.pose.orientation = tf::createQuaternionMsgFromYaw(current_pose_.yaw);
  vis_points.markers.emplace_back(point);

  point.header.stamp = ros::Time::now();
  point.header.frame_id = "map";
  point.scale.x = 1.0;
  point.scale.y = 0.3;
  point.scale.z = 0.3;
  point.action = visualization_msgs::Marker::ADD;
  point.type = visualization_msgs::Marker::ARROW;
  point.lifetime = ros::Duration(0.15);
  point.id = point_id++;
  point.color.r = 1.0;
  point.color.g = 0.0;
  point.color.b = 0.0;
  point.color.a = 1.0;
  point.pose.position.x = target_point.position.x;
  point.pose.position.y = target_point.position.y;
  point.pose.position.z = 0.0;
  point.pose.orientation = tf::createQuaternionMsgFromYaw(target_point.theta);
  vis_points.markers.emplace_back(point);

  // point.header.stamp = ros::Time::now();
  // point.header.frame_id = "map";
  // point.scale.x = 0.1;
  // point.scale.y = 0.1;
  // point.scale.z = 4.0;
  // point.action = visualization_msgs::Marker::ADD;
  // point.type = visualization_msgs::Marker::CYLINDER;
  // point.lifetime = ros::Duration(1.5);
  // point.id = point_id++;
  // point.color.r = 0.0;
  // point.color.g = 0.0;
  // point.color.b = 1.0;
  // point.color.a = 1.0;
  // point.pose.position.x = lookahead_point.position.x;
  // point.pose.position.y = lookahead_point.position.y;
  // point.pose.position.z = 0.0;
  // point.pose.orientation =
  //     tf::createQuaternionMsgFromYaw(lookahead_point.theta);
  // vis_points.markers.emplace_back(point);

  pub_point_marker_.publish(vis_points);

#pragma endregion
}

void LQR::update_matrix() {
  double cf = 0.0;
  double cr = 0.0;
  double v;
  double minimum_speed_protection = 0.1;

  // 倒档
  if (gear_ == vehicle_gear::R) {
    v = min(current_pose_.linear_v, -minimum_speed_protection);
    cf = -model_configs_.cf;
    cr = -model_configs_.cr;
    matrix_a_(0, 1) = 0.0;
    matrix_a_coeff_(0, 2) = 1.0;
    matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
  } else {
    v = max(current_pose_.linear_v, minimum_speed_protection);
    cf = model_configs_.cf;
    cr = model_configs_.cr;
    matrix_a_(0, 1) = 1.0;
    matrix_a_coeff_(0, 2) = 0.0;
    matrix_a_(0, 2) = 0.0;
  }
  // matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf + cr) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf - lr_ * cr) / iz_;

  matrix_a_coeff_(1, 1) = -(cf + cr) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr - lf_ * cf) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr - lf_ * cf) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf + lr_ * lr_ * cr) / iz_;

  matrix_b_(1, 0) = cf / mass_;
  matrix_b_(3, 0) = lf_ * cf / iz_;
  matrix_bd_ = matrix_b_ * model_configs_.ts;

  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;

  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - model_configs_.ts * 0.5 * matrix_a_).inverse() *
               (matrix_i + model_configs_.ts * 0.5 * matrix_a_);

  if (enable_gain_scheduler_) {
    double lat_err_gain = cyberc3::common::math::interpolate1d(
        current_pose_.linear_v, gain_ref_speeds_, lat_error_gains_);
    double heading_err_gain = cyberc3::common::math::interpolate1d(
        current_pose_.linear_v, gain_ref_speeds_, heading_error_gains_);
    matrix_q_(0, 0) = weight_of_lat_error_ * lat_err_gain;
    matrix_q_(1, 1) = weight_of_lat_error_rate_;
    matrix_q_(2, 2) = weight_of_heading_error_ * heading_err_gain;
    matrix_q_(3, 3) = weight_of_heading_error_rate_;
  } else {
    matrix_q_(0, 0) = weight_of_lat_error_;
    matrix_q_(1, 1) = weight_of_lat_error_rate_;
    matrix_q_(2, 2) = weight_of_heading_error_;
    matrix_q_(3, 3) = weight_of_heading_error_rate_;
  }
  matrix_r_(0, 0) = weight_of_control_;
}

void LQR::update_state() {
  matrix_state_(1, 0) = lateral_error_rate_;
  matrix_state_(3, 0) = heading_error_rate_;
  if (enable_lookahead_back_control_) {
    matrix_state_(0, 0) = lateral_error_feedback_;
    matrix_state_(2, 0) = heading_error_feedback_;
  } else {
    matrix_state_(0, 0) = lateral_error_;
    matrix_state_(2, 0) = heading_error_;
  }
  debug_msg_.lat_error = matrix_state_(0, 0);
  debug_msg_.lat_error_rate = matrix_state_(1, 0);
  debug_msg_.heading_error = matrix_state_(2, 0);
  debug_msg_.heading_error_rate = matrix_state_(3, 0);
}

void LQR::compute_control_command() {
  if (traj_updated_ == false) {
    AWARN << "Target trajectory is not received! Waiting for new trajectory!";
    return;
  }
  // update_target_point();
  compute_lat_error();

  update_state();

  update_matrix();

  // 倒档
  if (gear_ == vehicle_gear::R) {
    cyberc3::common::math::SolveLQRProblem(
        matrix_ad_, matrix_bd_, matrix_reverse_q_, matrix_r_,
        model_configs_.eps, model_configs_.max_iteration, &matrix_k_);
  } else {
    cyberc3::common::math::SolveLQRProblem(
        matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, model_configs_.eps,
        model_configs_.max_iteration, &matrix_k_);
  }

  const double steer_wheel_rad_feedback = -(matrix_k_ * matrix_state_)(0, 0);

  const double steer_wheel_degree_feedback =
      steer_wheel_rad_feedback * 180 / M_PI;
  AINFO << "wheel_degree_feedback(lqr output) ： "
        << steer_wheel_degree_feedback;
  const double steer_wheel_rad_forward = compute_feed_forward(ref_curvature_);
  const double steer_wheel_degree_forward =
      steer_wheel_rad_forward * 180 / M_PI;
  AINFO << "wheel_degree_forward(forward output) :  "
        << steer_wheel_degree_forward;

  double steer_wheel_rad_feedback_augment = 0.0;
  if (enable_leadlag_control_) {
    if (std::fabs(current_pose_.linear_v) < low_speed_bound_) {
      steer_wheel_rad_feedback_augment =
          leadlag_controller_.Control(-matrix_state_(0, 0), model_configs_.ts);
      if (std::fabs(current_pose_.linear_v) >
          low_speed_bound_ - low_speed_window_) {
        // Within the low-high speed transition window, linerly
        // interplolate the
        // augment control gain for "soft" control switch
        steer_wheel_rad_feedback_augment = cyberc3::common::math::lerp(
            steer_wheel_rad_feedback_augment,
            low_speed_bound_ - low_speed_window_, 0.0, low_speed_bound_,
            std::fabs(current_pose_.linear_v));
      }
    }
  }

  double steer_wheel_degree_feedback_augment =
      steer_wheel_rad_feedback_augment * 180 / M_PI;

  AINFO << "wheel_degree_augment(leadlag output) : "
        << steer_wheel_degree_feedback_augment;

  current_wheel_rad_ = steer_wheel_rad_feedback + steer_wheel_rad_forward +
                       steer_wheel_rad_feedback_augment;
  // current_wheel_degree_ = steer_wheel_degree_feedback +
  //                         steer_wheel_degree_forward +
  //                         steer_wheel_degree_feedback_augment + wheel_zero_;
  current_wheel_degree_ =
      steer_wheel_degree_feedback + steer_wheel_degree_forward;

  const double steer_limit =
      std::min(std::atan2(max_lat_acc_ * wheelbase_,
                          current_pose_.linear_v * current_pose_.linear_v) /
                   M_PI * 180 * k_wheel2steering_,
               max_steering_degree_);

  double final_steering_degree_raw =
      Wheel2Steering(current_wheel_degree_) + wheel_zero_;

  double final_steering_degree_limited = cyberc3::common::math::Clamp(
      final_steering_degree_raw, -steer_limit, steer_limit);

  double final_steering_degree_filtered =
      digital_filter_.Filter(final_steering_degree_limited);

  final_steering_degree_filtered = cyberc3::common::math::Clamp(
      final_steering_degree_filtered, -steer_limit, steer_limit);

  final_steering_degree_ = cyberc3::common::math::Clamp(
      final_steering_degree_filtered,
      former_steering_degree_ - max_delta_steering_degree_,
      former_steering_degree_ + max_delta_steering_degree_);

  former_steering_degree_ = final_steering_degree_;

#pragma region Show_some_details
  AINFO << "K0 : " << matrix_k_(0, 0);
  AINFO << "K1 : " << matrix_k_(0, 1);
  AINFO << "K2 : " << matrix_k_(0, 2);
  AINFO << "K3 : " << matrix_k_(0, 3);
  AINFO << "lat_error : " << lateral_error_;
  AINFO << "lat_error_rate : " << lateral_error_rate_;
  AINFO << "heading_error : " << heading_error_;
  AINFO << "heading_error_rate : " << heading_error_rate_;
  AINFO << "heading_rate : " << heading_rate_;
  AINFO << "ref_heading_rate : " << ref_heading_rate_;
  AINFO << "current_index_ : " << current_index_;
  AINFO << "max_index : " << target_traj_.points.size();
  AINFO << "contribution of lat_error : "
        << Wheel2Steering(-(matrix_k_(0, 0) * matrix_state_(0, 0)) * 180 /
                          M_PI);
  AINFO << "contribution of lat_error_rate : "
        << Wheel2Steering(-(matrix_k_(0, 1) * matrix_state_(1, 0)) * 180 /
                          M_PI);
  AINFO << "contribution of heading_error : "
        << Wheel2Steering(-(matrix_k_(0, 2) * matrix_state_(2, 0)) * 180 /
                          M_PI);
  AINFO << "contribution of heading_error_rate : "
        << Wheel2Steering(-(matrix_k_(0, 3) * matrix_state_(3, 0)) * 180 /
                          M_PI);
  AINFO << "final_output : " << final_steering_degree_;

  debug_msg_.lqr_feedback_angle = Wheel2Steering(steer_wheel_degree_feedback);
  debug_msg_.lqr_feedforward_angle = Wheel2Steering(steer_wheel_degree_forward);
  debug_msg_.lqr_raw_angle = final_steering_degree_raw;
  debug_msg_.lqr_filtered_angle = final_steering_degree_filtered;
  debug_msg_.lqr_final_angle = final_steering_degree_;
  debug_msg_.header.stamp = ros::Time::now();
  pub_debug_msg_.publish(debug_msg_);

#pragma endregion

  // update steer_cmd
  steer_cmd_.header.stamp = ros::Time::now();
  steer_cmd_.enable_auto_steer = true;
  steer_cmd_.is_updated = true;
  steer_cmd_.steer_cmd = final_steering_degree_;

  pub_steer_cmd_.publish(steer_cmd_);
}

double LQR::compute_feed_forward(double ref_curature) const {
  static bool enable_feedforward = false;
  if (!enable_feedforward && fabs(ref_curature) > 1.5 * k_minimum_curvature_)
    enable_feedforward = true;
  if (enable_feedforward && fabs(ref_curature) < 0.5 * k_minimum_curvature_)
    enable_feedforward = false;
  if (enable_feedforward) {
    const double kv = lr_ * mass_ / 2 / model_configs_.cf / wheelbase_ -
                      lf_ * mass_ / 2 / model_configs_.cr / wheelbase_;
    const double v = current_pose_.linear_v;
    double steer_angle_feedforwardterm;
    if (gear_ == vehicle_gear::R) {
      steer_angle_feedforwardterm = wheelbase_ * ref_curvature_;
    } else {
      steer_angle_feedforwardterm =
          (wheelbase_ * ref_curature + kv * v * v * ref_curature -
           matrix_k_(0, 2) *
               (lr_ * ref_curature - lf_ * mass_ * v * v * ref_curature / 2 /
                                         model_configs_.cr / wheelbase_));
    }
    return steer_angle_feedforwardterm;
  } else
    return 0.0;
}

void LQR::TrajectoryCallback(
    const cyber_msgs::LocalTrajList::ConstPtr &traj_ptr) {
  if (traj_ptr->points.size() == 0) {
    AWARN << "Empty trajectory received! Waiting for new trajectory!";
    traj_updated_ = false;
    return;
  }

  target_traj_ = *traj_ptr;
  current_index_ = 0;

  for (auto &point : target_traj_.points) {
    ComputeCOMPosition(point, lf_);
  }
  traj_updated_ = true;
  // std::cout << "total points in trajectory : " << traj_ptr->points.size()
  // << std::endl;
}

void LQR::LocalizationCallback(
    const cyber_msgs::LocalizationEstimate::ConstPtr &pose_in) {
  current_pose_.x = pose_in->pose.position.x;
  current_pose_.y = pose_in->pose.position.y;
  current_pose_.yaw = tf::getYaw(pose_in->pose.orientation);
  // if (pose_in->velocity.linear.x >= 0.0)
  //   current_pose_.linear_v = std::max(pose_in->velocity.linear.x, 1.0);
  // else
  //   current_pose_.linear_v = std::min(pose_in->velocity.linear.x, -1.0);
  current_pose_.linear_v = pose_in->velocity.linear.x;
  current_pose_.angular_v = pose_in->velocity.angular.z;
  // std_msgs::Float32 speed_msg;
  // speed_msg.data = pose_in->velocity.linear.x;
  // pub_current_speed_.publish(speed_msg);
  // 转换成车辆质心坐标
  current_pose_.x = current_pose_.x + lr_ * std::cos(current_pose_.yaw);
  current_pose_.y = current_pose_.y + lr_ * std::sin(current_pose_.yaw);

  compute_control_command();
}

void LQR::SteerCallback(const cyber_msgs::SteerFeedback::ConstPtr &msg) {
  double steer = msg->SteerAngle;
  wheel_degree_feedback_ = Steering2Wheel(steer);
  wheel_rad_feedback_ = current_wheel_degree_ * M_PI / 180.0;
}

void LQR::SpeedCallback(const cyber_msgs::SpeedFeedback::ConstPtr &msg) {
  gear_ = msg->gear;
}

void LQR::StagerModeCallback(const std_msgs::Int8 mode_in) {
  stager_mode_ = mode_in.data;
}

void LQR::configCallback(lqr::lqrConfig &config, uint32_t level) {
  weight_of_lat_error_ = config.weight_of_lat_error;
  weight_of_lat_error_rate_ = config.weight_of_lat_error_rate;
  weight_of_heading_error_ = config.weight_of_heading_error;
  weight_of_heading_error_rate_ = config.weight_of_heading_error_rate;
  weight_of_control_ = config.weight_of_control;
  filter_param_ = config.filter_param_;
  look_forward_distance_ = config.look_forward_dis;
  look_forward_time_ = config.look_forward_time;
  k_minimum_curvature_ = config.k_minimum_curvature_;
  std::cout << "weight_of_lat_error\t" << config.weight_of_lat_error
            << std::endl;
  std::cout << "weight_of_lat_error_rate\t" << config.weight_of_lat_error_rate
            << std::endl;
  std::cout << "weight_of_heading_error\t" << config.weight_of_heading_error
            << std::endl;
  std::cout << "weight_of_heading_error_rate\t"
            << config.weight_of_heading_error_rate << std::endl;
  std::cout << "weight_of_control\t" << config.weight_of_control << std::endl;
  std::cout << "filter_param\t" << config.filter_param_ << std::endl;
  std::cout << "k_minimum_curvature\t" << config.k_minimum_curvature_
            << std::endl;
  std::cout << "look_forward_dis\t" << config.look_forward_dis << std::endl;
}

double LQR::PointDistanceSquare(const cyber_msgs::LocalTrajPoint &point,
                                const double x, const double y) {
  const double dx = point.position.x - x;
  const double dy = point.position.y - y;
  return dx * dx + dy * dy;
}

cyber_msgs::LocalTrajPoint LQR::QueryNearestPointByRelativeTime(
    const double t) {
  auto func_comp = [](const cyber_msgs::LocalTrajPoint &point,
                      const double relative_time) {
    return point.t < relative_time;
  };

  auto it_low = std::lower_bound(target_traj_.points.begin(),
                                 target_traj_.points.end(), t, func_comp);

  if (it_low == target_traj_.points.begin()) {
    return target_traj_.points.front();
  }

  if (it_low == target_traj_.points.end()) {
    return target_traj_.points.back();
  }

  auto it_lower = it_low - 1;
  if (it_low->t - t < t - it_lower->t) {
    return *it_low;
  }
  return *it_lower;
}

cyber_msgs::LocalTrajPoint LQR::QueryNearestPointByPosition(
    const double x, const double y, const double distance) {
  double d_min = std::fabs(std::sqrt(PointDistanceSquare(
                               target_traj_.points[current_index_], x, y)) -
                           distance);
  size_t index_min = current_index_;

  for (size_t i = current_index_; i < target_traj_.points.size(); ++i) {
    double d_temp =
        std::fabs(std::sqrt(PointDistanceSquare(target_traj_.points[i], x, y)) -
                  distance);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }
  return target_traj_.points[index_min];
}

void LQR::ComputeCOMPosition(cyber_msgs::LocalTrajPoint &point,
                             double rear_to_com_distance) {
  point.position.x += rear_to_com_distance * std::cos(point.theta);
  point.position.y += rear_to_com_distance * std::sin(point.theta);
}

}  // namespace LQR
}  // namespace controller

int main(int argc, char **argv) {
  ros::init(argc, argv, "LQR");

  // if (!google::IsGoogleLoggingInitialized()) {
  google::InitGoogleLogging("lqr_controller");
  // }

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  FLAGS_log_prefix = true;
  FLAGS_logbufsecs = 0;
  FLAGS_max_log_size = 1024;
  FLAGS_log_dir = expand_catkin_ws("/log/");

  controller::LQR::LQR lqr_controller;
  return 0;
}
