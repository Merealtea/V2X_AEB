/**
 * @file track_manager.cc
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief
 * @version 0.1
 * @date 2023-04-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "track_manager.h"

#include <algorithm>

#include "bezier.h"
#include "common/log.h"
#include "common/math/filters/digital_filter_coefficients.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/struct/pnc_point.h"
#include "discretized_trajectory.h"
#include "julia_mpc_adapter.h"
#include "track_smoother.h"

namespace cyberc3 {
namespace planning {

TrackManager::TrackManager(const TrackManagerConfig &config) : config_(config) {
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::math::LpfCoefficients(0.02, config_.v2v_config.acc_filter_cutoff_freq,
                                &den, &num);
  v2v_acc_filter_.set_coefficients(den, num);
  common::math::LpfCoefficients(0.02, config_.distance_filter_cutoff_freq, &den,
                                &num);
  distance_feedback_filter_.set_coefficients(den, num);
  track_smoother_.Init(config_.smooth_config);
  smoothed_track_updated_ = false;
  target_lost_ = true;
  ref_curvatures_ = {0.0036, 0.004, 0.005, 0.006, 0.008, 0.01, 0.02, 0.03,
                     0.04,   0.05,  0.06,  0.07,  0.08,  0.09, 0.1,  0.15};
  constexpr double wheel_base = 2.65;
  ref_steer_angles_.resize(ref_curvatures_.size());
  std::transform(ref_curvatures_.begin(), ref_curvatures_.end(),
                 ref_steer_angles_.begin(),
                 [&wheel_base](double x) { return x * wheel_base; });
  ref_speed_limits_ = {15.67, 14.81, 13.14, 12.2, 10.9, 10.0, 7.4,  6.32,
                       5.71,  5.32,  4.77,  4.35, 4.0,  2.71, 2.47, 1.78};
  ref_acc_limits_ = {4.0, 3.75, 3.5, 3.25, 3.0, 2.75, 2.5, 2.25,
                     2.0, 1.75, 1.5, 1.25, 1.0, 0.75, 0.5, 0.25};
  if (config_.enable_mpc_adapter) {
    speed_predictor_ = std::make_shared<KalmanSpeedPredictor>();
    mpc_adapter_ =
        std::make_unique<JuliaMPCAdapter>(config.dis_config, speed_predictor_);
  }
}

void TrackManager::FeedTargetPose(common::TrajectoryPoint &target_pose) {
  if (target_lost_ || ego_state_.timestamp < 1) return;

  // update target state
  target_state_.timestamp_perception = target_pose.relative_time();
  target_state_.x = target_pose.path_point().x();
  target_state_.y = target_pose.path_point().y();
  target_state_.theta = target_pose.path_point().theta();
  UpdateTargetTrack(target_pose);
  target_state_.s = target_track_.back().path_point().s();
  if (config_.enable_mpc_adapter) {
    speed_predictor_->FeedTargetPerception(target_state_);
  }
  UpdateSmoothedTrack();
}

void TrackManager::FeedEgoPose(const common::TrajectoryPoint &ego_pose) {
  {
    std::lock_guard<std::mutex> lock(localization_mutex_);
    ego_state_.timestamp = ego_pose.relative_time();
    ego_state_.x = ego_pose.path_point().x();
    ego_state_.y = ego_pose.path_point().y();
    ego_state_.theta = ego_pose.path_point().theta();
    ego_state_.speed = ego_pose.v();
    ego_state_.acc_x = ego_pose.a();
    ego_state_.yaw_rate = ego_pose.steer();
  }
  UpdateEgoTrack(ego_pose);
  UpdateNearestIndex(ego_pose);
  UpdateEgoS(ego_pose);
  if (config_.enable_mpc_adapter) {
    mpc_adapter_->UpdateEgoState(ego_state_);
  }
}

void TrackManager::FeedEgoSteerAngle(const float steer_angle) {
  std::lock_guard<std::mutex> lock(localization_mutex_);
  ego_state_.steer_angle = steer_angle;
}

void TrackManager::FeedTargetV2V(const V2VPacket &v2v_msg) {
  if (v2v_msg.is_updated) {
    std::lock_guard<std::mutex> lock(v2v_mutex_);
    target_state_.timestamp = v2v_msg.timestamp;
    target_state_.speed = v2v_msg.speed;
    // target_state_.acc_x = v2v_acc_filter_.Filter(v2v_msg.acc_x);
    target_state_.acc_x = v2v_msg.acc_x;
    target_state_.yaw_rate = v2v_msg.yaw_rate;
    target_state_.steer_angle = v2v_msg.steer_angle;
    if (config_.enable_mpc_adapter) {
      speed_predictor_->FeedTargetV2V(v2v_msg);
    }
  } else {
    AERROR << "V2V msg is not updated! please check front vehicle driver!";
  }
}

void TrackManager::FeedTargetLostState(const bool target_lost) {
  if (!target_lost_ && target_lost) {
    AERROR << "Target lost!!! Reset track manager now!!!";
    Reset();
  }
  target_lost_ = target_lost;
}

void TrackManager::FeedTargetPerceptionSpeed(const float speed) {
  target_state_.speed_perception = speed;
}

void TrackManager::FeedAutoDriveMode(const bool auto_drive_mode) {
  if (config_.enable_mpc_adapter) {
    mpc_adapter_->SetAutoDriveMode(auto_drive_mode);
  }
}

void TrackManager::UpdateTargetTrack(common::TrajectoryPoint target_pose) {
  std::lock_guard<std::mutex> lock(target_track_mutex_);
  if (target_track_.empty()) {
    common::TrajectoryPoint start_pose;
    start_pose.mutable_path_point()->set_x(ego_state_.x);
    start_pose.mutable_path_point()->set_y(ego_state_.y);
    start_pose.mutable_path_point()->set_theta(ego_state_.theta);

    std::vector<common::math::Vec2d> bezier_points =
        GenerateBezierPoints(start_pose, target_pose);
    double sum_s = 0;
    for (int i = 0; i < bezier_points.size(); i++) {
      common::TrajectoryPoint traj_point;
      traj_point.mutable_path_point()->set_x(bezier_points[i].x());
      traj_point.mutable_path_point()->set_y(bezier_points[i].y());
      if (i < bezier_points.size() - 1) {
        traj_point.mutable_path_point()->set_theta(
            std::atan2(bezier_points[i + 1].y() - bezier_points[i].y(),
                       bezier_points[i + 1].x() - bezier_points[i].x()));
      } else {
        traj_point.mutable_path_point()->set_theta(
            target_pose.path_point().theta());
      }
      if (i > 0) {
        sum_s += bezier_points[i].DistanceTo(bezier_points[i - 1]);
      }
      traj_point.mutable_path_point()->set_s(sum_s);
      target_track_.push_back(std::move(traj_point));
    }
  }
  common::TrajectoryPoint last_pose = target_track_.back();
  common::math::Vec2d last_pose_vec(last_pose.path_point().x(),
                                    last_pose.path_point().y());
  common::math::Vec2d target_pose_vec(target_pose.path_point().x(),
                                      target_pose.path_point().y());
  const double d_s = last_pose_vec.DistanceTo(target_pose_vec);
  const double dot = (target_pose_vec - last_pose_vec)
                         .InnerProd({std::cos(last_pose.path_point().theta()),
                                     std::sin(last_pose.path_point().theta())});
  target_pose.set_v(target_state_.speed);
  target_pose.set_a(target_state_.acc_x);
  target_pose.set_steer(target_state_.yaw_rate);
  target_pose.mutable_path_point()->set_s(last_pose.path_point().s() + d_s);
  if (d_s >= config_.track_interval && dot > 0) {
    const double delta_s = d_s / std::ceil(d_s / config_.track_interval);
    const double start_s = last_pose.path_point().s() + delta_s;
    const double end_s = target_pose.path_point().s() + delta_s * 1e-6;
    for (double s = start_s; s < end_s; s += delta_s) {
      target_track_.push_back(
          common::math::InterpolateUsingLinearApproximationByS(last_pose,
                                                               target_pose, s));
    }
  }
  while (target_track_.size() > config_.max_track_size) {
    target_track_.pop_front();
    nearest_index_--;
  }
}

void TrackManager::UpdateSmoothedTrack() {
  std::lock_guard<std::mutex> lock(smoothed_track_mutex_);
  // return if last smoothed track is not published
  if (smoothed_track_updated_) return;
  // return if taget_track is too short
  if (target_track_.size() - nearest_index_ < 5) return;
  if (!smoothed_track_.empty()) {
    if (smoothed_track_.back().path_point().s() -
            smoothed_track_[smoothed_nearest_index_].path_point().s() >
        config_.smooth_config.resmooth_threshold) {
      return;
    }
  }
  common::math::Vec2d ego_vec(ego_state_.x, ego_state_.y);
  SplineSmoothResult spline_result{};
  smoothed_track_updated_ = track_smoother_.SmoothPlatoonTrack(
      target_track_, nearest_index_, ego_vec, &smoothed_track_, &spline_result);
  if (smoothed_track_updated_) {
    smoothed_nearest_index_ = 0;
    if (config_.enable_mpc_adapter) {
      mpc_adapter_->UpdateSpline(spline_result, smoothed_track_);
    }
  }
}

void TrackManager::UpdateEgoTrack(common::TrajectoryPoint ego_pose) {
  std::lock_guard<std::mutex> lock(ego_track_mutex_);
  if (ego_track_.empty()) {
    ego_track_.push_back(std::move(ego_pose));
    return;
  }
  common::math::Vec2d last_pose_vec(ego_track_.back().path_point().x(),
                                    ego_track_.back().path_point().y());
  common::math::Vec2d ego_pose_vec(ego_pose.path_point().x(),
                                   ego_pose.path_point().y());
  const double d_s = ego_pose_vec.DistanceTo(last_pose_vec);
  if (d_s > config_.track_interval) {
    ego_pose.mutable_path_point()->set_s(ego_track_.back().path_point().s() +
                                         d_s);
    ego_track_.push_back(std::move(ego_pose));
    while (ego_track_.size() > config_.max_track_size) {
      ego_track_.pop_front();
    }
  }
}

void TrackManager::UpdateNearestIndex(const common::TrajectoryPoint &ego_pose) {
  {
    std::lock_guard<std::mutex> lock(target_track_mutex_);
    if (target_track_.empty()) return;
    nearest_index_ = target_track_.QueryNearestPointWithStartIndex(
        {ego_pose.path_point().x(), ego_pose.path_point().y()}, nearest_index_);
  }
  {
    std::lock_guard<std::mutex> lock(smoothed_track_mutex_);
    if (smoothed_track_.empty()) return;
    smoothed_nearest_index_ = smoothed_track_.QueryNearestPointWithStartIndex(
        {ego_pose.path_point().x(), ego_pose.path_point().y()}, 0);
  }
}

bool TrackManager::GetEgoTrack(DiscretizedTrajectory *track_out) const {
  if (ego_track_.empty()) return false;
  std::lock_guard<std::mutex> lock(ego_track_mutex_);
  *track_out = ego_track_;
  return true;
}

bool TrackManager::GetTargetTrack(DiscretizedTrajectory *track_out) const {
  if (target_lost_ || target_track_.empty()) return false;
  std::lock_guard<std::mutex> lock(target_track_mutex_);
  *track_out = target_track_;
  return true;
}

// update smoothed track only when target track is updated
bool TrackManager::GetFrontTrack(DiscretizedTrajectory *track_out) {
  std::lock_guard<std::mutex> lock(smoothed_track_mutex_);
  if (!smoothed_track_updated_) return false;
  *track_out = smoothed_track_;
  smoothed_track_updated_ = false;
  return true;
}

bool TrackManager::GetControlCommand(const double cur_time,
                                     PlatoonControlCommand *cmd,
                                     PlatoonControlDebug *debug) {
  std::unique_lock<std::mutex> lock1(localization_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> lock2(v2v_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> lock3(target_track_mutex_, std::defer_lock);
  std::lock(lock1, lock2, lock3);
  if (target_lost_) {
    AERROR << "Target lost, please rematch!!!";
    return false;
  }
  if (cur_time - ego_state_.timestamp > config_.timeout_threshold) {
    AERROR << "Localization Timeout!";
    return false;
  }
  if (target_track_.empty()) {
    AERROR << "Target Pose Not Received!";
    return false;
  }
  // prepare debug info
  debug->speed_ego = ego_state_.speed;
  debug->speed_front_v2v = target_state_.speed;
  debug->speed_front_perception = target_state_.speed_perception;
  debug->speed_error = target_state_.speed - ego_state_.speed;
  debug->distance_expect =
      CalculateExpectDistance(ego_state_.speed, target_state_.speed);
  common::math::Vec2d ego_vec(ego_state_.x, ego_state_.y);
  common::math::Vec2d target_vec(target_track_.back().path_point().x(),
                                 target_track_.back().path_point().y());

  bool v2v_updated = std::abs(cur_time - target_state_.timestamp) <
                     config_.v2v_config.delay_threshold;
  double target_predict_distance = 0.0;
  if (v2v_updated) {
    target_predict_distance =
        target_state_.speed * (cur_time - target_state_.timestamp_perception);
  }
  double ego_predict_distance =
      ego_state_.speed * (cur_time - ego_state_.timestamp);
  debug->distance_feedback_perception = ego_vec.DistanceTo(target_vec) +
                                        target_predict_distance -
                                        config_.dis_config.dis_offset;
  const double distance_feedback_curve =
      target_state_.s + target_predict_distance -
      (ego_state_.s + ego_predict_distance) - config_.dis_config.dis_offset;
  // debug->distance_feedback_curve =
  //     distance_feedback_filter_.Filter(distance_feedback_curve);
  debug->distance_feedback_curve = distance_feedback_curve;
  debug->distance_error =
      debug->distance_feedback_curve - debug->distance_expect;
  debug->acc_front_v2v = target_state_.acc_x;
  CalculateSpeedAccLimitByCurvature(&debug->speed_limit_curvature,
                                    &debug->acc_limit_curvature);
  debug->speed_limit_steer = cyberc3::common::math::interpolate1d(
      std::abs(ego_state_.steer_angle), ref_steer_angles_, ref_speed_limits_);
  debug->acc_limit_steer = cyberc3::common::math::interpolate1d(
      std::abs(ego_state_.steer_angle), ref_steer_angles_, ref_acc_limits_);

  // calculate lateral and heading error
  {
    std::lock_guard<std::mutex> lock(smoothed_track_mutex_);
    const auto nearest_idx = smoothed_track_.QueryNearestPoint(ego_vec);
    const auto &nearest_point = smoothed_track_[nearest_idx];
    common::math::Vec2d ref_point(nearest_point.path_point().x(),
                                  nearest_point.path_point().y());
    auto ref_direction = common::math::Vec2d::CreateUnitVec2d(
        nearest_point.path_point().theta());
    debug->lateral_error = (ego_vec - ref_point).CrossProd(ref_direction);
    debug->heading_error = common::math::NormalizeAngle(
        nearest_point.path_point().theta() - ego_state_.theta);
  }

  // prepare  control command
  cmd->distance_expect = debug->distance_expect;
  cmd->distance_feedback = debug->distance_feedback_curve;
  if (v2v_updated) {
    cmd->speed_ref = target_state_.speed;
    // use v2v acc only if greater than threshold
    if (std::abs(target_state_.acc_x) > config_.v2v_config.use_acc_threshold) {
      cmd->acc_ref = target_state_.acc_x;
    } else {
      cmd->acc_ref = 0;
    }
  } else {
    cmd->speed_ref = target_state_.speed_perception;
    cmd->acc_ref = 0;
    AWARN << "V2V communication timeout, time delay: "
          << (cur_time - target_state_.timestamp) * 1000 << " ms";
  }
  cmd->speed_limit_curvature = debug->speed_limit_curvature;
  cmd->acc_limit_curvature = debug->acc_limit_curvature;
  cmd->speed_limit_steer = debug->speed_limit_steer;
  cmd->acc_limit_steer = debug->acc_limit_steer;
  AINFO << "cmd"
        << " = " << cmd->speed_ref << " + "
        << "0.7"
        << "*(" << cmd->distance_feedback << "-" << cmd->distance_expect << ")+"
        << cmd->acc_ref;
  return true;
}

VehicleState TrackManager::GetEgoState() const { return ego_state_; }

double TrackManager::CalculateExpectDistance(double /*ego_speed*/,
                                             double target_speed) const {
  double distance_expect = config_.dis_config.dis_base +
                           config_.dis_config.dis_speed_factor * target_speed;
  distance_expect = std::min(distance_expect, config_.dis_config.dis_max);
  distance_expect = std::max(distance_expect, config_.dis_config.dis_min);
  return distance_expect;
}

void TrackManager::CalculateSpeedAccLimitByCurvature(double *speed_limit,
                                                     double *acc_limit) const {
  std::lock_guard<std::mutex> lock(smoothed_track_mutex_);
  if (smoothed_track_.empty()) {
    AWARN << "smoothed_track_ is empty!";
    return;
  }
  constexpr double forward_consider_distance = 1.5;
  constexpr double backward_consider_distance = 1.0;
  const double currest_s =
      smoothed_track_[smoothed_nearest_index_].path_point().s();
  double sum_curvature = 0.0;
  size_t point_count = 0;
  for (int i = smoothed_nearest_index_;
       i < smoothed_track_.size() && smoothed_track_[i].path_point().s() <
                                         currest_s + forward_consider_distance;
       ++i) {
    sum_curvature += std::abs(smoothed_track_[i].path_point().kappa());
    point_count++;
  }
  for (int i = smoothed_nearest_index_ - 1;
       i >= 0 && smoothed_track_[i].path_point().s() >
                     currest_s - backward_consider_distance;
       --i) {
    sum_curvature += std::abs(smoothed_track_[i].path_point().kappa());
    point_count++;
  }
  const double mean_curvature = sum_curvature / point_count;
  *speed_limit = cyberc3::common::math::interpolate1d(
      mean_curvature, ref_curvatures_, ref_speed_limits_);
  *acc_limit = cyberc3::common::math::interpolate1d(
      mean_curvature, ref_curvatures_, ref_acc_limits_);
}

void TrackManager::Reset() {
  std::unique_lock<std::mutex> lock1(localization_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> lock2(v2v_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> lock3(target_track_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> lock4(ego_track_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> lock5(smoothed_track_mutex_, std::defer_lock);
  std::lock(lock1, lock2, lock3, lock4, lock5);
  target_lost_ = true;
  smoothed_track_updated_ = false;
  ego_track_.clear();
  target_track_.clear();
  smoothed_track_.clear();
  v2v_acc_filter_.reset_values();
  distance_feedback_filter_.reset_values();
  nearest_index_ = 0;
  smoothed_nearest_index_ = 0;
  target_state_ = VehicleState();
  ego_state_ = VehicleState();
  track_smoother_.Reset();
  if (config_.enable_mpc_adapter) {
    mpc_adapter_->Reset();
  }
}

std::vector<common::math::Vec2d> TrackManager::GenerateBezierPoints(
    const common::TrajectoryPoint &start_pose,
    const common::TrajectoryPoint &end_pose) const {
  using Vec2d = common::math::Vec2d;
  constexpr double start_ratio = 0.5;
  constexpr double end_ratio = 0.5;
  Vec2d pt1(start_pose.path_point().x(), start_pose.path_point().y());
  Vec2d pt4(end_pose.path_point().x(), end_pose.path_point().y());
  const double dis_start2end = pt1.DistanceTo(pt4);
  Vec2d pt2 = pt1 + Vec2d::CreateUnitVec2d(start_pose.path_point().theta()) *
                        dis_start2end * start_ratio;
  Vec2d pt3 = pt4 - Vec2d::CreateUnitVec2d(end_pose.path_point().theta()) *
                        dis_start2end * end_ratio;
  BezierCurve<Vec2d> bezier_curve({pt1, pt2, pt3, pt4});
  double t_interval = 1.0 / std::ceil(dis_start2end / config_.track_interval);
  std::vector<common::math::Vec2d> out_points;
  for (double t = 0.0; t < 1.0; t += t_interval) {
    out_points.emplace_back(bezier_curve(t));
  }
  out_points.emplace_back(pt4);
  return out_points;
}

void TrackManager::UpdateEgoS(const common::TrajectoryPoint &ego_pose) {
  std::unique_lock<std::mutex> lock1(target_track_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> lock2(localization_mutex_, std::defer_lock);
  std::lock(lock1, lock2);
  if (target_track_.empty() || nearest_index_ < 0 ||
      nearest_index_ >= target_track_.size()) {
    AWARN << "nearest index is invalid! " << nearest_index_;
    return;
  }
  if (nearest_index_ == 0 || nearest_index_ == target_track_.size() - 1) {
    ego_state_.s = target_track_[nearest_index_].path_point().s();
    return;
  }
  common::math::Vec2d ego_point = {ego_pose.path_point().x(),
                                   ego_pose.path_point().y()};
  common::math::Vec2d nearest_point{
      target_track_[nearest_index_].mutable_path_point()->x(),
      target_track_[nearest_index_].mutable_path_point()->y()};
  common::math::Vec2d last_point{
      target_track_[nearest_index_ - 1].mutable_path_point()->x(),
      target_track_[nearest_index_ - 1].mutable_path_point()->y()};
  common::math::Vec2d next_point{
      target_track_[nearest_index_ + 1].mutable_path_point()->x(),
      target_track_[nearest_index_ + 1].mutable_path_point()->y()};

  const double r1 =
      (ego_point - last_point).InnerProd(nearest_point - last_point) /
      (nearest_point - last_point).LengthSquare();
  if (r1 < 1.0) {
    ego_state_.s =
        (1 - r1) * target_track_[nearest_index_ - 1].path_point().s() +
        r1 * target_track_[nearest_index_].path_point().s();
  } else {
    const double r2 =
        (ego_point - nearest_point).InnerProd(next_point - nearest_point) /
        (next_point - nearest_point).LengthSquare();
    ego_state_.s = (1 - r2) * target_track_[nearest_index_].path_point().s() +
                   r2 * target_track_[nearest_index_ + 1].path_point().s();
  }
}

}  // namespace planning
}  // namespace cyberc3
