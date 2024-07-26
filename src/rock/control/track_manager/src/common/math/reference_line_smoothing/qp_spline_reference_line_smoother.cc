/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/
#include "qp_spline_reference_line_smoother.h"

#include <algorithm>
#include <utility>
#include <chrono>

#include "common/log.h"
#include "common/math/vec2d.h"
// #include "modules/planning/math/curve_math.h"
#include "common/struct/pnc_point.h"
#include "memory"
#include "spline_smoothing/osqp_spline_2d_solver.h"

namespace cyberc3 {
namespace planning {

QpSplineReferenceLineSmoother::QpSplineReferenceLineSmoother(
    const ReferenceLineSmootherConfig& config, double keepbehind_distance)
    : config_(config), keepbehind_distance_(keepbehind_distance) {
  spline_solver_ =
      std::make_unique<OsqpSpline2dSolver>(t_knots_, config.spline_order);
}

void QpSplineReferenceLineSmoother::Clear() { t_knots_.clear(); }

void QpSplineReferenceLineSmoother::Reset() {
  t_knots_.clear();
  anchor_points_.clear();
  last_smoothed_track_.clear();
  spline_result_ = SplineSmoothResult();
  spline_solver_ =
      std::make_unique<OsqpSpline2dSolver>(t_knots_, config_.spline_order);
}

bool QpSplineReferenceLineSmoother::UpdateRawTrajectory(
    const DiscretizedTrajectory& raw_track, const size_t& nearest_index,
    const common::math::Vec2d& ego_vec) {
  DiscretizedTrajectory raw_path;
  double last_smooth_end_s = 0;
  if (!last_smoothed_track_.empty()) {
    const size_t last_track_neareast_index =
        last_smoothed_track_.QueryNearestPoint(ego_vec);
    const double ego_s =
        last_smoothed_track_[last_track_neareast_index].path_point().s();
    for (const auto& point : last_smoothed_track_) {
      if (point.path_point().s() > ego_s - keepbehind_distance_) {
        raw_path.push_back(point);
      }
    }
    if (raw_path.empty()) {
      return false;
    }
    auto start_point = raw_path.back();
    last_smooth_end_s = raw_path.back().path_point().s();
    // find raw path points after last smoothed points
    const size_t raw_track_next_index =
        raw_track.QueryNearestPointWithStartIndex(
            {start_point.path_point().x(), start_point.path_point().y()},
            nearest_index);
    for (size_t i = raw_track_next_index; i < raw_track.size(); ++i) {
      // change point s value to avoid duplicate s value from different track
      common::TrajectoryPoint point = raw_track[i];
      point.mutable_path_point()->set_s(
          raw_path.back().path_point().s() +
          std::hypot(
              point.path_point().x() - raw_path.back().path_point().x(),
              point.path_point().y() - raw_path.back().path_point().y()));
      raw_path.push_back(std::move(point));
    }
  } else {
    raw_path = raw_track;
  }

  if (raw_path.empty()) {
    return false;
  }

  constexpr double kSmoothDeltaS = 0.5;
  const double start_s = raw_path.front().path_point().s();
  const double end_s = raw_path.back().path_point().s();
  double path_length = end_s - start_s;
  ADEBUG << "Current path_length is: " << path_length;
  const double delta_s = path_length / std::ceil(path_length / kSmoothDeltaS);
  // update smooth anchor points
  anchor_points_.clear();
  for (double s = start_s; s <= end_s; s += delta_s) {
    const auto point2d = raw_path.EvaluateByS(s);
    AnchorPoint anchor_point;
    anchor_point.x = point2d.path_point().x();
    anchor_point.y = point2d.path_point().y();
    anchor_point.s = point2d.path_point().s();
    anchor_point.theta = point2d.path_point().theta();
    anchor_point.lateral_bound = 0.25;
    anchor_point.longitudinal_bound = 0.25;
    if (s <= last_smooth_end_s) {
      anchor_point.lateral_bound = 1e-3;
      anchor_point.longitudinal_bound = 1e-3;
    }
    anchor_points_.emplace_back(std::move(anchor_point));
  }
  anchor_points_[anchor_points_.size() - 1].lateral_bound = 0.01;
  anchor_points_[anchor_points_.size() - 1].longitudinal_bound = 0.01;
  anchor_points_[anchor_points_.size() - 2].lateral_bound = 0.01;
  anchor_points_[anchor_points_.size() - 2].longitudinal_bound = 0.01;
  return true;
}

bool QpSplineReferenceLineSmoother::Smooth(
    DiscretizedTrajectory* const smoothed_track) {
  Clear();
  if (!Sampling()) {
    AERROR << "Fail to sample reference line smoother points!";
    return false;
  }

  spline_solver_->Reset(t_knots_, config_.spline_order);

  if (!AddConstraint()) {
    AERROR << "Add constraint for spline smoother failed";
    return false;
  }

  if (!AddKernel()) {
    AERROR << "Add kernel for spline smoother failed.";
    return false;
  }

  if (!Solve()) {
    AERROR << "Solve spline smoother problem failed";
    return false;
  }

  // mapping spline to reference line point
  const double start_t = t_knots_.front();
  const double end_t = t_knots_.back();

  const double scale = (anchor_points_.back().s - anchor_points_.front().s) /
                       (t_knots_.back() - t_knots_.front());
  const double resolution = config_.point_invterval / scale;
  double t = start_t;
  // std::vector<ReferencePoint> ref_points;
  const auto& spline = spline_solver_->spline();
  // check if the spline is valid
  auto xy = spline(start_t);
  if (std::abs(xy.first + ref_x_ - anchor_points_.front().x) > 1 ||
      std::abs(xy.second + ref_y_ - anchor_points_.front().y) > 1) {
    AERROR << "Fail to check spline validity at t = 0: " << xy.first << ", "
           << xy.second;
    return false;
  }
  smoothed_track->clear();
  const double final_resolution =
      (end_t - start_t) / std::ceil((end_t - start_t) / resolution);
  double sum_distance = 0.0;
  for (double t = start_t; t < end_t + final_resolution * 1e-6;
       t += final_resolution) {
    common::TrajectoryPoint traj_point;
    common::PathPoint* ref_point = traj_point.mutable_path_point();
    auto xy = spline(t);
    const double x_dot = spline.DerivativeX(t);
    const double y_dot = spline.DerivativeY(t);
    const double dot_norm = std::sqrt(x_dot * x_dot + y_dot * y_dot);
    const double x_dot_dot = spline.SecondDerivativeX(t);
    const double y_dot_dot = spline.SecondDerivativeY(t);
    ref_point->set_x(xy.first + ref_x_);
    ref_point->set_y(xy.second + ref_y_);
    ref_point->set_theta(std::atan2(y_dot, x_dot));
    ref_point->set_kappa((x_dot * y_dot_dot - y_dot * x_dot_dot) /
                         (dot_norm * dot_norm * dot_norm));
    ref_point->set_x_derivative(x_dot);
    ref_point->set_y_derivative(y_dot);
    // ref_point.set_s(t * scale + anchor_points_.front().s);
    if (!smoothed_track->empty()) {
      sum_distance +=
          std::hypot(ref_point->x() - smoothed_track->back().path_point().x(),
                     ref_point->y() - smoothed_track->back().path_point().y());
    }
    ref_point->set_s(sum_distance + anchor_points_.front().s);
    smoothed_track->emplace_back(std::move(traj_point));
  }
  // get spline result
  const auto& x_params = spline.smoothing_spline(0).spline_func_x().params();
  const auto& y_params = spline.smoothing_spline(0).spline_func_y().params();
  std::copy(x_params.begin(), x_params.end(), spline_result_.x_coefs.begin());
  std::copy(y_params.begin(), y_params.end(), spline_result_.y_coefs.begin());
  spline_result_.start_s = anchor_points_.front().s;
  spline_result_.length = sum_distance;
  spline_result_.x_coefs[0] += ref_x_;
  spline_result_.y_coefs[0] += ref_y_;

  if (smoothed_track->size() < 2) {
    AERROR << "Fail to generate smoothed reference line.";
    return false;
  }
  last_smoothed_track_ = *smoothed_track;
  return true;
}

bool QpSplineReferenceLineSmoother::GetSplineResult(
    SplineSmoothResult* spline_result) {
  if (spline_result_.x_coefs.empty()) return false;
  *spline_result = spline_result_;
  return true;
}

bool QpSplineReferenceLineSmoother::Sampling() {
  const double length = anchor_points_.back().s - anchor_points_.front().s;
  uint32_t num_spline = std::max(
      1u, static_cast<uint32_t>(length / config_.max_spline_length + 0.5));
  for (std::uint32_t i = 0; i <= num_spline; ++i) {
    t_knots_.push_back(i * 1.0);
  }
  // normalize point xy
  // ref_x_ = anchor_points_.front().x;
  // ref_y_ = anchor_points_.front().y;

  // disable normalization because of warm start
  ref_x_ = 0.0;
  ref_y_ = 0.0;
  return true;
}

bool QpSplineReferenceLineSmoother::AddConstraint() {
  // Add x, y boundary constraint
  std::vector<double> headings;
  std::vector<double> longitudinal_bound;
  std::vector<double> lateral_bound;
  std::vector<common::math::Vec2d> xy_points;
  for (const auto& point : anchor_points_) {
    headings.push_back(point.theta);
    longitudinal_bound.push_back(point.longitudinal_bound);
    lateral_bound.push_back(point.lateral_bound);
    xy_points.emplace_back(point.x - ref_x_, point.y - ref_y_);
  }
  const double scale = (anchor_points_.back().s - anchor_points_.front().s) /
                       (t_knots_.back() - t_knots_.front());
  std::vector<double> evaluated_t;
  for (const auto& point : anchor_points_) {
    evaluated_t.emplace_back((point.s - anchor_points_.front().s) / scale);
  }

  auto* spline_constraint = spline_solver_->mutable_constraint();

  // all points (x, y) should not deviate anchor points by a bounding box
  if (!spline_constraint->Add2dBoundary(evaluated_t, headings, xy_points,
                                        longitudinal_bound, lateral_bound)) {
    AERROR << "Add 2d boundary constraint failed.";
    return false;
  }

  // the heading of the first point should be identical to the anchor point.
  if (!spline_constraint->AddPointAngleConstraint(evaluated_t.back(),
                                                  headings.back())) {
    AERROR << "Add end point and angle constraint failed.";
    return false;
  }

  // all spline should be connected smoothly to the second order derivative.
  if (!spline_constraint->AddSecondDerivativeSmoothConstraint()) {
    AERROR << "Add jointness constraint failed.";
    return false;
  }

  return true;
}

bool QpSplineReferenceLineSmoother::AddKernel() {
  Spline2dKernel* kernel = spline_solver_->mutable_kernel();

  // add spline kernel
  if (config_.first_derivative_weight > 0.0) {
    kernel->AddDerivativeKernelMatrix(config_.first_derivative_weight);
  }
  if (config_.second_derivative_weight > 0.0) {
    kernel->AddSecondOrderDerivativeMatrix(config_.second_derivative_weight);
  }
  if (config_.third_derivative_weight > 0.0) {
    kernel->AddThirdOrderDerivativeMatrix(config_.third_derivative_weight);
  }
  // Add x, y boundary constraint
  std::vector<common::math::Vec2d> xy_points;
  for (const auto& point : anchor_points_) {
    xy_points.emplace_back(point.x - ref_x_, point.y - ref_y_);
  }
  const double scale = (anchor_points_.back().s - anchor_points_.front().s) /
                       (t_knots_.back() - t_knots_.front());
  std::vector<double> evaluated_t;
  for (const auto& point : anchor_points_) {
    evaluated_t.emplace_back((point.s - anchor_points_.front().s) / scale);
  }
  kernel->AddReferenceLineKernelMatrix(evaluated_t, xy_points,
                                       config_.reference_point_weight);
  kernel->AddRegularization(config_.regularization_weight);
  if (spline_solver_->IsLastProblemSolved()) {
    kernel->AddAlignmentKernel(spline_solver_->GetLastSolution(),
                               config_.alignment_weight);
  }
  return true;
}

bool QpSplineReferenceLineSmoother::Solve() { return spline_solver_->Solve(); }

}  // namespace planning
}  // namespace cyberc3
