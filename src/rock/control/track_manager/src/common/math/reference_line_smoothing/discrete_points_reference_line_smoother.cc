#include "discrete_points_reference_line_smoother.h"

#include <algorithm>
#include <cmath>

#include "common/log.h"
#include "discrete_points_math.h"
#include "discretized_points_smoothing/fem_pos_deviation_smoother.h"
#include "discretized_trajectory.h"

namespace cyberc3 {
namespace planning {

DiscretePointsReferenceLineSmoother::DiscretePointsReferenceLineSmoother(
    const FemPosDeviationSmootherConfig& config, double keepbehind_distance)
    : config_(config), keepbehind_distance_(keepbehind_distance) {}

void DiscretePointsReferenceLineSmoother::Reset() {
  last_smoothed_track_.clear();
  anchor_points_.clear();
  anchor_bounds_.clear();
  anchor_points_start_s_ = 0.0;
}

bool DiscretePointsReferenceLineSmoother::UpdateRawTrajectory(
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

  // Interpolate the traj
  constexpr double kSmoothDeltaS = 0.2;
  const double start_s = raw_path.front().path_point().s();
  const double end_s = raw_path.back().path_point().s();
  double path_length = end_s - start_s;
  ADEBUG << "Current path_length is: " << path_length;
  const double delta_s = path_length / std::ceil(path_length / kSmoothDeltaS);
  anchor_points_start_s_ = start_s;

  anchor_points_.clear();
  anchor_bounds_.clear();

  for (double s = start_s; s <= end_s; s += delta_s) {
    const auto point2d = raw_path.EvaluateByS(s);
    anchor_points_.emplace_back(point2d.path_point().x(),
                                point2d.path_point().y());
    if (s <= last_smooth_end_s) {
      anchor_bounds_.emplace_back(0.01 * (s - start_s) /
                                  (last_smooth_end_s - start_s));
    } else {
      anchor_bounds_.emplace_back(0.2);
    }
  }
  anchor_bounds_[0] = 1e-3;
  anchor_bounds_[1] = 1e-3;
  anchor_bounds_[anchor_points_.size() - 1] = 0.01;
  anchor_bounds_[anchor_points_.size() - 2] = 0.01;
  return true;
}

bool DiscretePointsReferenceLineSmoother::Smooth(
    DiscretizedTrajectory* const smoothed_track) {

  NormalizePoints(&anchor_points_);
  std::vector<std::pair<double, double>> smoothed_points;

  bool status = FemPosSmooth(anchor_points_, anchor_bounds_, &smoothed_points);

  if (!status) {
    AERROR << "discrete_points reference line smoother fails";
    return false;
  }

  std::vector<double> new_headings;
  std::vector<double> new_kappas;
  std::vector<double> new_dkappas;
  std::vector<double> new_s;
  DiscretePointsMath::ComputePathProfile(smoothed_points, &new_headings, &new_s,
                                         &new_kappas, &new_dkappas);
  AINFO << "smooth size is:" << new_kappas.size();
  for (int i = 0; i < new_kappas.size(); ++i)
    ADEBUG << "\t" << new_s[i] << "\t" << new_kappas[i];

  smoothed_track->clear();
  for (int i = 0; i < smoothed_points.size(); ++i) {
    common::TrajectoryPoint traj_point;
    traj_point.mutable_path_point()->set_x(smoothed_points[i].first + zero_x_);
    traj_point.mutable_path_point()->set_y(smoothed_points[i].second + zero_y_);
    traj_point.mutable_path_point()->set_s(new_s[i] + anchor_points_start_s_);
    traj_point.mutable_path_point()->set_theta(new_headings[i]);
    traj_point.mutable_path_point()->set_kappa(new_kappas[i]);
    traj_point.mutable_path_point()->set_dkappa(new_dkappas[i]);
    smoothed_track->emplace_back(std::move(traj_point));
  }

  last_smoothed_track_ = *smoothed_track;
  return true;
}

bool DiscretePointsReferenceLineSmoother::FemPosSmooth(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  FemPosDeviationSmoother smoother(config_);

  // box contraints on pos are used in fem pos smoother, thus shrink the
  // bounds by 1.0 / sqrt(2.0)
  std::vector<double> box_bounds = bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_point2d, box_bounds, &opt_x, &opt_y);

  if (!status) {
    AERROR << "Fem Pos reference line smoothing failed";
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    AERROR << "Return by fem pos smoother is wrong. Size smaller than 2 ";
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size()) << "x and y result size not equal";

  size_t point_size = opt_x.size();
  for (size_t i = 0; i < point_size; ++i) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}


void DiscretePointsReferenceLineSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

}  // namespace planning
}  // namespace cyberc3
