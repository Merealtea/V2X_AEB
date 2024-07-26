/**
 * @file track_smoother.cc
 * @author Hongxin Chen (angleochen@sjtu.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-08-01
 *
 * @copyright Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 */

#include "track_smoother.h"
#include "common/struct/pnc_point.h"
#include "reference_line/discrete_points_reference_line_smoother.h"
#include "common/math/vec2d.h"
#include "math/discrete_points_math.h"
#include <chrono>
#include <common/log.h>

namespace cyberc3 {
namespace planning {
using common::PathPoint;

bool SmoothPlatoonTrack(const std::list<TrackPoint> &track_in,
                        DiscretizedPath &track_out) {

  FemPosDeviationSmootherConfig fem_pos_config_;
  fem_pos_config_.curvature_constraint = 1. / 5.;

  fem_pos_config_.weight_fem_pos_deviation = 1e8;
  fem_pos_config_.weight_path_length = 1.;
  fem_pos_config_.weight_ref_deviation = .0;
  fem_pos_config_.weight_curvature_constraint_slack_var = 1e2;

  fem_pos_config_.max_iter = 1e3;
  fem_pos_config_.time_limit = 0.0;
  fem_pos_config_.verbose = false;
  fem_pos_config_.scaled_termination = true;
  fem_pos_config_.warm_start = true;

  fem_pos_config_.sqp_pen_max_iter = 100;
  fem_pos_config_.sqp_ftol = 1e-1;
  fem_pos_config_.sqp_sub_max_iter = 100;
  fem_pos_config_.sqp_ctol = 1e-1;

  // fem_pos_config_.print_level = 0;
  // fem_pos_config_.max_num_of_iterations = 10000;
  // fem_pos_config_.acceptable_tol = 10;
  // fem_pos_config_.tol = 1e-1;
  // fem_pos_config_.acceptable_tol = 1;

  track_out.clear();

  std::vector<std::pair<double, double>> warm_start_point2ds;
  // Interpolate the traj
  double kSmoothDeltaS = 0.2;

  DiscretizedPath warm_start_path;

  for (const auto &point : track_in) {
      PathPoint path_point;
      path_point.set_x(point.x_);
      path_point.set_y(point.y_);
      path_point.set_theta(point.theta_);
      path_point.set_s(point.curve_distance_);
      warm_start_path.emplace_back(std::move(path_point));
  }

  std::vector<std::pair<double, double>> interpolated_warm_start_point2ds;
  double path_length = warm_start_path.Length();
  AINFO << "Current path_length is: " << path_length;
  double delta_s = path_length / std::ceil(path_length / kSmoothDeltaS);
  path_length += delta_s * 1.0e-6;
  for (double s = 0; s < path_length; s += delta_s) {
      const auto point2d = warm_start_path.Evaluate(warm_start_path.begin()->s()+s);
      interpolated_warm_start_point2ds.emplace_back(point2d.x(), point2d.y());
  }

  // std::vector<double> headings;
  // std::vector<double> kappas;
  // std::vector<double> dkappas;
  // std::vector<double> s;
  // DiscretePointsMath::ComputePathProfile(interpolated_warm_start_point2ds,
  //                                        &headings, &s, &kappas, &dkappas);

  // AINFO << "original size is:" << kappas.size();
  // for (int i = 0; i < kappas.size(); ++i)
  //   AINFO << "\t" << s[i] << "\t" << kappas[i];

  std::vector<AnchorPoint> anchor_points;
  for (const auto &lane_point : interpolated_warm_start_point2ds) {
    AnchorPoint anchor_point;

    anchor_point.x = lane_point.first;
    anchor_point.y = lane_point.second;
    anchor_point.lateral_bound = 0.25;

    anchor_points.emplace_back(anchor_point);
  }

  // std::vector<AnchorPoint> anchor_points;
  // for (const auto &lane_point : track_in ) {
  //   AnchorPoint anchor_point;

  //   anchor_point.x = lane_point.x_;
  //   anchor_point.y = lane_point.y_;
  //   anchor_point.lateral_bound = 0.2;

  //   anchor_points.emplace_back(anchor_point);
  // }
  anchor_points[0].lateral_bound = 1e-3;
  anchor_points[1].lateral_bound = 1e-3;
  anchor_points[anchor_points.size() - 1].lateral_bound = 1e-3;
  anchor_points[anchor_points.size() - 2].lateral_bound = 1e-3;
  auto time_begin = std::chrono::system_clock::now();
  std::vector<std::pair<double, double>> smoothed_point2ds;
  DiscretePointsReferenceLineSmoother smoother(fem_pos_config_);
  smoother.SetAnchorPoints(anchor_points);
  if (!smoother.Smooth(&smoothed_point2ds)) {
    AERROR << "Smooth reference line failed!";
    return false;
  }
  auto time_end = std::chrono::system_clock::now();

  AINFO << "qp time is : "
        << std::chrono::duration_cast<std::chrono::nanoseconds>(time_end -
                                                                time_begin)
                   .count() *
               1.0 / 1e6
        << "ms.";

  std::vector<double> new_headings;
  std::vector<double> new_kappas;
  std::vector<double> new_dkappas;
  std::vector<double> new_s;
  DiscretePointsMath::ComputePathProfile(smoothed_point2ds, &new_headings,
                                         &new_s, &new_kappas, &new_dkappas);
  //注意：对于最终结果，逆行时kappa和heading需取反
  AINFO << "smooth size is:" << new_kappas.size();
  for (int i = 0; i < new_kappas.size(); ++i)
    AINFO << "\t" << new_s[i] << "\t" << new_kappas[i];

  double origin_s = track_in.front().curve_distance_;
  DiscretizedPath path;
  for (int i = 0; i < smoothed_point2ds.size(); ++i) {
    PathPoint path_point;
    path_point.set_x(smoothed_point2ds[i].first);
    path_point.set_y(smoothed_point2ds[i].second);
    path_point.set_theta(new_headings[i]);
    path_point.set_s(new_s[i] + origin_s);
    path_point.set_kappa(new_kappas[i]);
    path_point.set_dkappa(new_dkappas[i]);
    track_out.push_back(std::move(path_point));
  }
  // for (int i = 0; i < smoothed_point2ds.size(); ++i) {
  //   TrackPoint path_point;
  //   path_point.x_ = smoothed_point2ds[i].first;
  //   path_point.y_ = smoothed_point2ds[i].second;
  //   path_point.theta_ = new_headings[i];
  //   path_point.curve_distance_ = new_s[i];
  //   path_point.kappa_ = new_kappas[i];
  //   track_out.push_back(std::move(path_point));
  // }
  return true;
}
}
} // namespace cyberc3
