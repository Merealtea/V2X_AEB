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
 * @file qp_spline_reference_line_smoother.h
 **/

#pragma once

#include <memory>
#include <vector>

#include "common/math/vec2d.h"
#include "common/struct/pnc_point.h"
#include "reference_line_smoother.h"
#include "spline_smoothing/spline_2d_solver.h"
#include "track_manager_type.h"

namespace cyberc3 {
namespace planning {

struct ReferenceLineSmootherConfig {
  int spline_order = 5;
  int max_spline_length = 20;
  // int num_of_total_points = 500;
  double point_invterval = 0.02;
  double first_derivative_weight = 0.0;
  double second_derivative_weight = 200.0;
  double third_derivative_weight = 1000.0;
  double reference_point_weight = 500.0;
  double regularization_weight = 1e-4;
  double alignment_weight = 10;
};

struct AnchorPoint {
  double x = 0.;
  double y = 0.;
  double s = 0.;
  double theta = 0.;
  double lateral_bound = 0.0;
  double longitudinal_bound = 0.0;
  // enforce smoother to strictly follow this reference point
  bool enforced = false;
};

class QpSplineReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  explicit QpSplineReferenceLineSmoother(
      const ReferenceLineSmootherConfig& config, double keepbehind_distance);

  ~QpSplineReferenceLineSmoother() override = default;

  bool UpdateRawTrajectory(const DiscretizedTrajectory& raw_track,
                           const size_t& nearest_index,
                           const common::math::Vec2d& ego_vec) override;
  bool Smooth(DiscretizedTrajectory* const smoothed_track) override;
  bool GetSplineResult(SplineSmoothResult* spline_result) override;
  void Reset() override;

 private:
  void Clear();
  bool Sampling();

  bool AddConstraint();

  bool AddKernel();

  bool Solve();

  std::uint32_t FindIndex(const double t) const;

 private:
  std::vector<double> t_knots_;
  std::vector<AnchorPoint> anchor_points_;
  std::unique_ptr<Spline2dSolver> spline_solver_;
  ReferenceLineSmootherConfig config_;
  DiscretizedTrajectory last_smoothed_track_;
  SplineSmoothResult spline_result_;
  double keepbehind_distance_ = 5.0;

  double ref_x_ = 0.0;
  double ref_y_ = 0.0;
};

}  // namespace planning
}  // namespace cyberc3
