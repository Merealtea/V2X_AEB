/**
 * @file track_smoother.h
 * @author Hongxin Chen (angleochen@sjtu.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-08-01
 *
 * @copyright Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 */

#pragma once

#include <memory>

#include "common/math/vec2d.h"
#include "discretized_path.h"
#include "discretized_trajectory.h"
#include "reference_line_smoothing/discrete_points_reference_line_smoother.h"
#include "reference_line_smoothing/qp_spline_reference_line_smoother.h"
#include "track_manager_type.h"

namespace cyberc3 {
namespace planning {
class PlatoonTrackSmoother {
 public:
  void Init(SmoothConfig config);
  void Reset() { smoother_->Reset(); }
  bool SmoothPlatoonTrack(const DiscretizedTrajectory &track_in,
                          const size_t &neareast_index,
                          const common::math::Vec2d &ego_vec,
                          DiscretizedTrajectory *track_out,
                          SplineSmoothResult *spline_result);

 private:
  SmoothConfig config_;
  std::unique_ptr<ReferenceLineSmoother> smoother_;
};
}  // namespace planning
}  // namespace cyberc3
