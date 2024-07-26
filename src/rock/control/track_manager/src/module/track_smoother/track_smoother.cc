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

#include <chrono>
#include <cmath>

#include <common/log.h>

#include "common/math/vec2d.h"
#include "common/struct/pnc_point.h"
#include "discrete_points_math.h"
#include "discretized_trajectory.h"
#include "reference_line_smoothing/discrete_points_reference_line_smoother.h"
#include "reference_line_smoothing/qp_spline_reference_line_smoother.h"
#include "track_manager_type.h"

namespace cyberc3 {
namespace planning {

void PlatoonTrackSmoother::Init(SmoothConfig config) {
  config_ = config;
  if (config.smoother_type == SmootherType::Spline) {
    smoother_ = std::make_unique<QpSplineReferenceLineSmoother>(
        ReferenceLineSmootherConfig(), config.behind_distance);
  } else if (config.smoother_type == SmootherType::Discrete) {
    smoother_ = std::make_unique<DiscretePointsReferenceLineSmoother>(
        FemPosDeviationSmootherConfig(), config.behind_distance);
  } else {
    AERROR << "Unknown smoother type!";
  }
}

bool PlatoonTrackSmoother::SmoothPlatoonTrack(
    const DiscretizedTrajectory &track_in, const size_t &neareast_index,
    const common::math::Vec2d &ego_vec, DiscretizedTrajectory *track_out,
    SplineSmoothResult *spline_result) {
  auto time_begin = std::chrono::system_clock::now();
  if (!smoother_->UpdateRawTrajectory(track_in, neareast_index, ego_vec)) {
    AWARN << "Update raw trajectory failed!";
    return false;
  }
  if (!smoother_->Smooth(track_out)) {
    AWARN << "Smooth failed!";
    return false;
  }
  if (config_.smoother_type == SmootherType::Spline) {
    // only spline smoother has spline result
    if (!smoother_->GetSplineResult(spline_result)) {
      AWARN << "Get spline result failed!";
      return false;
    }
  }
  auto time_end = std::chrono::system_clock::now();
  AINFO << "smooth time is : "
        << std::chrono::duration_cast<std::chrono::nanoseconds>(time_end -
                                                                time_begin)
                   .count() *
               1.0 / 1e6
        << "ms.";
  return true;
}

}  // namespace planning
}  // namespace cyberc3
