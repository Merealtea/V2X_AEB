/**
 * @file reference_line_smoother.h
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief virtual base class of reference line smoother, inspired by Apollo
 * @version 0.1
 * @date 2023-11-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <vector>

#include "common/math/vec2d.h"
#include "discretized_trajectory.h"
#include "track_manager_type.h"

namespace cyberc3 {
namespace planning {

class ReferenceLineSmoother {
 public:
  ReferenceLineSmoother() = default;
  virtual bool UpdateRawTrajectory(const DiscretizedTrajectory&, const size_t&,
                                   const common::math::Vec2d&) = 0;
  virtual bool Smooth(DiscretizedTrajectory* const) = 0;
  virtual bool GetSplineResult(SplineSmoothResult*) = 0;
  virtual void Reset() = 0;

  virtual ~ReferenceLineSmoother() = default;
};

}  // namespace planning
}  // namespace cyberc3
