#pragma once

#include <deque>
#include <vector>

#include "common/log.h"
#include "common/math/vec2d.h"
#include "common/struct/pnc_point.h"

namespace cyberc3 {
namespace planning {

class DiscretizedTrajectory : public std::deque<common::TrajectoryPoint> {
 public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */
  explicit DiscretizedTrajectory(
      const std::deque<common::TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const std::deque<common::TrajectoryPoint>& trajectory_points);

  ~DiscretizedTrajectory() = default;

  common::TrajectoryPoint StartPoint() const;

  double GetTemporalLength() const;

  double GetSpatialLength() const;

  common::TrajectoryPoint EvaluateByTime(const double relative_time) const;

  common::TrajectoryPoint EvaluateByS(const double path_s) const;

  size_t QueryLowerBoundPoint(const double relative_time,
                              const double epsilon = 1.0e-5) const;

  size_t QueryNearestPoint(const common::math::Vec2d& position) const;

  size_t QueryNearestPointWithStartIndex(
      const common::math::Vec2d& position,const size_t start_index) const;

  size_t QueryNearestPointWithBuffer(const common::math::Vec2d& position,
                                     const double buffer) const;

  void AppendTrajectoryPoint(const common::TrajectoryPoint& trajectory_point);

  void PrependTrajectoryPoints(
      const std::vector<common::TrajectoryPoint>& trajectory_points) {
    if (!empty() && trajectory_points.size() > 1) {
      ACHECK(trajectory_points.back().relative_time() <
             front().relative_time());
    }
    insert(begin(), trajectory_points.begin(), trajectory_points.end());
  }

  const common::TrajectoryPoint& TrajectoryPointAt(const size_t index) const;

  size_t NumOfPoints() const { return size(); };

  void Clear() { clear(); }
};


}  // namespace planning
}  // namespace cyberc3
