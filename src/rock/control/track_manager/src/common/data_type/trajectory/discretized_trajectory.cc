#include "discretized_trajectory.h"

#include <algorithm>
#include <limits>

#include "common/log.h"
#include "common/math/linear_interpolation.h"

namespace cyberc3 {
namespace planning {

using cyberc3::common::TrajectoryPoint;

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::deque<TrajectoryPoint>& trajectory_points)
    : std::deque<TrajectoryPoint>(trajectory_points) {
  ACHECK(!trajectory_points.empty())
      << "trajectory_points should NOT be empty()";
}

TrajectoryPoint DiscretizedTrajectory::EvaluateByTime(
    const double relative_time) const {
  ACHECK(!empty());
  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    AWARN << "When evaluate trajectory, relative_time(" << relative_time
          << ") is too large";
    return back();
  }
  return common::math::InterpolateUsingLinearApproximationByTime(
      *(it_lower - 1), *it_lower, relative_time);
}

TrajectoryPoint DiscretizedTrajectory::EvaluateByS(const double path_s) const {
  ACHECK(!empty());
  auto comp = [](const TrajectoryPoint& p, const double path_s) {
    return p.path_point().s() < path_s;
  };

  auto it_lower = std::lower_bound(begin(), end(), path_s, comp);

  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    AWARN << "When evaluate trajectory, path_s(" << path_s << ") is too large";
    return back();
  }
  return common::math::InterpolateUsingLinearApproximationByS(
      *(it_lower - 1), *it_lower, path_s);
}

size_t DiscretizedTrajectory::QueryLowerBoundPoint(const double relative_time,
                                                   const double epsilon) const {
  ACHECK(!empty());

  if (relative_time >= back().relative_time()) {
    return size() - 1;
  }
  auto func = [&epsilon](const TrajectoryPoint& tp,
                         const double relative_time) {
    return tp.relative_time() + epsilon < relative_time;
  };
  auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
  return std::distance(begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryNearestPoint(
    const common::math::Vec2d& position) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const common::math::Vec2d curr_point(this->at(i).path_point().x(),
                                         this->at(i).path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

size_t DiscretizedTrajectory::QueryNearestPointWithStartIndex(
    const common::math::Vec2d& position, const size_t start_index) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = start_index;
  for (size_t i = start_index; i < size(); ++i) {
    const common::math::Vec2d curr_point(this->at(i).path_point().x(),
                                         this->at(i).path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
    const common::math::Vec2d& position, const double buffer) const {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const common::math::Vec2d curr_point(this->at(i).path_point().x(),
                                         this->at(i).path_point().y());

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  if (!empty()) {
    CHECK_GT(trajectory_point.relative_time(), back().relative_time());
  }
  push_back(trajectory_point);
}

const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const size_t index) const {
  CHECK_LT(index, NumOfPoints());
  return this->at(index);
}

TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  ACHECK(!empty());
  return front();
}

double DiscretizedTrajectory::GetTemporalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().relative_time() - front().relative_time();
}

double DiscretizedTrajectory::GetSpatialLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().path_point().s() - front().path_point().s();
}

}  // namespace planning
}  // namespace cyberc3
