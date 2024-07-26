#pragma once

#include <utility>
#include <vector>

#include "common/struct/pnc_point.h"

namespace cyberc3 {
namespace planning {

class DiscretizedPath : public std::vector<common::PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<common::PathPoint> path_points);

  double Length() const;

  common::PathPoint Evaluate(const double path_s) const;

  common::PathPoint EvaluateReverse(const double path_s) const;

 protected:
  std::vector<common::PathPoint>::const_iterator QueryLowerBound(
      const double path_s) const;
  std::vector<common::PathPoint>::const_iterator QueryUpperBound(
      const double path_s) const;
};

}  // namespace planning
}  // namespace cyberc3
