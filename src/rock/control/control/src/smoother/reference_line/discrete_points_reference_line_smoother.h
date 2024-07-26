#pragma once

#include <utility>
#include <vector>

#include "math/discretized_points_smoothing/fem_pos_deviation_smoother_config.h"

namespace cyberc3 {
namespace planning {

struct AnchorPoint {
  double x = 0.;
  double y = 0.;
  double lateral_bound = 0.0;
  double longitudinal_bound = 0.0;
  // enforce smoother to strictly follow this reference point
  bool enforced = false;
};

class DiscretePointsReferenceLineSmoother {
 public:
  explicit DiscretePointsReferenceLineSmoother(
      const FemPosDeviationSmootherConfig& config);

  virtual ~DiscretePointsReferenceLineSmoother() = default;

  bool Smooth(std::vector<std::pair<double, double>>* smoothed_point2d);

  void SetAnchorPoints(const std::vector<AnchorPoint>&);

 private:
  bool FemPosSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  FemPosDeviationSmootherConfig config_;

  std::vector<AnchorPoint> anchor_points_;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};

}  // namespace planning
}  // namespace cyberc3
