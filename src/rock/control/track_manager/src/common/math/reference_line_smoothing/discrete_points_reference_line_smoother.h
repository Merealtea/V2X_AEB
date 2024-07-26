#pragma once

#include <utility>
#include <vector>

#include "discretized_points_smoothing/fem_pos_deviation_smoother_config.h"

#include "reference_line_smoother.h"

namespace cyberc3 {
namespace planning {

class DiscretePointsReferenceLineSmoother : public ReferenceLineSmoother {
 public:
  explicit DiscretePointsReferenceLineSmoother(
      const FemPosDeviationSmootherConfig& config, double keepbehind_distance);

  ~DiscretePointsReferenceLineSmoother() override = default;

  bool UpdateRawTrajectory(const DiscretizedTrajectory& raw_track,
                           const size_t& nearest_index,
                           const common::math::Vec2d& ego_vec) override;
  bool Smooth(DiscretizedTrajectory* const smoothed_track) override;
  bool GetSplineResult(SplineSmoothResult*) override { return false; };
  void Reset() override;

 private:
  bool FemPosSmooth(
      const std::vector<std::pair<double, double>>& raw_point2d,
      const std::vector<double>& bounds,
      std::vector<std::pair<double, double>>* ptr_smoothed_point2d);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  FemPosDeviationSmootherConfig config_;

  DiscretizedTrajectory last_smoothed_track_;
  std::vector<std::pair<double, double>> anchor_points_;
  std::vector<double> anchor_bounds_;
  double anchor_points_start_s_ = 0.0;

  double keepbehind_distance_ = 2.5;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};

}  // namespace planning
}  // namespace cyberc3
