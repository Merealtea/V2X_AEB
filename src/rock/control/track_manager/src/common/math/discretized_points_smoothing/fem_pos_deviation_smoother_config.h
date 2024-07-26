#pragma once

namespace cyberc3 {
namespace planning {
struct FemPosDeviationSmootherConfig {
  // constraints
  bool apply_curvature_constraint = true;
  double curvature_constraint = 1. / 5.;

  // sqp_osqp
  // Weights in optimization cost function
  double weight_fem_pos_deviation = 1e8;
  double weight_path_length = 1.;
  double weight_ref_deviation = 1.;
  double weight_curvature_constraint_slack_var = 1e2;

  // Settings of sqp
  int sqp_sub_max_iter = 100;
  double sqp_ftol = 1e-1;
  int sqp_pen_max_iter = 100;
  double sqp_ctol = 1e-1;

  // Settings of osqp
  int max_iter = 1e3;
  double time_limit = 0.0;
  bool verbose = false;
  bool scaled_termination = true;
  bool warm_start = true;
};
}  // namespace planning
}  // namespace cyberc3
