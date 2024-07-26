#pragma once

namespace cyberc3 {
namespace planning {
struct FemPosDeviationSmootherConfig {
  // constraints
  bool apply_curvature_constraint;
  double curvature_constraint;

  // sqp_osqp
  // Weights in optimization cost function
  double weight_fem_pos_deviation;
  double weight_path_length;
  double weight_ref_deviation;
  double weight_curvature_constraint_slack_var;

  // Settings of sqp
  int sqp_sub_max_iter;
  double sqp_ftol;
  int sqp_pen_max_iter;
  double sqp_ctol;

  // Settings of osqp
  int max_iter;
  double time_limit;
  bool verbose;
  bool scaled_termination;
  bool warm_start;
};
}  // namespace planning
}  // namespace cyberc3
