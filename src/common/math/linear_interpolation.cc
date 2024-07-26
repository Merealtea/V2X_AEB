/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file linear_interpolation.cc
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#include "common/math/linear_interpolation.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "common/log.h"
#include "common/math/math_utils.h"

namespace cyberc3 {
namespace common {
namespace math {

double interpolate1d(const double x, const std::vector<double> &xs,
                     const std::vector<double> &ys) {
  const size_t N = xs.size();
  const size_t hi = std::lower_bound(xs.begin(), xs.end(), x) - xs.begin();
  if (hi == 0) return ys[0];
  if (hi == N) return ys[N - 1];
  const size_t low = hi - 1;
  const float x_diff = xs[hi] - xs[low];
  if (std::abs(x_diff) < 1e-5) return ys[low];
  return ys[low] + (ys[hi] - ys[low]) * (x - xs[low]) / x_diff;
}

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    ADEBUG << "input time difference is too small";
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s) {
  double s0 = p0.s();
  double s1 = p1.s();

  PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x() + weight * p1.x();
  double y = (1 - weight) * p0.y() + weight * p1.y();
  double theta = slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s);
  double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
  double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
  double ddkappa = (1 - weight) * p0.ddkappa() + weight * p1.ddkappa();
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  path_point.set_s(s);
  return path_point;
}

TrajectoryPoint InterpolateUsingLinearApproximationByTime(
    const TrajectoryPoint &tp0, const TrajectoryPoint &tp1, const double t) {
  const PathPoint pp0 = tp0.path_point();
  const PathPoint pp1 = tp1.path_point();
  double t0 = tp0.relative_time();
  double t1 = tp1.relative_time();

  TrajectoryPoint tp;
  tp.set_v(lerp(tp0.v(), t0, tp1.v(), t1, t));
  tp.set_a(lerp(tp0.a(), t0, tp1.a(), t1, t));
  tp.set_relative_time(t);
  tp.set_steer(slerp(tp0.steer(), t0, tp1.steer(), t1, t));

  PathPoint *path_point = tp.mutable_path_point();
  path_point->set_x(lerp(pp0.x(), t0, pp1.x(), t1, t));
  path_point->set_y(lerp(pp0.y(), t0, pp1.y(), t1, t));
  path_point->set_theta(slerp(pp0.theta(), t0, pp1.theta(), t1, t));
  path_point->set_kappa(lerp(pp0.kappa(), t0, pp1.kappa(), t1, t));
  path_point->set_dkappa(lerp(pp0.dkappa(), t0, pp1.dkappa(), t1, t));
  path_point->set_ddkappa(lerp(pp0.ddkappa(), t0, pp1.ddkappa(), t1, t));
  path_point->set_s(lerp(pp0.s(), t0, pp1.s(), t1, t));

  return tp;
}

TrajectoryPoint InterpolateUsingLinearApproximationByS(
    const TrajectoryPoint &tp0, const TrajectoryPoint &tp1, const double s) {
  const PathPoint pp0 = tp0.path_point();
  const PathPoint pp1 = tp1.path_point();
  double s0 = pp0.s();
  double s1 = pp1.s();

  TrajectoryPoint tp;
  tp.set_v(lerp(tp0.v(), s0, tp1.v(), s1, s));
  tp.set_a(lerp(tp0.a(), s0, tp1.a(), s1, s));
  tp.set_relative_time(
      lerp(tp0.relative_time(), s0, tp1.relative_time(), s1, s));
  tp.set_steer(slerp(tp0.steer(), s0, tp1.steer(), s1, s));

  PathPoint *path_point = tp.mutable_path_point();
  path_point->set_x(lerp(pp0.x(), s0, pp1.x(), s1, s));
  path_point->set_y(lerp(pp0.y(), s0, pp1.y(), s1, s));
  path_point->set_theta(slerp(pp0.theta(), s0, pp1.theta(), s1, s));
  path_point->set_kappa(lerp(pp0.kappa(), s0, pp1.kappa(), s1, s));
  path_point->set_dkappa(lerp(pp0.dkappa(), s0, pp1.dkappa(), s1, s));
  path_point->set_ddkappa(lerp(pp0.ddkappa(), s0, pp1.ddkappa(), s1, s));
  path_point->set_s(s);

  return tp;
}

}  // namespace math
}  // namespace common
}  // namespace cyberc3
