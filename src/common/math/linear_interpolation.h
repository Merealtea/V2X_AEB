/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file linear_interpolation.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief Linear interpolation functions.
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#pragma once

#include <cmath>

#include "common/log.h"
#include "common/struct/pnc_point.h"

/**
 * @namespace cyberc3::common::math
 * @brief cyberc3::common::math
 */
namespace cyberc3 {
namespace common {
namespace math {

/**
 * @brief Linear interpolation between two points of type T.
 * @param x0 The coordinate of the first point.
 * @param t0 The interpolation parameter of the first point.
 * @param x1 The coordinate of the second point.
 * @param t1 The interpolation parameter of the second point.
 * @param t The interpolation parameter for interpolation.
 * @param x The coordinate of the interpolated point.
 * @return Interpolated point.
 */
template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    AERROR << "input time difference is too small";
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

/**
 * @brief Spherical linear interpolation between two angles.
 *        The two angles are within range [-M_PI, M_PI).
 * @param a0 The value of the first angle.
 * @param t0 The interpolation parameter of the first angle.
 * @param a1 The value of the second angle.
 * @param t1 The interpolation parameter of the second angle.
 * @param t The interpolation parameter for interpolation.
 * @param a The value of the spherically interpolated angle.
 * @return Interpolated angle.
 */
double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t);

// get interpolated y value from x and two tables
double interpolate1d(const double x, const std::vector<double> &xs,
                     const std::vector<double> &ys);

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s);

TrajectoryPoint InterpolateUsingLinearApproximationByTime(
    const TrajectoryPoint &tp0, const TrajectoryPoint &tp1, const double t);
TrajectoryPoint InterpolateUsingLinearApproximationByS(
    const TrajectoryPoint &tp0, const TrajectoryPoint &tp1, const double s);

}  // namespace math
}  // namespace common
}  // namespace cyberc3
