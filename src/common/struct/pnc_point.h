#pragma once

#include <memory>
#include <string>
#include <utility>

#pragma once

namespace cyberc3 {
namespace common {
class SpeedPoint {
 public:
  double s() const { return s_; }
  double t() const { return t_; }
  double v() const { return v_; }
  double a() const { return a_; }
  double da() const { return da_; }

  void set_s(const double s) { s_ = s; }
  void set_t(const double t) { t_ = t; }
  void set_v(const double v) { v_ = v; }
  void set_a(const double a) { a_ = a; }
  void set_da(const double da) { da_ = da; }

 private:
  double s_ = .0;
  double t_ = .0;
  // speed (m/s)
  double v_ = .0;
  // acceleration (m/s^2)
  double a_ = .0;
  // jerk (m/s^3)
  double da_ = .0;
};

class PathPoint {
 public:
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double theta() const { return theta_; }
  double kappa() const { return kappa_; }
  double s() const { return s_; }
  double dkappa() const { return dkappa_; }
  double ddkappa() const { return ddkappa_; }
  double x_derivative() const { return x_derivative_; }
  double y_derivative() const { return y_derivative_; }

  void set_x(const double x) { x_ = x; }
  void set_y(const double y) { y_ = y; }
  void set_z(const double z) { z_ = z; }
  void set_theta(const double theta) { theta_ = theta; }
  void set_kappa(const double kappa) { kappa_ = kappa; }
  void set_s(const double s) { s_ = s; }
  void set_dkappa(const double dkappa) { dkappa_ = dkappa; }
  void set_ddkappa(const double ddkappa) { ddkappa_ = ddkappa; }
  void set_x_derivative(const double x_derivative) {
    x_derivative_ = x_derivative;
  }
  void set_y_derivative(const double y_derivative) {
    y_derivative_ = y_derivative;
  }

 private:
  // coordinates
  double x_ = 0.;
  double y_ = 0.;
  double z_ = 0.;

  // direction on the x-y plane
  double theta_ = 0.;
  // curvature on the x-y planning
  double kappa_ = 0.;
  // accumulated distance from beginning of the path
  double s_ = 0.;

  // derivative of kappa w.r.t s.
  double dkappa_ = 0.;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa_ = 0.;

  // derivative of x and y w.r.t parametric parameter t in
  // CosThetareferenceline
  double x_derivative_ = 0.;
  double y_derivative_ = 0.;
};

class TrajectoryPoint {
 public:
  PathPoint path_point() const { return path_point_; }
  double v() const { return v_; }
  double a() const { return a_; }
  double relative_time() const { return relative_time_; }
  double da() const { return da_; }
  double steer() const { return steer_; }

  PathPoint* mutable_path_point() { return &path_point_; }
  void set_v(const double v) { v_ = v; }
  void set_a(const double a) { a_ = a; }
  void set_relative_time(const double relative_time) {
    relative_time_ = relative_time;
  }
  void set_da(const double da) { da_ = da; }
  void set_steer(const double steer) { steer_ = steer; }

 private:
  // path point
  PathPoint path_point_;
  // linear velocity
  double v_ = .0;  // in [m/s]
  // linear acceleration
  double a_ = .0;
  // relative time from beginning of the trajectory
  double relative_time_ = .0;
  // longitudinal jerk
  double da_ = .0;
  // The angle between vehicle front wheel and vehicle longitudinal axis
  double steer_ = .0;
};
}  // namespace common
}  // namespace cyberc3
