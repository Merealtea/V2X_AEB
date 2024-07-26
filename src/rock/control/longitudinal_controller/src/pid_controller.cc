/**
 * @file pid_controller.cc
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief basic PID controller with output saturation and integral separation
 * @version 0.1
 * @date 2023-03-02
 *
 * @copyright CyberC3 Copyright (c) 2023
 *
 */
#include "pid_controller.h"

#include <algorithm>

namespace cyberc3 {
namespace control {

void PIDController::Init(const double kp, const double ki, const double kd,
                         const double out_max, const double out_min,
                         const double integral_max, const double integral_min,
                         const double i_err_range) {
  SetPID(kp, ki, kd);
  SetSaturation(out_max, out_min, integral_max, integral_min, i_err_range);
  Reset();
}

void PIDController::Init(const PIDConfig &config) {
  SetPID(config.kp, config.ki, config.kd);
  SetSaturation(config.out_max, -config.out_max, config.i_max, -config.i_max,
                config.i_err_range);
  Reset();
}

void PIDController::Reset() {
  integral_ = 0;
  err_last_ = 0;
  first_run_ = true;
}

void PIDController::SetPID(const double kp, const double ki, const double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::SetKp(const double kp) { kp_ = kp; }

void PIDController::SetSaturation(const double out_max, const double out_min,
                                  const double integral_max,
                                  const double integral_min,
                                  const double i_err_range) {
  out_max_ = out_max;
  out_min_ = out_min;
  integral_max_ = integral_max;
  integral_min_ = integral_min;
  i_err_range_ = i_err_range;
}

void PIDController::SetConfig(const PIDConfig &config) {
  SetPID(config.kp, config.ki, config.kd);
  SetSaturation(config.out_max, -config.out_max, config.i_max, -config.i_max,
                config.i_err_range);
}

double PIDController::Update(const double error, double dt) {
  if (first_run_) {
    err_last_ = error;
    first_run_ = false;
  }
  double err_diff = (error - err_last_) / dt;
  // consider integration only in error range to avoid large output during start
  if (std::abs(error) < i_err_range_) {
    integral_ += ki_ * error * dt;
    integral_ = std::max(integral_min_, std::min(integral_, integral_max_));
  } else {
    integral_ = 0;
  }
  double output = kp_ * error + integral_ + kd_ * err_diff;
  output = std::max(out_min_, std::min(output, out_max_));
  err_last_ = error;
  return output;
}
}  // namespace control
}  // namespace cyberc3
