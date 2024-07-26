/**
 * @file pid_controller.hpp
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief basic PID controller with output saturation and integral separation
 * @version 0.1
 * @date 2023-03-02
 *
 * @copyright CyberC3 Copyright (c) 2023
 *
 */
#pragma once
#include <limits>

namespace cyberc3 {
namespace control {

struct PIDConfig {
  double kp = 0;
  double ki = 0;
  double kd = 0;
  double i_max = 0.0;
  double out_max = 0.0;
  double i_err_range = std::numeric_limits<double>::max();
};

class PIDController {
 public:
  PIDController() = default;
  ~PIDController() = default;
  void Init(const double kp, const double ki, const double kd,
            const double out_max, const double out_min,
            const double integral_max, const double integral_min,
            const double i_err_range);
  void Init(const PIDConfig &config);
  double Update(const double error, const double dt);
  void Reset();
  void SetPID(const double kp, const double ki, const double kd);
  void SetKp(const double kp);
  void SetSaturation(const double out_max, const double out_min,
                     const double integral_max, const double integral_min,
                     const double i_err_range);
  void SetConfig(const PIDConfig &config);

 private:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double out_max_ = 0.0;
  double out_min_ = 0.0;
  double integral_max_ = 0.0;
  double integral_min_ = 0.0;
  double i_err_range_ = std::numeric_limits<double>::max();
  double integral_ = 0.0;
  double err_last_ = 0.0;
  bool first_run_ = true;
};
}  // namespace control
}  // namespace cyberc3
