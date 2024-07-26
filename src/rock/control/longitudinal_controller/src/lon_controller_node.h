/**
 * @file longitudinal_controller_node.h
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief
 * @version 0.1
 * @date 2023-03-04
 *
 * @copyright CyberC3 Copyright (c) 2023
 *
 */
#pragma once

#include <algorithm>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <longitudinal_controller/lon_controllerConfig.h>

#include "cyber_msgs/PlatoonControlTarget.h"
#include "cyber_msgs/SpeedFeedback.h"
#include "cyber_msgs/brakecmd.h"
#include "cyber_msgs/speedcmd.h"

#include "pid_controller.h"

namespace cyberc3 {
namespace control {

namespace vehicle_gear_feedback {
constexpr signed char R = 9;
constexpr signed char D = 11;
constexpr signed char P = 10;
constexpr signed char N = 0;
}  // namespace vehicle_gear_feedback

namespace vehicle_gear_cmd {
constexpr signed char D = 1;
constexpr signed char R = 2;
}  // namespace vehicle_gear_cmd

class LongitudinalControllerNode {
 public:
  LongitudinalControllerNode(ros::NodeHandle *nh);
  ~LongitudinalControllerNode() = default;
  void configCallback(longitudinal_controller::lon_controllerConfig &config,
                      uint32_t level);

 private:
  ros::NodeHandle *nh_;
  ros::Subscriber sub_vel_;
  ros::Subscriber sub_target_;
  ros::Publisher pub_speed_cmd_;
  ros::Publisher pub_brake_cmd_;
  ros::Publisher pub_debug_msg_;
  ros::Timer timer_;
  PIDController distance_pid_controller_;
  PIDController speed_pid_controller_;
  PIDConfig distance_pid_config_;
  PIDConfig speed_pid_config_;
  struct stop_state_config {
    double stop_ref_speed_threshold = 0.15;
    double stop_ref_acc_threshold = 0.1;
    double stop_acc_cmd = -1.0;
  } stop_state_config_;
  double ts_ = 0.02;
  bool enable_debug_msg_ = true;
  double timeout_threshold_ = 1.0;
  bool enable_speed_limit_curvature_ = false;
  bool enable_acc_limit_curvature_ = false;
  bool enable_speed_limit_steer_ = false;
  bool enable_acc_limit_steer_ = false;

  bool target_valid_ = false;
  bool feedback_valid_ = false;
  double speed_feedback_ = 0;
  double speed_max_ = 0;
  cyber_msgs::PlatoonControlTarget target_;
  cyber_msgs::SpeedFeedback can_feedback_;
  bool enable_distance_kp_gain_schedule_ = true;
  std::vector<double> gain_ref_distance_errors_ = {-2.0, -1.0, -0.5, -0.25,
                                                   0.0};
  std::vector<double> distance_kp_gains_ = {5.0, 3.0, 2.0, 1.25, 1.0};
  bool enable_feedforward_acc_gain_schedule_ = true;
  std::vector<double> gain_ref_target_acc_ = {-2.0, -1.0, -0.5, -0.25, 0.0};
  std::vector<double> feedforward_acc_gains_ = {2.0, 1.75, 1.5, 1.2, 1.0};

  //---------参数服务相关变量------------
  dynamic_reconfigure::Server<longitudinal_controller::lon_controllerConfig>
      dr_srv;

  void Reset();
  void SpeedCallback(const cyber_msgs::SpeedFeedback &msg);
  void TargetCallback(const cyber_msgs::PlatoonControlTarget &msg);
  void TimerCallback(const ros::TimerEvent &);
  // map acceleration target to corresponding throttle and brake command
  void MapAccToCmd(const double target_acc, cyber_msgs::speedcmd *speed_cmd,
                   cyber_msgs::brakecmd *brake_cmd);
};
}  // namespace control
}  // namespace cyberc3
