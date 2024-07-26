/**
 * @file longitudinal_controller_node.cpp
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief
 * @version 0.1
 * @date 2023-03-04
 *
 * @copyright CyberC3 Copyright (c) 2023
 *
 */
#include "lon_controller_node.h"

#include "longitudinal_controller/lon_controllerConfig.h"

#include "cyber_msgs/LongitudinalControlDebug.h"
#include "cyber_msgs/brakecmd.h"

#include "common/math/linear_interpolation.h"

namespace cyberc3 {
namespace control {

LongitudinalControllerNode::LongitudinalControllerNode(ros::NodeHandle *nh)
    : nh_(nh) {
  // 运行时可更改参数
  // 配置动态更改参数服务
  auto cb =
      boost::bind(&LongitudinalControllerNode::configCallback, this, _1, _2);
  dr_srv.setCallback(cb);

  // 配置默认参数
  nh_->param("ts", ts_, 0.02);
  nh_->param("debug_msg", enable_debug_msg_, true);
  nh_->param("timeout_threshold", timeout_threshold_, 1.0);

  nh_->param("distance_kp", distance_pid_config_.kp, 0.75);
  nh_->param("distance_ki", distance_pid_config_.ki, 0.0);
  nh_->param("distance_kd", distance_pid_config_.kd, 0.0);
  nh_->param("distance_i_max", distance_pid_config_.i_max, 0.5);
  nh_->param("distance_out_max", distance_pid_config_.out_max, 5.0);
  nh_->param("distance_i_err_range", distance_pid_config_.i_err_range, 2.0);

  nh_->param("speed_kp", speed_pid_config_.kp, 1.0);
  nh_->param("speed_ki", speed_pid_config_.ki, 0.0);
  nh_->param("speed_kd", speed_pid_config_.kd, 0.0);
  nh_->param("speed_i_max", speed_pid_config_.i_max, 0.5);
  nh_->param("speed_out_max", speed_pid_config_.out_max, 5.0);
  nh_->param("speed_i_err_range", distance_pid_config_.i_err_range, 3.0);
  nh_->param("speed_max", speed_max_, 40.0);  // km/h
  nh_->param("enable_speed_limit_curvature", enable_speed_limit_curvature_,
             false);
  nh_->param("enable_acc_limit_curvature", enable_acc_limit_curvature_, false);
  nh_->param("enable_speed_limit_steer", enable_speed_limit_steer_, false);
  nh_->param("enable_acc_limit_steer", enable_acc_limit_steer_, false);

  nh_->param("stop_ref_speed_threshold",
             stop_state_config_.stop_ref_speed_threshold, 0.1);
  nh_->param("stop_ref_acc_threshold",
             stop_state_config_.stop_ref_acc_threshold, 0.1);
  nh_->param("stop_acc_cmd", stop_state_config_.stop_acc_cmd, -1.0);

  std::cout << "distance kp : " << distance_pid_config_.kp << std::endl;
  std::cout << "distance ki : " << distance_pid_config_.ki << std::endl;
  std::cout << "distance kd : " << distance_pid_config_.kd << std::endl;
  std::cout << "speed kp : " << speed_pid_config_.kp << std::endl;
  std::cout << "speed ki : " << speed_pid_config_.ki << std::endl;
  std::cout << "speed kd : " << speed_pid_config_.kd << std::endl;
  std::cout << "speed max : " << speed_max_ << std::endl;

  distance_pid_controller_.Init(distance_pid_config_);
  speed_pid_controller_.Init(speed_pid_config_);

  sub_vel_ = nh_->subscribe("/rock_can/speed_feedback", 1,
                            &LongitudinalControllerNode::SpeedCallback, this);
  sub_target_ =
      nh_->subscribe("/control/control_target", 1,
                     &LongitudinalControllerNode::TargetCallback, this);

  pub_speed_cmd_ =
      nh_->advertise<cyber_msgs::speedcmd>("/rock_can/speed_command", 1);
  pub_brake_cmd_ =
      nh_->advertise<cyber_msgs::brakecmd>("/rock_can/brake_command", 1);
  timer_ = nh_->createTimer(ros::Duration(ts_),
                            &LongitudinalControllerNode::TimerCallback, this);
  if (enable_debug_msg_) {
    pub_debug_msg_ =
        nh_->advertise<cyber_msgs::LongitudinalControlDebug>("/lon_debug", 1);
  }
  ros::spin();
}

void LongitudinalControllerNode::Reset() {
  target_valid_ = false;
  feedback_valid_ = false;
  distance_pid_controller_.Reset();
  speed_pid_controller_.Reset();
  target_ = cyber_msgs::PlatoonControlTarget();

  // pub brake command to stop
  cyber_msgs::brakecmd brake_cmd;
  brake_cmd.enable_auto_brake = true;
  brake_cmd.deceleration = -3.0;
  pub_brake_cmd_.publish(brake_cmd);
}

void LongitudinalControllerNode::SpeedCallback(
    const cyber_msgs::SpeedFeedback &msg) {
  can_feedback_ = msg;
  speed_feedback_ = msg.speed_cms / 100.0;  // m/s
  feedback_valid_ = true;
}

void LongitudinalControllerNode::TargetCallback(
    const cyber_msgs::PlatoonControlTarget &msg) {
  if (msg.header.stamp.isZero()) {
    ROS_ERROR("Empty Target Received! Reset Controller Now!");
    Reset();
    return;
  }
  target_ = msg;
  target_valid_ = true;
}

void LongitudinalControllerNode::TimerCallback(const ros::TimerEvent &) {
  if (!target_valid_) {
    ROS_ERROR_THROTTLE(1, "Speed Target Not Received!");
    return;
  }
  if (!feedback_valid_) {
    ROS_ERROR_THROTTLE(1, "Speed Feedback Not Received!");
    return;
  }
  if (ros::Time::now().toSec() - target_.header.stamp.toSec() >
      timeout_threshold_) {
    ROS_ERROR("Speed Target Timeout, Reset Now!");
    Reset();
    return;
  }
  if (ros::Time::now().toSec() - can_feedback_.header.stamp.toSec() >
      timeout_threshold_) {
    ROS_ERROR("Speed Feedback Timeout, Reset Now!");
    Reset();
    return;
  }

  double distance_err = -(target_.distance_ref - target_.distance_feedback);
  double speed_ref_ff = target_.speed_ref;
  double acc_cmd_ff = target_.acc_ref;

  if (enable_distance_kp_gain_schedule_) {
    const double kp_gain = common::math::interpolate1d(
        distance_err, gain_ref_distance_errors_, distance_kp_gains_);
    distance_pid_controller_.SetKp(kp_gain * distance_pid_config_.kp);
  }
  double target_acc_gain = 1.0;
  if (enable_feedforward_acc_gain_schedule_) {
    target_acc_gain = common::math::interpolate1d(
        acc_cmd_ff, gain_ref_target_acc_, feedforward_acc_gains_);
  }

  double speed_ref_fb = distance_pid_controller_.Update(distance_err, ts_);

  double speed_ref = speed_ref_fb + speed_ref_ff;

  // only consider forward speed now
  if (speed_ref > speed_max_ / 3.6) {
    speed_ref = speed_max_;
  } else if (speed_ref < 0.0) {
    speed_ref = 0.0;
  }

  if (enable_speed_limit_curvature_) {
    if (speed_ref > target_.speed_limit_curvature) {
      speed_ref = target_.speed_limit_curvature;
      // reset pid controller to avoid integral windup
      distance_pid_controller_.Reset();
      speed_pid_controller_.Reset();
    }
  }

  if (enable_speed_limit_steer_) {
    if (speed_ref > target_.speed_limit_steer) {
      speed_ref = target_.speed_limit_steer;
      // reset pid controller to avoid integral windup
      distance_pid_controller_.Reset();
      speed_pid_controller_.Reset();
    }
  }

  double speed_err = speed_ref - speed_feedback_;

  double acc_cmd_fb = speed_pid_controller_.Update(speed_err, ts_);

  // TODO: add gravity compensation based on vehicle pitch angle
  double acc_cmd = acc_cmd_fb + acc_cmd_ff * target_acc_gain;
  // TODO: add filters to acc_cmd to avoid sharp movement

  if (enable_acc_limit_curvature_) {
    if (acc_cmd > target_.acc_limit_curvature) {
      acc_cmd = target_.acc_limit_curvature;
      // reset pid controller to avoid integral windup
      distance_pid_controller_.Reset();
      speed_pid_controller_.Reset();
    }
  }

  if (enable_acc_limit_steer_) {
    if (acc_cmd > target_.acc_limit_steer) {
      acc_cmd = target_.acc_limit_steer;
      // reset pid controller to avoid integral windup
      distance_pid_controller_.Reset();
      speed_pid_controller_.Reset();
    }
  }

  // check for vehicle stop state and set stop acc cmd
  if (std::fabs(speed_ref_ff) < stop_state_config_.stop_ref_speed_threshold &&
      std::fabs(acc_cmd_ff) < stop_state_config_.stop_ref_acc_threshold) {
    acc_cmd = std::min(acc_cmd, stop_state_config_.stop_acc_cmd);
    distance_pid_controller_.Reset();
    speed_pid_controller_.Reset();
  }

  cyber_msgs::speedcmd throttle_cmd;
  cyber_msgs::brakecmd brake_cmd;

  MapAccToCmd(acc_cmd, &throttle_cmd, &brake_cmd);

  pub_speed_cmd_.publish(throttle_cmd);
  pub_brake_cmd_.publish(brake_cmd);

  std::cout << "D_err: " << distance_err << " V_err: " << speed_err
            << std::endl;
  std::cout << "V_ref: " << speed_ref << " V_cur: " << speed_feedback_
            << std::endl;
  std::cout << "acc_cmd : " << acc_cmd << " acc_ff: " << acc_cmd_ff
            << " acc_fb: " << acc_cmd_fb << std::endl;
  std::cout << "speed_cmd: " << throttle_cmd.speed_cmd
            << " brake: " << brake_cmd.deceleration << std::endl;
  std::cout << "brake_status : "
            << static_cast<int>(brake_cmd.enable_auto_brake)
            << " gear : " << static_cast<int>(throttle_cmd.gear) << std::endl;

  if (enable_debug_msg_) {
    cyber_msgs::LongitudinalControlDebug debug_msg;
    debug_msg.header.stamp = ros::Time::now();
    debug_msg.distance_err = distance_err;
    debug_msg.speed_ref_feedback = speed_ref_fb;
    debug_msg.speed_ref_feedforward = speed_ref_ff;
    debug_msg.speed_ref = speed_ref;
    debug_msg.speed_feedback = speed_feedback_;
    debug_msg.speed_err = speed_err;
    debug_msg.acc_cmd_feedback = acc_cmd_fb;
    debug_msg.acc_cmd_feedforward = acc_cmd_ff * target_acc_gain;
    debug_msg.acc_cmd = acc_cmd;
    debug_msg.throttle_cmd = throttle_cmd.speed_cmd;
    debug_msg.brake_cmd = brake_cmd.deceleration;
    pub_debug_msg_.publish(debug_msg);
  }
}
void LongitudinalControllerNode::MapAccToCmd(const double target_acc,
                                             cyber_msgs::speedcmd *speed_cmd,
                                             cyber_msgs::brakecmd *brake_cmd) {
  constexpr int THROTTLE_MAX = 3000;
  constexpr int THROTTLE_MIN = 0;
  constexpr double BRAKE_MAX = 0;
  constexpr double BRAKE_MIN = -5.0;
  double k0 = 51.45;
  double k1 = 794.8;

  double throttle_cmd = k0 + k1 * target_acc;
  if (throttle_cmd > 0) {
    if (throttle_cmd > THROTTLE_MAX) {
      throttle_cmd = THROTTLE_MAX;
    } else if (throttle_cmd < THROTTLE_MIN) {
      throttle_cmd = THROTTLE_MIN;
    }
    speed_cmd->speed_cmd = throttle_cmd;
    speed_cmd->gear = vehicle_gear_cmd::D;
    speed_cmd->enable_auto_speed = true;
    speed_cmd->is_updated = true;

    brake_cmd->deceleration = 0;
    brake_cmd->enable_auto_brake = false;
  } else {
    double brake_acc = target_acc;
    if (brake_acc > BRAKE_MAX) {
      brake_acc = BRAKE_MAX;
    } else if (brake_acc < BRAKE_MIN) {
      brake_acc = BRAKE_MIN;
    }
    speed_cmd->speed_cmd = 0;
    speed_cmd->enable_auto_speed = false;
    speed_cmd->is_updated = true;

    brake_cmd->deceleration = brake_acc;
    brake_cmd->enable_auto_brake = true;
  }
}

void LongitudinalControllerNode::configCallback(
    longitudinal_controller::lon_controllerConfig &config, uint32_t level) {
  if (config.update_param_) {
    distance_pid_config_.kp = config.distance_kp;
    distance_pid_config_.ki = config.distance_ki;
    distance_pid_config_.kd = config.distance_kd;
    distance_pid_config_.i_max = config.distance_i_max;
    distance_pid_config_.out_max = config.distance_out_max;

    speed_pid_config_.kp = config.speed_kp;
    speed_pid_config_.ki = config.speed_ki;
    speed_pid_config_.kd = config.speed_kd;
    speed_pid_config_.i_max = config.speed_i_max;
    speed_pid_config_.out_max = config.speed_out_max;
    speed_max_ = config.speed_max;

    distance_pid_controller_.SetConfig(distance_pid_config_);
    speed_pid_controller_.SetConfig(speed_pid_config_);

    std::cout << "----------reconfigure----------" << std::endl;
    std::cout << "distance_kp\t" << config.distance_kp << std::endl;
    std::cout << "distance_ki\t" << config.distance_ki << std::endl;
    std::cout << "distance_kd\t" << config.distance_kd << std::endl;
    std::cout << "speed_kp\t" << config.speed_kp << std::endl;
    std::cout << "speed_ki\t" << config.speed_ki << std::endl;
    std::cout << "speed_kd\t" << config.speed_kd << std::endl;
    config.update_param_ = false;
  }
}
}  // namespace control
}  // namespace cyberc3

int main(int argc, char **argv) {
  ros::init(argc, argv, "lon_controller_node");
  ros::NodeHandle pnh("~");

  cyberc3::control::LongitudinalControllerNode LongitudinalControllerNode_obj(
      &pnh);

  return 0;
}
